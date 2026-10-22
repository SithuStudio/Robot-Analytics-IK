using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>26-connected occupancy guidance followed by strict MoveL validation.</summary>
    public sealed class CartesianGridWaypointSearch
    {
        private readonly World world;
        private readonly CartesianLinearPlanner linear;
        public CartesianGridWaypointSearch(World world,CartesianLinearPlanner linear) { this.world=world; this.linear=linear; }
        private struct Entry : IComparable<Entry>
        {
            public int Id,Serial; public float Score,Cost;
            public int CompareTo(Entry other) { int c=Score.CompareTo(other.Score); return c!=0?c:Serial.CompareTo(other.Serial); }
        }
        private sealed class RejectedRoute
        {
            public string Reason;
            public Vector3 Position,EdgeStart;
            public bool InvalidStart;
        }
        public bool TryPlan(float[] start,float[] goal,Vector3 a,Quaternion ar,Vector3 b,Quaternion br,Result result)
        {
            var grid=world.LinearOccupancyGrid;
            var timer=System.Diagnostics.Stopwatch.StartNew();
            double begun=world.PlanningElapsedSeconds?.Invoke()??0;
            bool Stop()=> (world.PlanningElapsedSeconds==null?timer.Elapsed.TotalSeconds:world.PlanningElapsedSeconds()-begun)>=world.LinearWaypointSearchSeconds;
            var penalties=new Dictionary<int,float>();
            var rejectedRoutes=new Dictionary<string,RejectedRoute>();
            int checks=0, expanded=0, attempts=0;
            string failure="No free grid route.";
            string stopped="Route attempt limit reached.";
            int maximumAttempts=Math.Max(1,world.LinearGridMaximumRouteAttempts);
            for(int attempt=0;attempt<maximumAttempts&&!Stop();attempt++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                var route=FindRoute(grid,a,b,penalties,Stop,ref expanded);
                if(route==null)
                {
                    stopped=Stop()?"Routing time budget exhausted.":expanded>=world.LinearGridMaximumNodes?
                        "Grid node expansion budget exhausted.":"No traversable route was produced by the scanned-grid search.";
                    result.RoutingDiagnostics+=stopped+"\n"; break;
                }
                attempts++;
                var joints=new List<float[]> { (float[])start.Clone() };
                float[] from=start; Vector3 previous=a; Quaternion previousRotation=ar;
                bool valid=true, invalidStart=false; Vector3 rejected=a, rejectedEdgeStart=a;
                var routeIds=new List<int>();
                foreach(Vector3 p in route) { grid.TryCell(p,out int id); routeIds.Add(id); }
                string key=string.Join(",",routeIds);
                bool cached=rejectedRoutes.TryGetValue(key,out var prior);
                if(cached)
                {
                    valid=false; failure=prior.Reason; rejected=prior.Position;
                    rejectedEdgeStart=prior.EdgeStart; invalidStart=prior.InvalidStart;
                    result.RoutingDiagnostics+="Repeated corridor: reused its previous validation rejection without repeating scene queries.\n";
                }
                for(int i=1;!cached&&i<route.Count;i++)
                {
                    if(Stop() || checks>=world.LinearWaypointMaximumEdges) { failure="Grid MoveL validation budget exhausted."; valid=false; break; }
                    checks++;
                    Vector3 position=route[i], axis=b-a;
                    float fraction=axis.sqrMagnitude<1e-10f?0:Mathf.Clamp01(Vector3.Dot(position-a,axis)/axis.sqrMagnitude);
                    bool final=i==route.Count-1;
                    Quaternion rotation=final?br:Quaternion.Slerp(ar,br,fraction);
                    var edge=new Result();
                    // Pose-goal segments let the existing branch search find a continuous
                    // IK solution; no independently selected intermediate joint target.
                    bool accepted=linear.TryPlanDirect(from,goal,previous,previousRotation,position,rotation,edge,
                        // Use the same adaptive 1/2/4/8 sampling passes as direct
                        // MoveL. Collision/singularity failures still stop immediately;
                        // only continuity/accuracy failures request finer samples.
                        poseGoal:!final,refinements:4,shouldAbort:Stop);
                    result.RoutesTried+=edge.RoutesTried; result.Iterations+=edge.Iterations;
                    if(!accepted)
                    {
                        failure=edge.FailureReason; rejected=edge.HasFailurePosition?edge.FailurePosition:position;
                        rejectedEdgeStart=previous;
                        invalidStart=i==1 && Vector3.Distance(rejected,a)<0.001f &&
                            failure!=null && failure.StartsWith("MoveL endpoint rejected:");
                        result.FailurePosition=rejected; result.HasFailurePosition=true; valid=false; break;
                    }
                    for(int j=1;j<edge.JointPath.Count;j++) joints.Add(edge.JointPath[j]);
                    from=edge.JointPath[edge.JointPath.Count-1]; previous=position; previousRotation=rotation;
                }
                if(valid)
                {
                    result.JointPath.Clear(); result.JointPath.AddRange(joints);
                    result.AutomaticWaypoints.Clear();
                    for(int i=1;i<route.Count-1;i++) result.AutomaticWaypoints.Add(route[i]);
                    result.FailureReason=null; result.HasFailurePosition=false;
                    result.RoutingDiagnostics+=$"Scanner grid route accepted: {expanded} expanded cells, {checks} MoveL edges, {attempt+1} route attempts.\n";
                    return true;
                }
                result.RoutingDiagnostics+=$"Scanner route {attempt+1} rejected: {failure}\n";
                if(!cached) rejectedRoutes[key]=new RejectedRoute{Reason=failure,Position=rejected,EdgeStart=rejectedEdgeStart,InvalidStart=invalidStart};
                if(invalidStart) { stopped="The fixed starting robot configuration is invalid; another corridor cannot repair it."; break; }
                if(checks>=world.LinearWaypointMaximumEdges) { stopped="MoveL edge budget exhausted."; break; }
                if(Stop()) { stopped="Routing time budget exhausted."; break; }
                if(attempt+1>=maximumAttempts) break;
                if(!grid.TryCell(a,out int first)||!grid.TryCell(b,out int last)) break;
                // Protect the endpoints: a failed arrival branch does not mean the
                // goal pose is impossible. Divert the incoming corridor instead.
                bool Usable(Vector3 point,out int id) => grid.TryCell(point,out id) && id!=first && id!=last && grid.IsFree(point);
                int blocked;
                if(!Usable(rejected,out blocked))
                {
                    bool found=false;
                    foreach(float fraction in new[]{0.9f,0.75f,0.5f,0.25f})
                        if(Usable(Vector3.Lerp(rejectedEdgeStart,rejected,fraction),out blocked)) { found=true; break; }
                    // A short final edge may lie entirely in the goal voxel. Its
                    // arrival configuration can still change with an earlier detour.
                    if(!found)
                        for(int j=route.Count-2;j>0;j--)
                            if(Usable(route[j],out blocked)) { found=true; break; }
                    if(!found) { stopped="No distinct interior corridor cell is available to divert this transition."; break; }
                }
                grid.Cell(blocked,out int bx,out int by,out int bz);
                int radius=Math.Min(2,attempt/3), added=0;
                for(int dx=-radius;dx<=radius;dx++)for(int dy=-radius;dy<=radius;dy++)for(int dz=-radius;dz<=radius;dz++)
                {
                    int x=bx+dx,y=by+dy,z=bz+dz;
                    if(!grid.IsFree(x,y,z)) continue;
                    int id=grid.Index(x,y,z);
                    if(id==first||id==last) continue;
                    penalties.TryGetValue(id,out float previousPenalty);
                    penalties[id]=previousPenalty+8f*(1+attempt/3); added++;
                }
                result.RoutingDiagnostics+=$"Searching another corridor: increased route costs in {added} cells near the failed transition (radius {radius}); scanned free space and endpoints retained.\n";
            }
            result.JointPath.Clear(); result.AutomaticWaypoints.Clear();
            if(Stop()) stopped="Routing time budget exhausted.";
            result.FailureReason=$"Scanner-guided MoveL failed ({expanded} expanded cells, {checks} edges, {attempts}/{maximumAttempts} route attempts): {failure} {stopped} No partial path will execute.";
            return false;
        }

        private List<Vector3> FindRoute(CartesianOccupancyGrid grid,Vector3 a,Vector3 b,Dictionary<int,float> penalties,Func<bool> stop,ref int expanded)
        {
            if(!grid.TryCell(a,out int first)||!grid.TryCell(b,out int last)||!grid.IsFree(a)||!grid.IsFree(b)) return null;
            var open=new SortedSet<Entry>(); var cost=new Dictionary<int,float>{{first,0}}; var parent=new Dictionary<int,int>();
            int serial=0; open.Add(new Entry{Id=first,Score=Vector3.Distance(a,b),Serial=serial++});
            bool reached=false;
            while(open.Count>0 && expanded<world.LinearGridMaximumNodes && !stop())
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                Entry current=open.Min; open.Remove(current);
                if(current.Cost>cost[current.Id]) continue;
                expanded++;
                if(current.Id==last) { reached=true; break; }
                grid.Cell(current.Id,out int x,out int y,out int z); Vector3 p=grid.Center(current.Id);
                for(int dx=-1;dx<=1;dx++) for(int dy=-1;dy<=1;dy++) for(int dz=-1;dz<=1;dz++)
                {
                    if(dx==0&&dy==0&&dz==0 || !grid.IsFree(x+dx,y+dy,z+dz)) continue;
                    int id=grid.Index(x+dx,y+dy,z+dz); Vector3 next=grid.Center(id);
                    int mask=(dx!=0?1:0)|(dy!=0?2:0)|(dz!=0?4:0);
                    bool clear=true;
                    for(int subset=1;subset<8;subset++)
                        if((subset&mask)==subset && !grid.IsFree(x+((subset&1)!=0?dx:0),y+((subset&2)!=0?dy:0),z+((subset&4)!=0?dz:0)))
                        { clear=false; break; }
                    if(!clear) continue;
                    penalties.TryGetValue(current.Id,out float fromPenalty); penalties.TryGetValue(id,out float toPenalty);
                    float candidate=current.Cost+Vector3.Distance(p,next)*(1+(fromPenalty+toPenalty)*.5f);
                    if(cost.TryGetValue(id,out float known)&&known<=candidate) continue;
                    cost[id]=candidate; parent[id]=current.Id;
                    open.Add(new Entry{Id=id,Cost=candidate,Score=candidate+Vector3.Distance(next,grid.Center(last)),Serial=serial++});
                }
            }
            if(!reached) return null;
            var reversed=new List<int>{last};
            while(reversed[reversed.Count-1]!=first) reversed.Add(parent[reversed[reversed.Count-1]]);
            reversed.Reverse(); var raw=new List<Vector3>{a};
            foreach(int id in reversed) raw.Add(grid.Center(id)); raw.Add(b);
            var prefixCost=new float[raw.Count];
            for(int i=1;i<raw.Count;i++) prefixCost[i]=prefixCost[i-1]+grid.SegmentCost(raw[i-1],raw[i],penalties);
            var simplified=new List<Vector3>{a};
            for(int i=0;i<raw.Count-1;)
            {
                world.CancellationToken.ThrowIfCancellationRequested(); if(stop()) return null;
                int next=raw.Count-1;
                // Do not shortcut straight back through the costly rejected region
                // and undo the alternative corridor selected by A*.
                while(next>i+1&&(!grid.SegmentFree(raw[i],raw[next]) ||
                    grid.SegmentCost(raw[i],raw[next],penalties)>prefixCost[next]-prefixCost[i]+0.0001f))
                {
                    world.CancellationToken.ThrowIfCancellationRequested();
                    if(stop()) return null;
                    next--;
                }
                if(!grid.SegmentFree(raw[i],raw[next])) return null;
                simplified.Add(raw[next]); i=next;
            }
            return simplified;
        }
    }
}
