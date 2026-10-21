using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Bounded geometric improvement of a complete MoveL route. Dense joint
    /// samples remain intact; each proposed replacement is fully replanned/validated.
    /// Failed or unfinished proposals leave the best complete route unchanged.</summary>
    public sealed class CartesianLinearPathOptimizer
    {
        private readonly World world;
        private readonly CartesianLinearPlanner linear;
        public CartesianLinearPathOptimizer(World world, CartesianLinearPlanner linear)
        { this.world = world; this.linear = linear; }

        internal static float RouteLength(Vector3 start, IReadOnlyList<Vector3> waypoints, Vector3 end)
        {
            float length = 0;
            Vector3 previous = start;
            foreach (Vector3 point in waypoints) { length += Vector3.Distance(previous, point); previous = point; }
            return length + Vector3.Distance(previous, end);
        }

        public void Optimize(float[] start, float[] goal, Vector3 a, Quaternion ar, Vector3 b, Quaternion br, Result best)
        {
            if (best.JointPath.Count < 2 || best.AutomaticWaypoints.Count == 0) return;
            var timer = System.Diagnostics.Stopwatch.StartNew();
            bool Stop() => timer.Elapsed.TotalSeconds >= Math.Max(0, world.LinearOptimizationSeconds) ||
                (world.OptimizationShouldStop != null && world.OptimizationShouldStop());
            int attempts = 0;
            float bestLength = RouteLength(a, best.AutomaticWaypoints, b);
            bool Improve(List<Vector3> nodes)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                float length = RouteLength(a, nodes, b);
                if (Stop() || attempts >= 12 || length >= bestLength - 0.0005f) return false;
                attempts++;
                var candidate = new Result();
                candidate.JointPath.Add((float[])start.Clone());
                float[] from = start;
                Vector3 fromPosition = a;
                Quaternion fromRotation = ar;
                for (int i = 0; i <= nodes.Count; i++)
                {
                    if (Stop()) return false;
                    bool final = i == nodes.Count;
                    Vector3 position = final ? b : nodes[i];
                    Vector3 axis = b - a;
                    float t = axis.sqrMagnitude < 1e-10f ? 0f : Mathf.Clamp01(Vector3.Dot(position - a, axis) / axis.sqrMagnitude);
                    Quaternion rotation = final ? br : Quaternion.Slerp(ar, br, t);
                    world.GetTcpPosition(from);
                    float[] endpoint = goal;
                    if (!final)
                    {
                        var solutions = world.SolveIk(position, rotation);
                        if (solutions == null || solutions.Count == 0) return false;
                        endpoint = solutions[0];
                    }
                    var edge = new Result();
                    bool valid = linear.TryPlanDirect(from, endpoint, fromPosition, fromRotation,
                        position, rotation, edge, poseGoal: !final, refinements: 2, shouldAbort: Stop);
                    best.RoutesTried += edge.RoutesTried; best.Iterations += edge.Iterations;
                    if (!valid) return false;
                    for (int j = 1; j < edge.JointPath.Count; j++) candidate.JointPath.Add(edge.JointPath[j]);
                    from = edge.JointPath[edge.JointPath.Count - 1];
                    fromPosition = position; fromRotation = rotation;
                }
                best.JointPath.Clear(); best.JointPath.AddRange(candidate.JointPath);
                best.AutomaticWaypoints.Clear(); best.AutomaticWaypoints.AddRange(nodes);
                bestLength = length;
                return true;
            }

            // Test tight, low corridors around the combined obstacle silhouette.
            // Unlike the initial large-apex seeds, these follow clearance boundaries.
            if (world.LinearRoutingObstacles != null && world.LinearRoutingObstacles.Length > 0)
            {
                Bounds obstacle = world.LinearRoutingObstacles[0];
                foreach (Bounds box in world.LinearRoutingObstacles) obstacle.Encapsulate(box);
                foreach (float extra in new[] { 0.01f, 0.04f, 0.1f })
                {
                    if (Stop()) break;
                    Bounds inflated = obstacle; inflated.Expand(2 * (world.Clearance + extra));
                    float height = Mathf.Max(inflated.max.y, Mathf.Max(a.y, b.y));
                    // Follow the top corners, or rise/traverse/descend directly above
                    // the endpoints like the requested low rectangular route.
                    float Clamp(float x, float lo, float hi) => Math.Max(lo, Math.Min(hi, x));
                    Improve(new List<Vector3> {
                        new Vector3(Clamp(a.x,inflated.min.x,inflated.max.x),height,Clamp(a.z,inflated.min.z,inflated.max.z)),
                        new Vector3(Clamp(b.x,inflated.min.x,inflated.max.x),height,Clamp(b.z,inflated.min.z,inflated.max.z)) });
                    Improve(new List<Vector3> { new Vector3(a.x,height,a.z), new Vector3(b.x,height,b.z) });
                }
            }
            // Remove redundant generated corners. Authored targets are segment
            // boundaries outside this optimizer and cannot be removed here.
            for (int i = 0; i < best.AutomaticWaypoints.Count && !Stop() && attempts < 12;)
            {
                var nodes = new List<Vector3>(best.AutomaticWaypoints); nodes.RemoveAt(i);
                // The original direct line already failed; avoid retesting it.
                if (nodes.Count == 0 || !Improve(nodes)) i++;
            }
            // Contract excess detour toward the original line, retaining only safe
            // improvements. A rejected contraction never replaces the valid route.
            for (int i = 0; i < best.AutomaticWaypoints.Count && !Stop() && attempts < 12; i++)
            {
                Vector3 original = best.AutomaticWaypoints[i];
                Vector3 delta = b - a;
                float t = delta.sqrMagnitude < 1e-10f ? 0f : Mathf.Clamp01(Vector3.Dot(original - a, delta) / delta.sqrMagnitude);
                Vector3 projection = Vector3.Lerp(a, b, t);
                foreach (float fraction in new[] { 0.5f, 0.25f })
                {
                    var nodes = new List<Vector3>(best.AutomaticWaypoints);
                    nodes[i] = Vector3.Lerp(original, projection, fraction);
                    if (Improve(nodes)) break;
                }
            }
        }
    }
}
