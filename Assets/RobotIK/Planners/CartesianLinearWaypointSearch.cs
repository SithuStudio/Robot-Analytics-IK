using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Bounded best-first Cartesian roadmap search. Every accepted edge is a
    /// strict MoveL with full IK, collision, singularity and interpolation validation.
    /// Occupancy is only a cheap rejection filter; TCP clearance alone never accepts an edge.
    /// Call synchronously on the same thread as the supplied physics/FK services.</summary>
    public sealed class CartesianLinearWaypointSearch
    {
        private readonly World world;
        private readonly CartesianLinearPlanner linear;
        private int initialEscapeEnd;
        private sealed class State
        {
            public int Point;
            public float[] Angles;
            public float Cost;
            public State Parent;
            public List<float[]> Edge;
            public List<int> Targets;
            public int NextTarget, Expansions;
            public bool LocalProbesPrioritized;
        }

        public CartesianLinearWaypointSearch(World world, CartesianLinearPlanner linear)
        {
            this.world = world ?? throw new ArgumentNullException(nameof(world));
            this.linear = linear ?? throw new ArgumentNullException(nameof(linear));
        }

        public bool TryPlan(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            if (world.LinearForceLiftTraverseDescend)
                return TryLiftTraverseDescend(start, goal, a, ar, b, br, result);
            // An invalid initial robot pose cannot be repaired by inserting later points.
            if (result.FailureReason != null && result.FailureReason.StartsWith("MoveL endpoint rejected:")) return false;
            string directFailure = result.FailureReason;
            var points = CreateCandidates(a, b, result.HasFailurePosition ? result.FailurePosition : (a + b) * 0.5f);
            var rotations = new List<Quaternion>();
            Vector3 axis = b - a;
            foreach (Vector3 point in points)
            {
                float t = axis.sqrMagnitude < 1e-10f ? 0f : Mathf.Clamp01(Vector3.Dot(point - a, axis) / axis.sqrMagnitude);
                rotations.Add(Quaternion.Slerp(ar, br, t));
            }
            rotations[0] = ar; rotations[1] = br;
            bool initialEscape = result.HasFailurePosition && Vector3.Distance(a, result.FailurePosition) < Mathf.Min(0.25f, Vector3.Distance(a, b) * 0.25f);
            var open = new List<State> { new State { Point = 0, Angles = (float[])start.Clone(), LocalProbesPrioritized = initialEscape } };
            var visited = new Dictionary<string, float>();
            visited[Key(0, start)] = 0f;
            var timer = System.Diagnostics.Stopwatch.StartNew();
            double budgetStart = world.PlanningElapsedSeconds?.Invoke() ?? 0;
            double BudgetElapsed() => world.PlanningElapsedSeconds == null
                ? timer.Elapsed.TotalSeconds : world.PlanningElapsedSeconds() - budgetStart;
            int tested = 0;
            string lastFailure = null;
            if (world.LinearExploreSpatialCorridors &&
                TrySpatialCorridors(start, goal, a, ar, b, br, result, BudgetElapsed, ref tested, ref lastFailure)) return true;
            if (TryDiagonalRoutes(start, goal, a, ar, b, br, result, BudgetElapsed, ref tested, ref lastFailure, false)) return true;
            if (TryEscapeCorridors(start, goal, a, ar, b, br, result, BudgetElapsed, ref tested, ref lastFailure)) return true;
            if (TryDiagonalRoutes(start, goal, a, ar, b, br, result, BudgetElapsed, ref tested, ref lastFailure, true)) return true;
            while (open.Count > 0)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                int best = 0;
                float Score(State state) => state.Cost + 1.5f * Vector3.Distance(points[state.Point], b) + 0.25f * state.Expansions;
                for (int i = 1; i < open.Count; i++) if (open[i].Point == 1 || (open[best].Point != 1 && Score(open[i]) < Score(open[best]))) best = i;
                State current = open[best]; open.RemoveAt(best);
                if (visited.TryGetValue(Key(current.Point, current.Angles), out float known) && known < current.Cost) continue;
                if (current.Point == 1)
                {
                    var chain = new List<State>();
                    for (State node = current; node.Parent != null; node = node.Parent) chain.Add(node);
                    chain.Reverse();
                    result.JointPath.Clear(); result.AutomaticWaypoints.Clear();
                    result.JointPath.Add((float[])start.Clone());
                    foreach (State node in chain)
                    {
                        for (int i = 1; i < node.Edge.Count; i++) result.JointPath.Add(node.Edge[i]);
                        if (node.Point != 1) result.AutomaticWaypoints.Add(points[node.Point]);
                    }
                    result.FailureReason = null; result.HasFailurePosition = false;
                    return true;
                }
                if (tested >= Math.Max(1, world.LinearWaypointMaximumEdges) ||
                    BudgetElapsed() >= Math.Max(0.1f, world.LinearWaypointSearchSeconds)) break;
                if (current.Targets == null)
                {
                    var neighbours = new List<int>();
                    for (int i = 2; i < points.Count; i++) if (i != current.Point) neighbours.Add(i);
                    neighbours.Sort((x, y) =>
                    {
                        // First probe short escape directions at Start. Goal-biased long
                        // edges can all hit the same wrist/obstacle constraint immediately.
                        if (current.Point == 0 && current.LocalProbesPrioritized)
                        {
                            bool localX = x < initialEscapeEnd, localY = y < initialEscapeEnd;
                            if (localX != localY) return localX ? -1 : 1;
                            if (localX) return Vector3.Distance(a, points[x]).CompareTo(Vector3.Distance(a, points[y]));
                        }
                        return (0.25f * Vector3.Distance(points[current.Point], points[x]) + Vector3.Distance(points[x], b)).CompareTo(
                            0.25f * Vector3.Distance(points[current.Point], points[y]) + Vector3.Distance(points[y], b));
                    });
                    if (current.Point != 0) neighbours.Insert(0, 1); // the original Start -> Goal edge already failed
                    current.Targets = neighbours;
                }
                int attemptsThisExpansion = 0;
                // Resume this frontier later: one difficult node must not consume the
                // entire search budget before another validated node can expand.
                while (current.NextTarget < current.Targets.Count && attemptsThisExpansion < 4)
                {
                    if (tested >= world.LinearWaypointMaximumEdges || BudgetElapsed() >= world.LinearWaypointSearchSeconds) break;
                    int target = current.Targets[current.NextTarget++];
                    if (!TcpLineClear(points[current.Point], points[target])) continue;
                    float cost = current.Cost + Vector3.Distance(points[current.Point], points[target]);
                    if (HasAncestor(current, target)) continue;
                    tested++; attemptsThisExpansion++;
                    world.CancellationToken.ThrowIfCancellationRequested();
                    world.GetTcpPosition(current.Angles); // seed scene-dependent analytical IK
                    float[] endpoint = goal;
                    if (target != 1)
                    {
                        var solutions = world.SolveIk(points[target], rotations[target]);
                        if (solutions == null || solutions.Count == 0) continue;
                        endpoint = solutions[0]; // the strict validator considers all pose-goal branches
                    }
                    var edge = new Result();
                    bool success = linear.TryPlanDirect(current.Angles, endpoint,
                        points[current.Point], rotations[current.Point], points[target], rotations[target], edge,
                        poseGoal: target != 1, refinements: 2,
                        shouldAbort: () => BudgetElapsed() >= world.LinearWaypointSearchSeconds);
                    result.RoutesTried += edge.RoutesTried;
                    result.Iterations += edge.Iterations;
                    if (!success)
                    {
                        lastFailure = edge.FailureReason;
                        continue;
                    }
                    float[] actualEnd = edge.JointPath[edge.JointPath.Count - 1];
                    string key = Key(target, actualEnd);
                    if (visited.TryGetValue(key, out float oldCost) && oldCost <= cost) continue;
                    visited[key] = cost;
                    open.Add(new State { Point = target, Angles = actualEnd, Cost = cost, Parent = current, Edge = edge.JointPath });
                }
                current.Expansions++;
                if (current.NextTarget < current.Targets.Count) open.Add(current);
            }
            // Keep the original straight-line failure marker; later rejected alternatives
            // are search diagnostics, not evidence that the original point became safe.
            result.FailureReason = $"Direct MoveL: {directFailure}\nAutomatic waypoint search stopped after {tested} edge checks in {BudgetElapsed():F2} active s / {timer.Elapsed.TotalSeconds:F2} wall s (limits {world.LinearWaypointMaximumEdges} checks / {world.LinearWaypointSearchSeconds:F1} active s); no validated route found." +
                (lastFailure == null ? "" : "\nLast candidate edge: " + lastFailure);
            return false;
        }

        private static void RouteBasis(Vector3 a, Vector3 b, out Vector3 forward, out Vector3 side, out Vector3 lift)
        {
            Vector3 delta = b - a;
            forward = delta.magnitude > 1e-6f ? delta / delta.magnitude : Vector3.forward;
            side = Vector3.Cross(forward, Vector3.up);
            if (side.sqrMagnitude < 1e-6f) side = Vector3.Cross(forward, Vector3.right);
            side = side / side.magnitude;
            lift = Vector3.Cross(side, forward);
            if (lift.y < 0f) lift = -lift;
        }

        private bool TryLiftTraverseDescend(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            result.RoutingDiagnostics = "Forced lift/traverse/descend mode is active. Select Automatic Free Space to search side and gap routes.\n";
            var timer = System.Diagnostics.Stopwatch.StartNew();
            double started = world.PlanningElapsedSeconds?.Invoke() ?? 0;
            double Elapsed() => world.PlanningElapsedSeconds == null ? timer.Elapsed.TotalSeconds : world.PlanningElapsedSeconds() - started;
            float height = Mathf.Max(a.y, b.y) + Mathf.Max(0.01f, world.LinearMinimumLiftMeters);
            if (world.LinearRoutingObstacles != null)
                foreach (Bounds obstacle in world.LinearRoutingObstacles)
                    if (CrossesHorizontalFootprint(a, b, obstacle, world.Clearance + 0.02f))
                        height = Mathf.Max(height, obstacle.max.y + world.Clearance + 0.02f);
            int tested = 0;
            string lastFailure = null;
            foreach (float extra in new[] { 0f, 0.03f, 0.08f, 0.15f, 0.25f, 0.4f, 0.65f, 1f })
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                var targets = new List<Vector3> {
                    new Vector3(a.x,height + extra,a.z),
                    new Vector3(b.x,height + extra,b.z), b };
                if (TryCandidateRoute(start, goal, a, ar, b, br, targets, result, Elapsed,
                    world.LinearWaypointMaximumEdges, ref tested, ref lastFailure)) return true;
                if (tested >= world.LinearWaypointMaximumEdges || Elapsed() >= world.LinearWaypointSearchSeconds) break;
            }
            result.JointPath.Clear(); result.AutomaticWaypoints.Clear();
            result.FailureReason = $"Required rise-traverse-descend route failed after {tested} edge checks. " +
                (lastFailure ?? "TCP corridor is blocked or the search budget was exhausted.") +
                " No triangular fallback is allowed in this mode.";
            return false;
        }

        // Height seeding only: this does not remove any obstacle from robot/link
        // collision validation. Test the actual projected segment, not its large AABB.
        internal static bool CrossesHorizontalFootprint(Vector3 a, Vector3 b, Bounds obstacle, float padding)
        {
            float enter = 0f, exit = 1f;
            for (int dimension = 0; dimension <= 2; dimension += 2)
            {
                float low = obstacle.min[dimension] - Math.Max(0f, padding);
                float high = obstacle.max[dimension] + Math.Max(0f, padding);
                float origin = a[dimension], delta = b[dimension] - origin;
                if (Math.Abs(delta) < 1e-8f)
                { if (origin < low || origin > high) return false; continue; }
                float first = (low - origin) / delta, last = (high - origin) / delta;
                if (first > last) { float swap = first; first = last; last = swap; }
                enter = Math.Max(enter, first); exit = Math.Min(exit, last);
                if (enter > exit) return false;
            }
            return true;
        }

        private float DetourRadius()
        {
            float radius = 0.15f;
            if (world.LinearRoutingObstacles != null)
                foreach (Bounds obstacle in world.LinearRoutingObstacles)
                    radius = Mathf.Max(radius, obstacle.size.magnitude * 0.5f + world.Clearance + 0.05f);
            return radius;
        }

        private bool TryDiagonalRoutes(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result, Func<double> budgetElapsed,
            ref int tested, ref string lastFailure, bool staggeredOnly)
        {
            RouteBasis(a, b, out Vector3 forward, out Vector3 side, out Vector3 lift);
            int limit = Math.Min(world.LinearWaypointMaximumEdges, staggeredOnly ? 52 : 18);
            foreach (float scale in new[] { 1f, 2f, 3f })
                foreach (Vector3 direction in new[] { lift, side, -side, -lift, (lift + side) / 1.414214f, (lift - side) / 1.414214f })
                {
                    float radius = DetourRadius() * scale;
                    if (!staggeredOnly)
                    {
                        var apex = new List<Vector3> { Vector3.Lerp(a, b, 0.5f) + direction * radius, b };
                        if (TryCandidateRoute(start, goal, a, ar, b, br, apex, result, budgetElapsed, limit, ref tested, ref lastFailure)) return true;
                        if (tested >= limit || budgetElapsed() >= world.LinearWaypointSearchSeconds) return false;
                        continue;
                    }
                    // Staggered offsets create a diagonal dogleg or zigzag. Directions
                    // come from the start-goal frame, not the world's X/Y/Z grid.
                    Vector3 turn = Vector3.Cross(forward, direction);
                    var staggered = new List<Vector3> {
                        Vector3.Lerp(a, b, 0.3f) + direction * radius + turn * (radius * 0.45f),
                        Vector3.Lerp(a, b, 0.7f) + direction * radius - turn * (radius * 0.45f), b };
                    if (TryCandidateRoute(start, goal, a, ar, b, br, staggered, result, budgetElapsed, limit, ref tested, ref lastFailure)) return true;
                    if (tested >= limit || budgetElapsed() >= world.LinearWaypointSearchSeconds) return false;
                }
            return false;
        }

        private bool TryCandidateRoute(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, List<Vector3> targets, Result result, Func<double> budgetElapsed,
            int edgeLimit, ref int tested, ref string lastFailure)
        {
            float[] from = start;
            Vector3 fromPosition = a;
            Quaternion fromRotation = ar;
            var jointPath = new List<float[]> { (float[])start.Clone() };
            var waypointPositions = new List<Vector3>();
            for (int i = 0; i < targets.Count; i++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                if (tested >= edgeLimit || budgetElapsed() >= world.LinearWaypointSearchSeconds) return false;
                Vector3 position = targets[i];
                bool final = i == targets.Count - 1;
                if (!TcpLineClear(fromPosition, position))
                { lastFailure = $"TCP occupancy blocks candidate segment {i + 1}/{targets.Count}."; return false; }
                tested++;
                Vector3 delta = b - a;
                float t = delta.sqrMagnitude < 1e-10f ? 0f : Mathf.Clamp01(Vector3.Dot(position - a, delta) / delta.sqrMagnitude);
                Quaternion rotation = final ? br : Quaternion.Slerp(ar, br, t);
                world.GetTcpPosition(from);
                float[] endpoint = goal;
                if (!final)
                {
                    var solutions = world.SolveIk(position, rotation);
                    if (solutions == null || solutions.Count == 0)
                    { lastFailure = $"No IK solution at candidate segment {i + 1}/{targets.Count} endpoint."; return false; }
                    endpoint = solutions[0];
                }
                var edge = new Result();
                bool accepted = linear.TryPlanDirect(from, endpoint, fromPosition, fromRotation,
                    position, rotation, edge, poseGoal: !final, refinements: 2,
                    shouldAbort: () => budgetElapsed() >= world.LinearWaypointSearchSeconds);
                result.RoutesTried += edge.RoutesTried;
                result.Iterations += edge.Iterations;
                if (!accepted)
                {
                    lastFailure = edge.FailureReason;
                    if (world.LinearForceLiftTraverseDescend && edge.HasFailurePosition)
                    { result.HasFailurePosition = true; result.FailurePosition = edge.FailurePosition; }
                    return false;
                }
                for (int j = 1; j < edge.JointPath.Count; j++) jointPath.Add(edge.JointPath[j]);
                from = edge.JointPath[edge.JointPath.Count - 1];
                fromPosition = position; fromRotation = rotation;
                if (!final) waypointPositions.Add(position);
            }
            result.JointPath.AddRange(jointPath);
            result.AutomaticWaypoints.AddRange(waypointPositions);
            result.FailureReason = null; result.HasFailurePosition = false;
            return true;
        }

        private bool TrySpatialCorridors(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result, Func<double> elapsed, ref int tested, ref string lastFailure)
        {
            if (world.LinearRoutingObstacles == null || world.LinearRoutingObstacles.Length == 0) return false;
            var boxes = new List<Bounds>(world.LinearRoutingObstacles);
            Bounds combined = boxes[0];
            foreach (Bounds box in boxes) combined.Encapsulate(box);
            boxes.Add(combined);
            var routes = new List<List<Vector3>>();
            void AddLevel(int dimension, float level)
            {
                Vector3 AtLevel(Vector3 p) => dimension == 0 ? new Vector3(level,p.y,p.z) :
                    dimension == 1 ? new Vector3(p.x,level,p.z) : new Vector3(p.x,p.y,level);
                var candidate = new List<Vector3> { AtLevel(a), AtLevel(b), b };
                float length = Vector3.Distance(a,candidate[0]) + Vector3.Distance(candidate[0],candidate[1]) + Vector3.Distance(candidate[1],b);
                // Splitting the already-rejected direct line into collinear pieces
                // is not a detour and must not consume the spatial-search budget.
                if (length <= Vector3.Distance(a,b) + 0.0005f) return;
                foreach (var existing in routes)
                    if ((existing[0]-candidate[0]).sqrMagnitude < 1e-8f && (existing[1]-candidate[1]).sqrMagnitude < 1e-8f) return;
                routes.Add(candidate);
            }
            foreach (Bounds box in boxes)
                foreach (float extra in new[] { 0.02f, 0.08f, 0.2f })
                    for (int dimension = 0; dimension < 3; dimension++)
                        foreach (bool positive in new[] { false, true })
                        {
                            float level = positive ? box.max[dimension] + world.Clearance + extra : box.min[dimension] - world.Clearance - extra;
                            AddLevel(dimension, level);
                        }
            // Surface clearance does not account for the wrist/tool's extent.
            // Seed interior gap positions as well as obstacle-face offsets.
            for (int i = 0; i < world.LinearRoutingObstacles.Length; i++)
                for (int j = 0; j < world.LinearRoutingObstacles.Length; j++)
                    if (i != j)
                        for (int dimension = 0; dimension < 3; dimension++)
                        {
                            float low = boxes[i].max[dimension] + world.Clearance;
                            float high = boxes[j].min[dimension] - world.Clearance;
                            if (high <= low) continue;
                            foreach (float fraction in new[] { 0.25f, 0.5f, 0.75f })
                                AddLevel(dimension, Mathf.Lerp(low, high, fraction));
                        }
            float Length(List<Vector3> route) => Vector3.Distance(a,route[0]) + Vector3.Distance(route[0],route[1]) + Vector3.Distance(route[1],b);
            // A lower obstacle's top and an upper obstacle's underside also seed
            // corridors through gaps. Individual bounds avoid enclosing free gaps
            // inside a single combined obstacle. All six faces compete by length.
            routes.Sort((x,y)=>Length(x).CompareTo(Length(y)));
            int limit = Math.Min(world.LinearWaypointMaximumEdges, 24);
            int attempts = 0;
            foreach (var route in routes)
            {
                if (tested >= limit || elapsed() >= world.LinearWaypointSearchSeconds) break;
                attempts++;
                lastFailure = null;
                bool accepted = TryCandidateRoute(start, goal, a, ar, b, br, route, result, elapsed, limit, ref tested, ref lastFailure);
                Vector3 p = route[0];
                string description = $"Corridor via ({p.x:F3}, {p.y:F3}, {p.z:F3}) m, length {Length(route):F3} m: ";
                if (accepted)
                {
                    result.RoutingDiagnostics += description + $"accepted after {attempts} candidates / {tested} edge checks.\n";
                    return true;
                }
                if (attempts <= 12)
                {
                    string reason = lastFailure ?? "TCP occupancy, unavailable IK, or corridor budget rejected this candidate.";
                    if (reason.Length > 450) reason = reason.Substring(0,450) + "...";
                    result.RoutingDiagnostics += description + reason + "\n";
                }
            }
            result.RoutingDiagnostics += $"Spatial corridor phase ended after {attempts} candidates / {tested} edge checks; continuing fallback search.\n";
            return false;
        }

        // Keep orthogonal corridors as fallback candidates, not as a path constraint.
        // These deliberately allow a short retreat and a vertical lift as separate
        // segments; a diagonal edge can hit the wrist obstacle during the same move.
        private bool TryEscapeCorridors(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result, Func<double> budgetElapsed,
            ref int tested, ref string lastFailure)
        {
            float top = Mathf.Max(a.y, b.y);
            float left = Mathf.Min(a.x,b.x), right = Mathf.Max(a.x,b.x), back = Mathf.Min(a.z,b.z), front = Mathf.Max(a.z,b.z);
            if (world.LinearRoutingObstacles != null)
                foreach (Bounds obstacle in world.LinearRoutingObstacles)
                {
                    top = Mathf.Max(top, obstacle.max.y);
                    left = Mathf.Min(left,obstacle.min.x); right = Mathf.Max(right,obstacle.max.x);
                    back = Mathf.Min(back,obstacle.min.z); front = Mathf.Max(front,obstacle.max.z);
                }
            int corridorBudget = Math.Min(world.LinearWaypointMaximumEdges, 40);
            foreach (float elevation in new[] { 0.08f, 0.25f, 0.45f })
            {
                float height = top + world.Clearance + elevation;
                var retreats = new List<Vector3> { Vector3.zero };
                foreach (float distance in new[] { 0.08f, 0.2f, 0.45f })
                    foreach (Vector3 direction in new[] { Vector3.left, Vector3.right, Vector3.forward, Vector3.back })
                        retreats.Add(direction * distance);
                // Try a small retreat at all heights before the larger retreat shells.
                for (int face = 0; face < 5; face++)
                {
                    int retreatCount = face == 0 ? (elevation == 0.08f ? 5 : retreats.Count) : 1;
                    for (int r = 0; r < retreatCount; r++)
                    {
                        if (tested >= corridorBudget || budgetElapsed() >= world.LinearWaypointSearchSeconds) return false;
                        Vector3 retreat = a + retreats[r];
                        var targets = new List<Vector3>();
                        if (retreats[r].sqrMagnitude > 1e-8f) targets.Add(retreat);
                        float margin = world.Clearance + elevation;
                        Vector3 entry, exit;
                        if (face == 0) { entry = new Vector3(retreat.x,height,retreat.z); exit = new Vector3(b.x,height,b.z); }
                        else if (face == 1) { entry = new Vector3(left-margin,retreat.y,retreat.z); exit = new Vector3(left-margin,b.y,b.z); }
                        else if (face == 2) { entry = new Vector3(right+margin,retreat.y,retreat.z); exit = new Vector3(right+margin,b.y,b.z); }
                        else if (face == 3) { entry = new Vector3(retreat.x,retreat.y,back-margin); exit = new Vector3(b.x,b.y,back-margin); }
                        else { entry = new Vector3(retreat.x,retreat.y,front+margin); exit = new Vector3(b.x,b.y,front+margin); }
                        targets.Add(entry);
                        targets.Add(exit);
                        targets.Add(b);
                        if (TryCandidateRoute(start, goal, a, ar, b, br, targets, result, budgetElapsed,
                            corridorBudget, ref tested, ref lastFailure)) return true;
                    }
                }
            }
            return false;
        }
        private static bool HasAncestor(State state, int point)
        {
            for (; state != null; state = state.Parent) if (state.Point == point) return true;
            return false;
        }

        private static string Key(int point, float[] angles)
        {
            string key = point.ToString();
            // Actual unwrapped joint positions retain distinct IK branches and turns.
            foreach (float angle in angles) key += ":" + Mathf.RoundToInt(angle / 2f);
            return key;
        }

        private bool TcpLineClear(Vector3 a, Vector3 b)
        {
            if (world.IsCartesianPointBlocked == null) return true;
            int count = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(a, b) / 0.02f));
            for (int i = 1; i <= count; i++)
                if (world.IsCartesianPointBlocked(Vector3.Lerp(a, b, i / (float)count), world.Clearance)) return false;
            return true;
        }

        private List<Vector3> CreateCandidates(Vector3 a, Vector3 b, Vector3 failure)
        {
            var points = new List<Vector3> { a, b };
            void Add(Vector3 p)
            {
                if (points.Count >= 192 || (world.IsCartesianPointBlocked != null && world.IsCartesianPointBlocked(p, world.Clearance))) return;
                foreach (Vector3 existing in points) if ((existing - p).sqrMagnitude < 0.0001f) return;
                points.Add(p);
            }
            // Near a wrist-clearance boundary a feasible detour can initially move
            // away from the goal. Include axis and diagonal probes around the actual
            // start, rather than only around the later failed TCP location.
            foreach (float radius in new[] { 0.08f, 0.2f, 0.45f })
            {
                foreach (Vector3 direction in new[] { Vector3.up, Vector3.left, Vector3.right, Vector3.forward, Vector3.back, Vector3.down })
                    Add(a + direction * radius);
                if (radius != 0.2f) continue;
                for (int x = -1; x <= 1; x++)
                for (int y = -1; y <= 1; y++)
                for (int z = -1; z <= 1; z++)
                {
                    var direction = new Vector3(x, y, z);
                    if (direction.sqrMagnitude <= 1f) continue;
                    Add(a + direction * (radius / direction.magnitude));
                }
            }
            initialEscapeEnd = points.Count;
            // Free-angle candidates along the route allow multi-segment diagonal and
            // zigzag connections in the roadmap as well as the quick route probes.
            RouteBasis(a, b, out _, out Vector3 side, out Vector3 lift);
            foreach (float fraction in new[] { 0.25f, 0.5f, 0.75f })
                foreach (float scale in new[] { 1f, 2f })
                    foreach (Vector3 direction in new[] { lift, -lift, side, -side, (lift + side) / 1.414214f, (lift - side) / 1.414214f })
                        Add(Vector3.Lerp(a, b, fraction) + direction * (DetourRadius() * scale));
            void Around(Bounds obstacle)
            {
                foreach (float extra in new[] { 0.08f, 0.25f, 0.45f })
                {
                    Bounds expanded = obstacle;
                    expanded.Expand(2f * (world.Clearance + extra));
                    Vector3 low = expanded.min, high = expanded.max;
                    // Offset lifting columns let the arm retreat before rising. A
                    // diagonal from the original start can cross a wrist boundary even
                    // when both the retreat and the subsequent vertical segment are safe.
                    foreach (Vector3 direction in new[] { Vector3.left, Vector3.right, Vector3.forward, Vector3.back })
                    {
                        Vector3 retreat = a + direction * extra;
                        Add(retreat);
                        Add(new Vector3(retreat.x, high.y, retreat.z));
                    }
                    // Two overhead nodes produce the useful rise / traverse / descend route.
                    Add(new Vector3(a.x, high.y, a.z)); Add(new Vector3(b.x, high.y, b.z));
                    Add(new Vector3(low.x, a.y, a.z)); Add(new Vector3(low.x, b.y, b.z));
                    Add(new Vector3(high.x, a.y, a.z)); Add(new Vector3(high.x, b.y, b.z));
                    Add(new Vector3(a.x, a.y, low.z)); Add(new Vector3(b.x, b.y, low.z));
                    Add(new Vector3(a.x, a.y, high.z)); Add(new Vector3(b.x, b.y, high.z));
                    for (int x = 0; x < 2; x++)
                    for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++) Add(new Vector3(x == 0 ? low.x : high.x, y == 0 ? low.y : high.y, z == 0 ? low.z : high.z));
                }
            }
            if (world.LinearRoutingObstacles != null && world.LinearRoutingObstacles.Length > 0)
            {
                Bounds combined = world.LinearRoutingObstacles[0];
                foreach (Bounds obstacle in world.LinearRoutingObstacles) combined.Encapsulate(obstacle);
                Around(combined);
                foreach (Bounds obstacle in world.LinearRoutingObstacles) Around(obstacle);
            }
            // Also try detours near the failing position for link collisions and singularities
            // that a TCP-only obstacle map cannot represent.
            foreach (float distance in new[] { 0.1f, 0.25f, 0.5f })
                foreach (Vector3 direction in new[] { Vector3.up, Vector3.down, Vector3.left, Vector3.right, Vector3.forward, Vector3.back })
                    Add(failure + direction * distance);
            return points;
        }
    }

    /// <summary>Worker computation plus scene-query execution, excluding queue waits.
    /// The owning worker pauses before dispatch and resumes after the answer arrives.</summary>
    internal sealed class PlanningComputationClock
    {
        private readonly System.Diagnostics.Stopwatch computation = System.Diagnostics.Stopwatch.StartNew();
        private long sceneTicks;
        public double ElapsedSeconds => computation.Elapsed.TotalSeconds +
            sceneTicks / (double)System.Diagnostics.Stopwatch.Frequency;
        public void SuspendForQuery() => computation.Stop();
        public void ResumeAfterQuery(long executionTicks)
        {
            sceneTicks += executionTicks;
            computation.Start();
        }
    }
}











