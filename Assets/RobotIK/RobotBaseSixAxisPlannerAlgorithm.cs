using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Non-MonoBehaviour implementation of the six-axis path search algorithms.
    /// A planning world supplies IK, collision, and FK queries; the search and path
    /// construction themselves live here, so this class can be created by any caller.
    /// </summary>
    public sealed class RobotBaseSixAxisPlannerAlgorithm
    {
        public enum Kind { RrtConnectFast, RrtConnectOptimized, CartesianArc, GridAStar }

        /// <summary>Scene/query services required by planning, independent of Unity lifecycle.</summary>
        public sealed class World
        {
            public Func<float[], float[], bool> IsJointSegmentFree;
            public Func<Vector3, Quaternion, List<float[]>> SolveIk;
            public Func<float[], Vector3> GetTcpPosition;
            public Func<float[], Quaternion> GetTcpRotation;
            public Func<float[], bool> IsConfigurationInCollision;
            public Func<float[]> RandomConfiguration;
            public Func<float[], float[], float> MotionCost;
            public Func<float[], float[], float> RouteCost;
            public Func<float[], float[], float> TcpTravelCost;
            public Func<IReadOnlyList<float[]>, float> TcpPathCost;
            public Action<int> IterationProgress;
            public bool PrioritizeShortestPath;
            public CancellationToken CancellationToken;
            public Func<Vector3, float, bool> IsCartesianPointBlocked;
            public Vector2[] JointLimits;
            public Bounds Workspace;
            public Vector3[] ObstacleBounds;
            public Vector3 GridOrigin;
            public float GridCellSize;
            public float Clearance;
            public float ExtensionStepDegrees = 20f;
            public float CollisionResolutionDegrees = 4f;
            public int MaximumIterations = 4500;
            public int MaximumGridNodes = 20000;
            public int MaximumGridRouteAttempts = 8;
            public float GoalBias = 0.08f;
            public float ArcStepMeters = 0.04f;
            public System.Random Random = new System.Random(1);
        }

        public sealed class Result
        {
            public readonly List<float[]> JointPath = new List<float[]>();
            public int Iterations;
            public int RoutesTried;
            public Vector3 FailurePosition;
            public string FailureReason;
        }

        /// <summary>Immutable input snapshot for background-only occupancy search.</summary>
        public sealed class GridSearchSnapshot
        {
            public Bounds Workspace;
            public Bounds[] InflatedObstacles;
            public Vector3 StartPosition;
            public Vector3 GoalPosition;
            public float CellSize;
            public int MaximumNodes;
            public Vector3Int[] ForbiddenCells;
            public CancellationToken CancellationToken;
        }

        public sealed class GridSearchResult
        {
            public readonly List<Vector3> Points = new List<Vector3>();
            public int ExpandedNodes;
            public Vector3 FailurePosition;
            public string FailureReason;
            public bool Found => Points.Count > 1;
        }

        /// <summary>
        /// Searches an immutable occupancy snapshot. This method reads only copied value
        /// data and can run on a worker thread; it must never receive Collider/Transform objects.
        /// </summary>
        public static GridSearchResult SearchGridSnapshot(GridSearchSnapshot snapshot)
        {
            if (snapshot == null) throw new ArgumentNullException(nameof(snapshot));
            if (snapshot.CellSize <= 0f) throw new ArgumentOutOfRangeException(nameof(snapshot.CellSize));
            var result = new GridSearchResult();
            float cell = snapshot.CellSize;
            Vector3 origin = snapshot.Workspace.min;
            Vector3Int ToCell(Vector3 p) => new Vector3Int(
                Clamp(Round((p.x - origin.x) / cell), 0, Ceil(snapshot.Workspace.size.x / cell)),
                Clamp(Round((p.y - origin.y) / cell), 0, Ceil(snapshot.Workspace.size.y / cell)),
                Clamp(Round((p.z - origin.z) / cell), 0, Ceil(snapshot.Workspace.size.z / cell)));
            Vector3 Center(Vector3Int c) => origin + new Vector3(c.x * cell, c.y * cell, c.z * cell);
            bool Blocked(Vector3Int c)
            {
                if (snapshot.ForbiddenCells != null)
                    for (int f = 0; f < snapshot.ForbiddenCells.Length; f++)
                        if (snapshot.ForbiddenCells[f].Equals(c)) return true;
                Vector3 p = Center(c);
                Bounds[] obstacles = snapshot.InflatedObstacles;
                for (int i = 0; i < obstacles.Length; i++) if (obstacles[i].Contains(p)) return true;
                return false;
            }

            Vector3Int start = ToCell(snapshot.StartPosition), goal = ToCell(snapshot.GoalPosition);
            if (start == goal)
            {
                result.Points.Add(snapshot.StartPosition); result.Points.Add(snapshot.GoalPosition); return result;
            }
            var open = new SortedSet<GridSearchEntry>();
            var parents = new Dictionary<Vector3Int, Vector3Int>();
            var costs = new Dictionary<Vector3Int, float> { [start] = 0f };
            int order = 0;
            open.Add(new GridSearchEntry(start, Distance(Center(start), Center(goal)), order++));
            int maxX = Ceil(snapshot.Workspace.size.x / cell);
            int maxY = Ceil(snapshot.Workspace.size.y / cell);
            int maxZ = Ceil(snapshot.Workspace.size.z / cell);
            Vector3Int frontier = start;
            float frontierDistance = Distance(Center(start), Center(goal));
            bool found = false;
            while (open.Count > 0 && result.ExpandedNodes < snapshot.MaximumNodes)
            {
                if ((result.ExpandedNodes & 127) == 0 && snapshot.CancellationToken.IsCancellationRequested)
                { result.FailureReason = "Grid search cancelled."; return result; }
                GridSearchEntry entry = open.Min; open.Remove(entry);
                Vector3Int current = entry.Cell;
                float currentCost = costs[current];
                float h = Distance(Center(current), Center(goal));
                if (Math.Abs(entry.Score - currentCost - h) > 0.0001f) continue;
                result.ExpandedNodes++;
                if (current.Equals(goal)) { found = true; break; }
                if (h < frontierDistance) { frontierDistance = h; frontier = current; }
                for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    var next = new Vector3Int(current.x + dx, current.y + dy, current.z + dz);
                    if (next.x < 0 || next.y < 0 || next.z < 0 || next.x > maxX || next.y > maxY || next.z > maxZ) continue;
                    if (!next.Equals(goal) && !next.Equals(start) && Blocked(next)) continue;
                    // Reject diagonal moves that cut through an occupied adjacent cell.
                    int axisCount = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
                    bool cornerBlocked = false;
                    for (int mask = 1; mask < (1 << axisCount) && !cornerBlocked; mask++)
                    {
                        int bit = 0, sx = current.x, sy = current.y, sz = current.z;
                        if (dx != 0) { if ((mask & (1 << bit)) != 0) sx += dx; bit++; }
                        if (dy != 0) { if ((mask & (1 << bit)) != 0) sy += dy; bit++; }
                        if (dz != 0 && (mask & (1 << bit)) != 0) sz += dz;
                        var side = new Vector3Int(sx, sy, sz);
                        if (!side.Equals(start) && !side.Equals(goal) && Blocked(side)) cornerBlocked = true;
                    }
                    if (cornerBlocked) continue;
                    float nextCost = currentCost + Distance(Center(current), Center(next));
                    if (costs.TryGetValue(next, out float oldCost) && oldCost <= nextCost) continue;
                    costs[next] = nextCost; parents[next] = current;
                    open.Add(new GridSearchEntry(next, nextCost + Distance(Center(next), Center(goal)), order++));
                }
            }
            if (!found)
            {
                result.FailurePosition = Center(frontier);
                result.FailureReason = result.ExpandedNodes >= snapshot.MaximumNodes
                    ? $"A* reached the {snapshot.MaximumNodes} node limit before reaching the goal."
                    : "A* could not connect the start and goal cells.";
                return result;
            }
            var cells = new List<Vector3Int> { goal };
            Vector3Int cursor = goal;
            while (!cursor.Equals(start)) { cursor = parents[cursor]; cells.Add(cursor); }
            cells.Reverse();
            var simplified = new List<Vector3> { snapshot.StartPosition };
            int anchor = 0;
            while (anchor < cells.Count - 1)
            {
                int furthest = anchor + 1;
                for (int candidate = cells.Count - 1; candidate > anchor + 1; candidate--)
                {
                    Vector3 from = Center(cells[anchor]), to = Center(cells[candidate]);
                    int samples = Math.Max(1, Ceil(Distance(from, to) / (cell * 0.5f)));
                    bool clear = true;
                    for (int sample = 1; sample < samples; sample++)
                        if (Blocked(ToCell(Lerp(from, to, sample / (float)samples)))) { clear = false; break; }
                    if (clear) { furthest = candidate; break; }
                }
                if (furthest < cells.Count - 1) simplified.Add(Center(cells[furthest]));
                anchor = furthest;
            }
            simplified.Add(snapshot.GoalPosition);
            result.Points.AddRange(simplified);
            return result;
        }

        // Kept separate from Unity's scene APIs so the worker uses only managed math.
        private static int Round(float value) => (int)Math.Round(value, MidpointRounding.ToEven);
        private static int Ceil(float value) => (int)Math.Ceiling(value);
        private static int Clamp(int value, int min, int max) => value < min ? min : value > max ? max : value;
        private static float Distance(Vector3 a, Vector3 b)
        { float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z; return (float)Math.Sqrt(x * x + y * y + z * z); }
        private static Vector3 Lerp(Vector3 a, Vector3 b, float t) => new Vector3(
            a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);

        private readonly struct GridSearchEntry : IComparable<GridSearchEntry>
        {
            public readonly Vector3Int Cell;
            public readonly float Score;
            private readonly int order;
            public GridSearchEntry(Vector3Int cell, float score, int order) { Cell = cell; Score = score; this.order = order; }
            public int CompareTo(GridSearchEntry other) { int c = Score.CompareTo(other.Score); return c != 0 ? c : order.CompareTo(other.order); }
        }

        private readonly World world;
        public RobotBaseSixAxisPlannerAlgorithm(World world)
        {
            this.world = world ?? throw new ArgumentNullException(nameof(world));
        }

        public bool TryPlan(Kind kind, float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, out Result result)
        {
            ValidateWorld();
            result = new Result();
            switch (kind)
            {
                case Kind.RrtConnectFast:
                case Kind.RrtConnectOptimized:
                    return TryPlanRrt(kind == Kind.RrtConnectFast, start, goal, result);
                case Kind.CartesianArc:
                    return TryPlanCartesianArc(start, goal, startPosition, startRotation,
                        goalPosition, goalRotation, result);
                case Kind.GridAStar:
                    return TryPlanGrid(start, goal, startPosition, startRotation,
                        goalPosition, goalRotation, result);
                default:
                    throw new ArgumentOutOfRangeException(nameof(kind), kind, null);
            }
        }

        private void ValidateWorld()
        {
            if (world.IsJointSegmentFree == null || world.SolveIk == null ||
                world.GetTcpPosition == null || world.GetTcpRotation == null ||
                world.IsConfigurationInCollision == null || world.MotionCost == null)
                throw new InvalidOperationException("Planning world is missing IK, FK, or collision query callbacks.");
        }

        private bool TryPlanRrt(bool firstSolution, float[] start, float[] goal, Result result)
        {
            if (world.IsJointSegmentFree(start, goal))
            {
                result.JointPath.Add((float[])start.Clone());
                result.JointPath.Add((float[])goal.Clone());
                return true;
            }

            var treeA = new List<Node> { new Node((float[])start.Clone(), -1) };
            var treeB = new List<Node> { new Node((float[])goal.Clone(), -1) };
            bool aFromStart = true;
            int firstConnection = -1;
            int budget = Math.Max(1, world.MaximumIterations);
            // Optimized mode spends the remaining search budget comparing route length.
            // Fast mode exits at the first valid connection.
            int optimizationBudget = firstSolution ? 0 : budget;
            List<float[]> best = null;
            float bestCost = float.PositiveInfinity;

            for (int iteration = 0; iteration < budget; iteration++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                if (firstConnection >= 0 && iteration - firstConnection > optimizationBudget) break;
                result.Iterations++;
                world.IterationProgress?.Invoke(result.Iterations);
                float[] sample = world.Random != null && world.Random.NextDouble() < world.GoalBias
                    ? treeB[0].Angles : GetRandomConfiguration();
                int nearA = FindNearest(treeA, sample);
                if (TryExtend(treeA, nearA, sample, out int addedA))
                {
                    float[] target = treeA[addedA].Angles;
                    int nearB = FindNearest(treeB, target);
                    bool connected = false;
                    while (TryExtend(treeB, nearB, target, out int next))
                    {
                        nearB = next;
                        if (JointDistance(treeB[nearB].Angles, target) < 0.01f) { connected = true; break; }
                    }
                    if (connected)
                    {
                        List<float[]> candidate = JoinTrees(treeA, addedA, treeB, nearB, aFromStart);
                        float cost = PathCost(candidate);
                        if (cost < bestCost) { best = candidate; bestCost = cost; }
                        if (firstConnection < 0) firstConnection = iteration;
                    }
                }
                SwapTrees(ref treeA, ref treeB);
                aFromStart = !aFromStart;
                if (best != null && firstSolution) break;
            }
            if (best == null) { result.FailureReason = "RRT-Connect exhausted its iteration budget without a connection."; return false; }
            result.JointPath.AddRange(best);
            return true;
        }

        private bool TryPlanCartesianArc(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            Vector3 delta = b - a;
            Vector3 side = Vector3.Cross(delta.normalized, Vector3.up);
            if (side.sqrMagnitude < 1e-5f) side = Vector3.Cross(delta.normalized, Vector3.right);
            side.Normalize();
            Vector3[] directions = { Vector3.up, -Vector3.up, side, -side, Vector3.right, -Vector3.right,
                Vector3.forward, -Vector3.forward };
            float extent = 0.1f;
            if (world.ObstacleBounds != null)
                for (int i = 0; i < world.ObstacleBounds.Length; i++) extent = Mathf.Max(extent, world.ObstacleBounds[i].magnitude + world.Clearance);
            float[] heights = { 0f, 0.15f, 0.3f, extent * 2f, extent * 3f };
            int samples = Mathf.Clamp(Mathf.CeilToInt(delta.magnitude / Mathf.Max(0.005f, world.ArcStepMeters)), 12, 64);
            List<float[]> best = null;
            float bestCost = float.PositiveInfinity;
            for (int h = 0; h < heights.Length; h++)
            for (int d = 0; d < (h == 0 ? 1 : directions.Length); d++)
            {
                result.RoutesTried++;
                Vector3 control = (a + b) * 0.5f + directions[d] * heights[h];
                var candidate = new List<float[]> { (float[])start.Clone() };
                float[] previous = start;
                bool valid = true;
                for (int i = 1; i <= samples; i++)
                {
                    float t = i / (float)samples;
                    Vector3 p = i == samples ? b : (1f-t)*(1f-t)*a + 2f*(1f-t)*t*control + t*t*b;
                    Quaternion q = Quaternion.Slerp(ar, br, t);
                    List<float[]> solutions = i == samples ? new List<float[]> { (float[])goal.Clone() } : world.SolveIk(p, q);
                    if (solutions == null || solutions.Count == 0) { valid = false; break; }
                    solutions.Sort((x, y) => world.MotionCost(previous, x).CompareTo(world.MotionCost(previous, y)));
                    float[] selected = null;
                    for (int k = 0; k < solutions.Count; k++)
                        if (!world.IsConfigurationInCollision(solutions[k]) && world.IsJointSegmentFree(previous, solutions[k]))
                        { selected = solutions[k]; break; }
                    if (selected == null) { valid = false; break; }
                    candidate.Add((float[])selected.Clone());
                    previous = selected;
                }
                if (!valid) continue;
                float cost = PathCost(candidate);
                if (cost < bestCost) { best = candidate; bestCost = cost; }
            }
            if (best == null) { result.FailureReason = "No collision-free Cartesian TCP arc was found."; return false; }
            result.JointPath.AddRange(best);
            return true;
        }

        private bool TryPlanGrid(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            if (world.IsCartesianPointBlocked == null || world.GridCellSize <= 0f)
            { result.FailureReason = "Grid planning requires a positive cell size and occupancy query."; return false; }
            // Grid nodes represent TCP positions. Each candidate cell must admit IK and
            // a collision-free joint transition from the preceding solution.
            Vector3 cell = Vector3.one * world.GridCellSize;
            Vector3 origin = world.GridOrigin;
            Vector3Int Cell(Vector3 p) => Vector3Int.RoundToInt((p - origin) / world.GridCellSize);
            Vector3 Center(Vector3Int c) => origin + Vector3.Scale((Vector3)c, cell);
            Vector3Int first = Cell(a), last = Cell(b);
            var open = new SortedSet<GridEntry>();
            var previous = new Dictionary<Vector3Int, Vector3Int>();
            var costs = new Dictionary<Vector3Int, float> { [first] = 0f };
            int order = 0;
            open.Add(new GridEntry(first, Vector3.Distance(a, b), order++));
            Vector3Int[] offsets = { Vector3Int.right, Vector3Int.left, Vector3Int.up, Vector3Int.down,
                new Vector3Int(0,0,1), new Vector3Int(0,0,-1) };
            while (open.Count > 0 && result.Iterations < world.MaximumGridNodes)
            {
                GridEntry entry = open.Min; open.Remove(entry); result.Iterations++;
                if (entry.Cell == last) break;
                for (int i = 0; i < offsets.Length; i++)
                {
                    Vector3Int next = entry.Cell + offsets[i];
                    Vector3 p = Center(next);
                    if (!world.Workspace.Contains(p) || world.IsCartesianPointBlocked(p, world.Clearance)) continue;
                    float tentative = costs[entry.Cell] + world.GridCellSize;
                    if (costs.TryGetValue(next, out float old) && tentative >= old) continue;
                    costs[next] = tentative; previous[next] = entry.Cell;
                    open.Add(new GridEntry(next, tentative + Vector3.Distance(p, b), order++));
                }
            }
            if (!costs.ContainsKey(last)) { result.FailureReason = "A* found no route through free Cartesian cells."; return false; }
            var cells = new List<Vector3Int> { last };
            while (cells[cells.Count - 1] != first) cells.Add(previous[cells[cells.Count - 1]]);
            cells.Reverse();
            var route = new List<Vector3> { a };
            for (int i = 1; i < cells.Count - 1; i++) route.Add(Center(cells[i]));
            route.Add(b);
            // Greedily smooth the occupancy route before solving its TCP poses.
            for (int i = 0; i + 2 < route.Count;)
            {
                if (CanSee(route[i], route[i + 2])) route.RemoveAt(i + 1); else i++;
            }
            var jointPath = new List<float[]> { (float[])start.Clone() };
            float[] prevJoints = start;
            for (int i = 1; i < route.Count; i++)
            {
                Quaternion q = Quaternion.Slerp(ar, br, i / (float)(route.Count - 1));
                List<float[]> candidates = i == route.Count - 1 ? new List<float[]> { (float[])goal.Clone() } : world.SolveIk(route[i], q);
                if (candidates == null || candidates.Count == 0) { result.FailurePosition = route[i]; result.FailureReason = "Grid route cell has no IK solution."; return false; }
                candidates.Sort((x, y) => world.MotionCost(prevJoints, x).CompareTo(world.MotionCost(prevJoints, y)));
                float[] chosen = null;
                for (int c = 0; c < candidates.Count; c++)
                    if (!world.IsConfigurationInCollision(candidates[c]) && world.IsJointSegmentFree(prevJoints, candidates[c])) { chosen = candidates[c]; break; }
                if (chosen == null) { result.FailurePosition = route[i]; result.FailureReason = "Grid route failed IK collision validation."; return false; }
                jointPath.Add((float[])chosen.Clone()); prevJoints = chosen;
            }
            result.RoutesTried = 1;
            result.JointPath.AddRange(jointPath);
            return true;

            bool CanSee(Vector3 from, Vector3 to)
            {
                int n = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(from, to) / (world.GridCellSize * 0.5f)));
                for (int k = 1; k < n; k++) if (world.IsCartesianPointBlocked(Vector3.Lerp(from, to, k / (float)n), world.Clearance)) return false;
                return true;
            }
        }

        private float[] GetRandomConfiguration()
        {
            if (world.RandomConfiguration != null) return world.RandomConfiguration();
            var result = new float[world.JointLimits.Length];
            for (int i = 0; i < result.Length; i++) result[i] = Mathf.Lerp(world.JointLimits[i].x, world.JointLimits[i].y, (float)world.Random.NextDouble());
            return result;
        }

        private bool TryExtend(List<Node> tree, int fromIndex, float[] target, out int added)
        {
            float[] from = tree[fromIndex].Angles;
            float distance = SearchDistance(from, target);
            added = fromIndex;
            if (distance < 0.001f) return true;
            float t = Math.Min(1f, world.ExtensionStepDegrees / distance);
            var next = new float[from.Length];
            for (int i = 0; i < next.Length; i++) next[i] = from[i] + (target[i] - from[i]) * t;
            if (!world.IsJointSegmentFree(from, next)) return false;
            added = tree.Count; tree.Add(new Node(next, fromIndex)); return true;
        }

        private int FindNearest(List<Node> tree, float[] target)
        {
            int best = 0; float distance = SearchDistance(tree[0].Angles, target);
            for (int i = 1; i < tree.Count; i++) { float d = SearchDistance(tree[i].Angles, target); if (d < distance) { best = i; distance = d; } }
            return best;
        }

        private float SearchDistance(float[] a, float[] b) => world.PrioritizeShortestPath
            ? JointDistance(a, b) : world.MotionCost(a, b);

        private float PathCost(List<float[]> path)
        {
            if (world.PrioritizeShortestPath && world.TcpPathCost != null)
                return world.TcpPathCost(path);
            float total = 0f;
            for (int i = 1; i < path.Count; i++)
                total += world.PrioritizeShortestPath
                    ? world.TcpTravelCost != null ? world.TcpTravelCost(path[i - 1], path[i]) : JointDistance(path[i - 1], path[i])
                    : world.RouteCost != null ? world.RouteCost(path[i - 1], path[i]) : world.MotionCost(path[i - 1], path[i]);
            return total;
        }

        private static float JointDistance(float[] a, float[] b)
        { double sum = 0.0; for (int i = 0; i < a.Length; i++) { double d = a[i] - b[i]; sum += d * d; } return (float)Math.Sqrt(sum); }

        private static List<float[]> JoinTrees(List<Node> a, int ai, List<Node> b, int bi, bool aStartsAtStart)
        {
            List<float[]> first = Trace(a, ai), second = Trace(b, bi);
            var result = new List<float[]>(first.Count + second.Count - 1);
            if (aStartsAtStart) { first.Reverse(); result.AddRange(first); for (int i = 1; i < second.Count; i++) result.Add(second[i]); }
            else { second.Reverse(); result.AddRange(second); result.AddRange(first); }
            return result;
        }

        private static List<float[]> Trace(List<Node> tree, int index)
        { var result = new List<float[]>(); while (index >= 0) { result.Add(tree[index].Angles); index = tree[index].Parent; } return result; }

        private static void SwapTrees(ref List<Node> a, ref List<Node> b) { List<Node> tmp = a; a = b; b = tmp; }

        private sealed class Node
        { public readonly float[] Angles; public readonly int Parent; public Node(float[] angles, int parent) { Angles = angles; Parent = parent; } }

        private readonly struct GridEntry : IComparable<GridEntry>
        {
            public readonly Vector3Int Cell; private readonly float score; private readonly int order;
            public GridEntry(Vector3Int cell, float score, int order) { Cell = cell; this.score = score; this.order = order; }
            public int CompareTo(GridEntry other) { int c = score.CompareTo(other.score); return c != 0 ? c : order.CompareTo(other.order); }
        }
    }
}
