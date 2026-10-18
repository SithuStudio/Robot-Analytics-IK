using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;
using GridSearchSnapshot = RobotIK.RobotBaseSixAxisPlannerAlgorithm.GridSearchSnapshot;
using GridSearchResult = RobotIK.RobotBaseSixAxisPlannerAlgorithm.GridSearchResult;

namespace RobotIK
{
    /// <summary>Cartesian grid A* and background occupancy snapshot search. SearchSnapshot uses copied value data only.</summary>
    public sealed class GridAStarPlanner : RobotBaseSixAxisPlanner
    {
        public GridAStarPlanner(World world) : base(world) { }

        protected override bool Plan(float[] start, float[] goal, Vector3 a, Quaternion ar,
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

        public static GridSearchResult SearchSnapshot(GridSearchSnapshot snapshot)
        {
            if (snapshot == null) throw new ArgumentNullException(nameof(snapshot));
            if (snapshot.CellSize <= 0f) throw new ArgumentOutOfRangeException(nameof(snapshot.CellSize));
            var result = new GridSearchResult();
            float cell = snapshot.CellSize;
            Vector3 origin = snapshot.Workspace.min;
            int maxX = Math.Max(0, Ceil(snapshot.Workspace.size.x / cell) - 1);
            int maxY = Math.Max(0, Ceil(snapshot.Workspace.size.y / cell) - 1);
            int maxZ = Math.Max(0, Ceil(snapshot.Workspace.size.z / cell) - 1);
            int dimY = maxY + 1, dimZ = maxZ + 1;
            long totalCells = (long)(maxX + 1) * dimY * dimZ;
            if (totalCells > 16000000L)
            { result.FailureReason = $"Grid occupancy would contain {totalCells:N0} cells. Increase Grid Cell Size Millimeters."; return result; }
            Vector3Int ToCell(Vector3 p) => new Vector3Int(
                Clamp((int)Math.Floor((p.x - origin.x) / cell), 0, maxX),
                Clamp((int)Math.Floor((p.y - origin.y) / cell), 0, maxY),
                Clamp((int)Math.Floor((p.z - origin.z) / cell), 0, maxZ));
            Vector3 Center(Vector3Int c) => origin + new Vector3((c.x + 0.5f) * cell, (c.y + 0.5f) * cell, (c.z + 0.5f) * cell);
            bool Inside(Vector3 p) => snapshot.Workspace.Contains(p);
            if (!Inside(snapshot.StartPosition) || !Inside(snapshot.GoalPosition))
            { result.FailureReason = "Start or End Target lies outside the configured cubic grid workspace. Move Grid Workspace Center or increase Grid Workspace Size Meters."; return result; }
            var occupancy = new byte[(int)totalCells];
            int Index(int x, int y, int z) => (x * dimY + y) * dimZ + z;
            Bounds[] obstacleBounds = snapshot.InflatedObstacles ?? Array.Empty<Bounds>();
            for (int i = 0; i < obstacleBounds.Length; i++)
            {
                Bounds b = obstacleBounds[i];
                if (!snapshot.Workspace.Intersects(b)) continue;
                Vector3Int low = ToCell(b.min), high = ToCell(b.max);
                for (int x = low.x; x <= high.x; x++)
                for (int y = low.y; y <= high.y; y++)
                for (int z = low.z; z <= high.z; z++)
                    if (b.Contains(Center(new Vector3Int(x, y, z)))) occupancy[Index(x, y, z)] = 1;
            }
            bool Blocked(Vector3Int c)
            {
                if (snapshot.ForbiddenCells != null)
                    for (int f = 0; f < snapshot.ForbiddenCells.Length; f++)
                        if (snapshot.ForbiddenCells[f].Equals(c)) return true;
                return occupancy[Index(c.x, c.y, c.z)] != 0;
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

        private readonly struct GridEntry : IComparable<GridEntry>
        {
            public readonly Vector3Int Cell; private readonly float score; private readonly int order;
            public GridEntry(Vector3Int cell, float score, int order) { Cell = cell; this.score = score; this.order = order; }
            public int CompareTo(GridEntry other) { int c = score.CompareTo(other.score); return c != 0 ? c : order.CompareTo(other.order); }
        }
    }
}
