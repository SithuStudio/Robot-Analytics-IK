using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Strict sampled MoveL: straight TCP translation and shortest quaternion rotation.
    /// All callbacks execute synchronously; use the main thread for Unity physics callbacks.
    /// Optional automatic routing connects validated straight segments. Joint angles are in degrees.</summary>
    public sealed class CartesianLinearPlanner : RobotBaseSixAxisPlanner
    {
        public CartesianLinearPlanner(World world) : base(world) { }
        public double LastSingularityRatio { get; private set; }
        public double SingularityThreshold => world.LinearMinimumSingularValueRatio;
        private string rejection;
        private Vector3 rejectionPosition;
        private bool refinementCanHelp;
        private CartesianOccupancyGrid endpointGrid;
        private System.Diagnostics.Stopwatch poseRetryTimer;
        private double poseRetryBegan;
        internal bool PoseRetryExpired => poseRetryTimer != null &&
            (world.PlanningElapsedSeconds == null ? poseRetryTimer.Elapsed.TotalSeconds : world.PlanningElapsedSeconds() - poseRetryBegan) >= 5.0;
        internal CartesianOccupancyGrid OccupancyGrid => endpointGrid ?? world.LinearOccupancyGrid;

        private bool Reject(string reason, bool canRefine = false) { rejection = reason; refinementCanHelp = canRefine; return false; }

        private sealed class Node
        {
            public float[] Angles;
            public Node Previous;
            public float Cost;
        }

        protected override bool Plan(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            if (!string.IsNullOrEmpty(world.LinearOccupancyError))
            { result.FailureReason = world.LinearOccupancyError; return false; }
            try
            {
                var grid = world.LinearOccupancyGrid;
                if (grid != null)
                {
                    if (!grid.TryCell(a, out int first) || !grid.TryCell(b, out int last))
                    {
                        result.FailureReason = "MoveL endpoints must be inside the scanned volume. Expand the scanner volume and rescan.";
                        return false;
                    }
                    // One opened voxel can remain isolated behind adjacent occupied
                    // voxels. Validate poses before allowing local endpoint access.
                    if (!ValidateEndpoint(ref start, a, ar, "Start", false, result) ||
                        !ValidateEndpoint(ref goal, b, br, "End", world.LinearAllowAlternateGoalConfiguration, result)) return false;
                    var access = grid.WithValidatedEndpointAccess(a, b, out int opened);
                    if (opened > 0)
                    {
                        endpointGrid = access;
                        result.RoutingDiagnostics += $"Start and End IK poses passed joint limits, pose accuracy, robot collision/clearance and singularity checks. Occupancy bypass opens {opened} local endpoint access cells within {grid.EndpointAccessRadius * 1000f:F0} mm of Start/End. All MoveL samples and swept edges remain validated; other occupied cells and scan boundaries stay blocked.\n";
                    }
                }
                if (PlanRoute(start, goal, a, ar, b, br, result)) return true;
                if (!world.LinearIntermediatePositionRetry && !world.LinearIntermediateRotationRetry) return false;
                return TryIntermediatePose(start, goal, a, ar, b, br, result);
            }
            finally { endpointGrid = null; poseRetryTimer = null; }
        }

        // Only the runtime's second pass enables this search, after all exact
        // Start-posture attempts. Temporary poses never modify authored markers.
        private bool TryIntermediatePose(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            string exactFailure = result.FailureReason;
            if (!ValidateEndpoint(ref start, a, ar, "Start", false, result) ||
                !ValidateEndpoint(ref goal, b, br, "End", world.LinearAllowAlternateGoalConfiguration, result)) return false;
            poseRetryTimer = System.Diagnostics.Stopwatch.StartNew();
            poseRetryBegan = world.PlanningElapsedSeconds?.Invoke() ?? 0;
            float fraction = .75f;
            Vector3 axis = b - a;
            if (result.HasFailurePosition && axis.sqrMagnitude > 1e-10f)
                fraction = Math.Max(.15f, Math.Min(.85f, Vector3.Dot(result.FailurePosition - a, axis) / axis.sqrMagnitude));
            var poses = new List<Pose>();
            foreach (float t in new[] { fraction, .5f, .875f })
                foreach (float scale in new[] { .5f, 1f })
                    for (int j = 0; j < 3; j++)
                        foreach (float sign in new[] { 1f, -1f })
                        {
                            Vector3 direction = j == 0 ? Vector3.right : j == 1 ? Vector3.up : Vector3.forward;
                            Vector3 p = Vector3.Lerp(a, b, t);
                            Quaternion r = Quaternion.Slerp(ar, br, t);
                            Vector3 euler = direction * (sign * scale * 5f);
                            Quaternion deviated = Quaternion.Euler(euler.x, euler.y, euler.z) * r;
                            if (world.LinearIntermediatePositionRetry) poses.Add(new Pose(p + direction * (sign * scale * .02f), r));
                            if (world.LinearIntermediateRotationRetry) poses.Add(new Pose(p, deviated));
                            if (world.LinearIntermediatePositionRetry && world.LinearIntermediateRotationRetry)
                                poses.Add(new Pose(p + direction * (sign * scale * .02f), deviated));
                        }
            int tested = 0;
            string lastFailure = exactFailure;
            // Try inexpensive direct pairs first; only then spend the remaining
            // shared budget on grid/geometry routing around obstacles.
            for (int routed = 0; routed < 2 && !PoseRetryExpired; routed++)
                foreach (Pose pose in poses)
                {
                    world.CancellationToken.ThrowIfCancellationRequested();
                    if (PoseRetryExpired) break;
                    if (OccupancyGrid != null && !OccupancyGrid.IsFree(pose.position)) continue;
                    world.GetTcpPosition(start);
                    var solutions = world.SolveIk(pose.position, pose.rotation);
                    if (solutions == null) continue;
                    foreach (var candidate in solutions)
                    {
                        if (PoseRetryExpired) break;
                        tested++;
                        var first = new Result();
                        bool firstOk = routed == 0
                            ? TryPlanDirect(start, candidate, a, ar, pose.position, pose.rotation, first)
                            : PlanRoute(start, candidate, a, ar, pose.position, pose.rotation, first);
                        result.Iterations += first.Iterations; result.RoutesTried += first.RoutesTried;
                        if (!firstOk) { lastFailure = first.FailureReason; continue; }
                        var second = new Result();
                        float[] atPose = first.JointPath[first.JointPath.Count - 1];
                        bool secondOk = routed == 0
                            ? TryPlanDirect(atPose, goal, pose.position, pose.rotation, b, br, second)
                            : PlanRoute(atPose, goal, pose.position, pose.rotation, b, br, second);
                        result.Iterations += second.Iterations; result.RoutesTried += second.RoutesTried;
                        if (!secondOk) { lastFailure = second.FailureReason; continue; }
                        result.JointPath.Clear(); result.AutomaticWaypoints.Clear();
                        result.JointPath.AddRange(first.JointPath);
                        for (int i = 1; i < second.JointPath.Count; i++) result.JointPath.Add(second.JointPath[i]);
                        result.AutomaticWaypoints.AddRange(first.AutomaticWaypoints);
                        result.AutomaticWaypoints.Add(pose.position);
                        result.AutomaticWaypoints.AddRange(second.AutomaticWaypoints);
                        result.HasFailurePosition = false; result.FailureReason = null;
                        result.RoutingDiagnostics += $"Automatic intermediate pose succeeded after {tested} IK trials (position <=20 mm, rotation <=5 degrees as enabled). Start/End stay exact; all samples retain the 5-degree joint-step, singularity and collision/clearance checks.\n";
                        return true;
                    }
                }
            result.JointPath.Clear(); result.AutomaticWaypoints.Clear();
            result.FailureReason = $"{exactFailure} Intermediate pose retry failed after {tested} IK trials{(PoseRetryExpired ? " (5 s computation budget reached)" : string.Empty)}. Last rejection: {lastFailure}. Start/End remain exact; no partial path will execute.";
            return false;
        }

        private bool ValidateEndpoint(ref float[] angles, Vector3 position, Quaternion rotation,
            string label, bool alternate, Result result)
        {
            world.CancellationToken.ThrowIfCancellationRequested();
            if (Valid(angles, position, rotation)) return true;
            string reason = rejection;
            Vector3 failurePosition = rejectionPosition;
            if (alternate && world.SolveIk != null)
            {
                var candidates = world.SolveIk(position, rotation);
                if (candidates != null)
                    foreach (var candidate in candidates)
                    {
                        world.CancellationToken.ThrowIfCancellationRequested();
                        if (Valid(candidate, position, rotation)) { angles = candidate; return true; }
                        reason = rejection; failurePosition = rejectionPosition;
                    }
            }
            result.FailurePosition = failurePosition;
            result.HasFailurePosition = true;
            result.FailureReason = "MoveL endpoint rejected: " + label + ": " + reason;
            return false;
        }

        private bool PlanRoute(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            if (world.LinearAutomaticWaypoints && world.LinearForceLiftTraverseDescend)
            {
                if (Vector3.Distance(a, b) < 0.00001f && Quaternion.Angle(ar, br) < 0.01f)
                    return TryPlanDirect(start, goal, a, ar, b, br, result);
                return new CartesianLinearWaypointSearch(world, this).TryPlan(start, goal, a, ar, b, br, result);
            }
            if (TryPlanDirect(start, goal, a, ar, b, br, result)) return true;
            if (!world.LinearAutomaticWaypoints) return false;
            if (OccupancyGrid != null)
            {
                if (result.FailureReason != null && result.FailureReason.StartsWith("MoveL endpoint rejected:")) return false;
                if (!new CartesianGridWaypointSearch(world, this).TryPlan(start, goal, a, ar, b, br, result))
                {
                    if (endpointGrid != null)
                        result.FailureReason += $" Endpoint IK poses passed validation; local scan access within {endpointGrid.EndpointAccessRadius * 1000f:F0} mm was allowed. A collision-free connecting route is still required.";
                    return false;
                }
                if (world.LinearOptimizePath && poseRetryTimer == null)
                    new CartesianLinearPathOptimizer(world, this).Optimize(start, goal, a, ar, b, br, result);
                return true;
            }
            if (!new CartesianLinearWaypointSearch(world, this).TryPlan(start, goal, a, ar, b, br, result)) return false;
            if (world.LinearOptimizePath && poseRetryTimer == null)
                new CartesianLinearPathOptimizer(world, this).Optimize(start, goal, a, ar, b, br, result);
            return true;
        }

        // Routing uses the same strict segment validator, with no recursive detours.
        internal bool TryPlanDirect(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result, bool poseGoal = false, int refinements = 4, Func<bool> shouldAbort = null)
        {
            if (PoseRetryExpired) { result.FailureReason = "Intermediate pose search budget reached."; return false; }
            var callerAbort = shouldAbort;
            if (poseRetryTimer != null) shouldAbort = () => PoseRetryExpired || (callerAbort != null && callerAbort());
            if (OccupancyGrid != null && !OccupancyGrid.SegmentFree(a,b))
            { result.RoutesTried=1; result.FailureReason="MoveL line crosses an occupied or unscanned grid cell."; return false; }
            if (shouldAbort != null && shouldAbort()) { result.FailureReason = "MoveL edge validation stopped at the automatic search deadline."; return false; }
            bool alternateGoal = poseGoal || world.LinearAllowAlternateGoalConfiguration;
            result.RoutesTried = 1;
            if (!Valid(start, a, ar) || (!alternateGoal && !Valid(goal, b, br)))
            {
                result.FailurePosition = rejectionPosition;
                result.HasFailurePosition = true;
                result.FailureReason = "MoveL endpoint rejected: " + rejection;
                return false;
            }
            // A pose target already satisfied by the current configuration does not
            // require a reconfiguration to a different analytical endpoint branch.
            if (alternateGoal && PoseMatches(start, b, br) &&
                EdgeValid(start, start, a, ar, b, br))
            {
                result.JointPath.Add((float[])start.Clone());
                result.JointPath.Add((float[])start.Clone());
                return true;
            }
            // Reset-to-start commonly leaves only floating-point FK/IK closure error.
            // Preserve that configuration instead of asking analytical IK to reselect a
            // wrist branch along an effectively stationary Cartesian segment.
            float endpointJointTravel = 0f;
            for (int j = 0; j < start.Length; j++)
                endpointJointTravel = Mathf.Max(endpointJointTravel, Mathf.Abs(goal[j] - start[j]));
            if (endpointJointTravel <= 0.05f && EdgeValid(start, goal, a, ar, b, br))
            {
                result.JointPath.Add((float[])start.Clone());
                result.JointPath.Add((float[])goal.Clone());
                return true;
            }
            // Retry with finer Cartesian samples when joint interpolation cannot preserve the line.
            for (int refinement = 0; refinement < refinements; refinement++)
            {
                int count = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(a, b) / 0.005f),
                    Mathf.CeilToInt(Quaternion.Angle(ar, br) / 2f)) * (1 << refinement);
                if (count > 8192) break;
                var previous = new List<Node> { new Node { Angles = (float[])start.Clone() } };
                bool retryFiner = false;
                for (int i = 1; i <= count; i++)
                {
                    retryFiner = false;
                    world.CancellationToken.ThrowIfCancellationRequested();
                    if (shouldAbort != null && shouldAbort()) { result.FailureReason = "MoveL edge validation stopped at the automatic search deadline."; return false; }
                    result.Iterations++;
                    float t = i / (float)count;
                    Vector3 position = Vector3.Lerp(a, b, t);
                    Quaternion rotation = Quaternion.Slerp(ar, br, t);
                    world.GetTcpPosition(previous[0].Angles);
                    var solutions = i == count && !alternateGoal
                        ? new List<float[]> { goal } : world.SolveIk(position, rotation);
                    if (i == count && alternateGoal)
                    {
                        solutions = solutions == null ? new List<float[]>() : new List<float[]>(solutions);
                        solutions.Add(goal);
                    }
                    var next = new List<Node>();
                    int rawCandidateCount = solutions?.Count ?? 0;
                    var reasons = new HashSet<string>();
                    Vector3 failurePosition = position;
                    // A distant analytical branch cannot connect through a <=5 degree
                    // edge. Reject it before FK, collision, or Jacobian queries.
                    if (solutions != null && solutions.Count > 0)
                    {
                        var reachable = new List<float[]>();
                        float nearestJump = float.PositiveInfinity;
                        int nearestJoint = -1;
                        float nearestFrom = 0f, nearestTo = 0f;
                        foreach (float[] angles in solutions)
                        {
                            if (angles == null || angles.Length != start.Length) continue;
                            float jump = float.PositiveInfinity;
                            foreach (Node parent in previous)
                            {
                                float largest = 0;
                                int largestJoint = -1;
                                for (int j = 0; j < angles.Length; j++)
                                {
                                    float delta = Mathf.Abs(angles[j] - parent.Angles[j]);
                                    if (delta > largest) { largest = delta; largestJoint = j; }
                                }
                                if (largest < nearestJump)
                                {
                                    nearestJump = largest;
                                    nearestJoint = largestJoint;
                                    if (largestJoint >= 0)
                                    {
                                        nearestFrom = parent.Angles[largestJoint];
                                        nearestTo = angles[largestJoint];
                                    }
                                }
                                jump = Mathf.Min(jump, largest);
                            }
                            if (jump <= 5f) reachable.Add(angles);
                        }
                        if (reachable.Count == 0)
                        {
                            retryFiner = true;
                            reasons.Add($"No continuous IK branch: nearest joint jump {nearestJump:F3} degrees exceeds 5 degrees" +
                                (nearestJoint >= 0 ? $" (J{nearestJoint + 1}: {nearestFrom:F3} -> {nearestTo:F3} degrees)" : string.Empty));
                        }
                        solutions = reachable;
                        if (solutions.Count > 0 && world.EvaluateConfigurationBatch != null)
                            world.EvaluateConfigurationBatch(solutions);
                    }
                    if (solutions != null)
                        foreach (float[] angles in solutions)
                        {
                            if (!Valid(angles, position, rotation)) { retryFiner |= refinementCanHelp; reasons.Add(rejection); failurePosition = rejectionPosition; continue; }
                            Node best = null;
                            float cost = float.PositiveInfinity;
                            foreach (Node parent in previous)
                            {
                                float candidateCost = parent.Cost + world.MotionCost(parent.Angles, angles);
                                if (candidateCost >= cost) continue;
                                if (!EdgeValid(parent.Angles, angles,
                                    Vector3.Lerp(a, b, (i - 1f) / count), Quaternion.Slerp(ar, br, (i - 1f) / count),
                                    position, rotation)) { retryFiner |= refinementCanHelp; reasons.Add(rejection); failurePosition = rejectionPosition; continue; }
                                best = parent;
                                cost = candidateCost;
                            }
                            if (best != null) next.Add(new Node { Angles = (float[])angles.Clone(), Previous = best, Cost = cost });
                        }
                    if (next.Count == 0)
                    {
                        result.FailurePosition = failurePosition;
                        result.HasFailurePosition = true;
                        result.FailureReason = $"MoveL rejected at {t:P1} (sample {i}/{count}, {rawCandidateCount} IK candidates, {solutions?.Count ?? 0} within joint-step limit). " +
                            (reasons.Count == 0 ? "Analytical IK returned no solutions." : string.Join("; ", reasons));
                        // A collision at the requested TCP pose or a singular branch
                        // cannot be fixed by resampling the same straight path 2/4/8x.
                        if (!retryFiner) return false;
                        break;
                    }
                    previous = next;
                    if (i != count) continue;
                    Node end = previous[0];
                    foreach (Node candidate in previous) if (candidate.Cost < end.Cost) end = candidate;
                    for (; end != null; end = end.Previous) result.JointPath.Add(end.Angles);
                    result.JointPath.Reverse();
                    result.FailureReason = null;
                    return true;
                }
            }
            if (result.FailureReason == null) result.FailureReason = "MoveL exceeds the sampling budget.";
            return false;
        }

        private bool Valid(float[] angles, Vector3 position, Quaternion rotation)
        {
            rejectionPosition = position;
            if (angles == null || world.JointLimits == null || angles.Length != world.JointLimits.Length) return Reject("Invalid joint count");
            for (int j = 0; j < angles.Length; j++)
                if (float.IsNaN(angles[j]) || float.IsInfinity(angles[j]) ||
                    angles[j] < world.JointLimits[j].x || angles[j] > world.JointLimits[j].y) return Reject($"J{j + 1} violates its joint limits");
            if (world.EvaluateConfiguration != null)
            {
                var check = world.EvaluateConfiguration(angles);
                if (!PoseMatches(check.Position, check.Rotation, position, rotation)) return false;
                if (check.Collision)
                {
                    rejectionPosition = check.Position;
                    return Reject("IK configuration violates collision/clearance constraints: " + check.CollisionDescription);
                }
                return check.SingularityFailure == null || Reject(check.SingularityFailure);
            }
            if (!PoseMatches(angles, position, rotation)) return false;
            if (world.IsConfigurationInCollision(angles)) return RejectCollision("IK configuration violates collision/clearance constraints");
            return HasSingularityMargin(angles);
        }

        private bool PoseMatches(float[] angles, Vector3 position, Quaternion rotation)
        {
            return PoseMatches(world.GetTcpPosition(angles), world.GetTcpRotation(angles), position, rotation);
        }

        private bool PoseMatches(Vector3 actualPosition, Quaternion actualRotation, Vector3 position, Quaternion rotation)
        {
            float distance = Vector3.Distance(actualPosition, position);
            float angle = Quaternion.Angle(actualRotation, rotation);
            return distance <= 0.0005f && angle <= 0.1f ||
                Reject($"Cartesian error {distance * 1000f:F4} mm / {angle:F4} degrees exceeds 0.5 mm / 0.1 degrees", true);
        }

        // Numerical geometric Jacobian. Translation is scaled by a characteristic robot
        // length (one meter by default), angular rows use radians. Reject a small
        // minimum/maximum singular-value ratio; this is separate from jump detection.
        internal string GetSingularityRejection(float[] angles) => HasSingularityMargin(angles) ? null : rejection;
        internal string GetPoseRejection(float[] angles, Vector3 position, Quaternion rotation)
            => Valid(angles, position, rotation) ? null : rejection;

        internal static double[,] GeometricJacobian(Vector3[] origins, Vector3[] axes, Vector3 tcp, float length)
        {
            var matrix = new double[6, 6];
            for (int j = 0; j < 6; j++)
            {
                Vector3 linear = Vector3.Cross(axes[j], tcp - origins[j]) / Mathf.Max(0.001f, length);
                for (int row = 0; row < 3; row++)
                {
                    matrix[row, j] = linear[row];
                    matrix[row + 3, j] = axes[j][row];
                }
            }
            return matrix;
        }

        private bool HasSingularityMargin(float[] angles)
        {
            if (angles.Length != 6) return Reject("Singularity evaluator requires six joints");
            if (world.GetJacobian != null) return CheckJacobian(world.GetJacobian(angles));
            var jacobian = new double[6, 6];
            for (int j = 0; j < 6; j++)
            {
                var low = (float[])angles.Clone();
                var high = (float[])angles.Clone();
                low[j] = Mathf.Max(world.JointLimits[j].x, angles[j] - 0.05f);
                high[j] = Mathf.Min(world.JointLimits[j].y, angles[j] + 0.05f);
                float step = (high[j] - low[j]) * Mathf.Deg2Rad;
                if (step <= 0f) return Reject("Singularity evaluator cannot perturb a locked joint");
                Vector3 translation = (world.GetTcpPosition(high) - world.GetTcpPosition(low)) /
                    (step * Mathf.Max(0.001f, world.LinearCharacteristicLengthMeters));
                Quaternion delta = world.GetTcpRotation(high) * Quaternion.Inverse(world.GetTcpRotation(low));
                // The quaternion vector retains small rotations even when w rounds to 1.
                // Avoid acos/ToAngleAxis, which can erase small finite differences.
                if (delta.w < 0f) delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);
                var vector = new Vector3(delta.x, delta.y, delta.z);
                float length = vector.magnitude;
                Vector3 angular = length < 1e-8f ? vector * (2f / step) :
                    vector * (2f * Mathf.Atan2(length, delta.w) / (length * step));
                for (int row = 0; row < 3; row++)
                {
                    jacobian[row, j] = translation[row];
                    jacobian[row + 3, j] = angular[row];
                }
            }
            return CheckJacobian(jacobian);
        }

        private bool CheckJacobian(double[,] jacobian)
        {
            var gram = new double[6, 6];
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    for (int k = 0; k < 6; k++) gram[i, j] += jacobian[k, i] * jacobian[k, j];
            // Jacobi eigenvalue iteration for the symmetric J-transpose-J matrix.
            for (int iteration = 0; iteration < 100; iteration++)
            {
                int p = 0, q = 1;
                for (int i = 0; i < 6; i++)
                    for (int j = i + 1; j < 6; j++)
                        if (Math.Abs(gram[i, j]) > Math.Abs(gram[p, q])) { p = i; q = j; }
                if (Math.Abs(gram[p, q]) < 1e-12) break;
                double theta = 0.5 * Math.Atan2(2 * gram[p, q], gram[q, q] - gram[p, p]);
                double c = Math.Cos(theta), s = Math.Sin(theta);
                double pp = gram[p, p], qq = gram[q, q], pq = gram[p, q];
                for (int k = 0; k < 6; k++)
                {
                    if (k == p || k == q) continue;
                    double kp = gram[k, p], kq = gram[k, q];
                    gram[k, p] = gram[p, k] = c * kp - s * kq;
                    gram[k, q] = gram[q, k] = s * kp + c * kq;
                }
                gram[p, p] = c*c*pp - 2*s*c*pq + s*s*qq;
                gram[q, q] = s*s*pp + 2*s*c*pq + c*c*qq;
                gram[p, q] = gram[q, p] = 0;
            }
            double minimum = double.PositiveInfinity, maximum = 0;
            for (int i = 0; i < 6; i++)
            {
                if (double.IsNaN(gram[i, i]) || double.IsInfinity(gram[i, i])) return Reject("Nonfinite numerical Jacobian");
                minimum = Math.Min(minimum, gram[i, i]);
                maximum = Math.Max(maximum, gram[i, i]);
            }
            double margin = world.LinearMinimumSingularValueRatio;
            double ratio = maximum > 1e-12 ? Math.Sqrt(Math.Max(0, minimum) / maximum) : 0;
            LastSingularityRatio = ratio;
            return ratio > margin || Reject($"Singularity ratio {ratio:G5}, required > {margin:G5}");
        }
        private bool RejectCollision(string message)
        {
            if (world.GetLastCollisionTcpPosition != null) rejectionPosition = world.GetLastCollisionTcpPosition();
            string pair = world.GetLastCollisionDescription?.Invoke();
            return Reject(string.IsNullOrEmpty(pair) ? message : message + ": " + pair);
        }

        private bool EdgeValid(float[] from, float[] to, Vector3 a, Quaternion ar, Vector3 b, Quaternion br)
        {
            rejectionPosition = b;
            float largest = 0f;
            for (int j = 0; j < from.Length; j++) largest = Mathf.Max(largest, Mathf.Abs(to[j] - from[j]));
            // Reject branch flips using actual joint travel, never wrapped angles.
            if (largest > 5f) return Reject($"Joint jump {largest:F3} degrees exceeds 5 degrees", true);
            if (world.EvaluateConfigurationBatch != null)
            {
                double squared = 0;
                for (int j = 0; j < from.Length; j++) squared += (to[j] - from[j]) * (to[j] - from[j]);
                int count = Math.Max(4, (int)Math.Ceiling(Math.Sqrt(squared) /
                    Math.Max(0.001f, Math.Min(0.25f, world.CollisionResolutionDegrees))));
                var states = new List<float[]>();
                // Include endpoints: this batched path replaces the separate swept
                // collision query and is at least as dense as its configured resolution.
                for (int i = 0; i <= count; i++)
                {
                    var state = new float[from.Length];
                    for (int j = 0; j < state.Length; j++) state[j] = Mathf.Lerp(from[j], to[j], i / (float)count);
                    states.Add(state);
                }
                var checks = world.EvaluateConfigurationBatch(states);
                for (int i = 0; i < checks.Length; i++)
                {
                    var check = checks[i];
                    float t = i / (float)count;
                    rejectionPosition = Vector3.Lerp(a, b, t);
                    if (!PoseMatches(check.Position, check.Rotation, rejectionPosition, Quaternion.Slerp(ar, br, t))) return false;
                    if (check.Collision) { rejectionPosition = check.Position; return Reject("Swept joint segment violates collision/clearance constraints: " + check.CollisionDescription); }
                    if (check.SingularityFailure != null) return Reject(check.SingularityFailure);
                }
                return true;
            }
            if (!world.IsJointSegmentFree(from, to)) return RejectCollision("Swept joint segment violates collision/clearance constraints");
            int samples = Mathf.Max(4, Mathf.CeilToInt(largest / 0.25f));
            var interpolated = new float[from.Length];
            for (int i = 1; i < samples; i++)
            {
                float t = i / (float)samples;
                for (int j = 0; j < from.Length; j++) interpolated[j] = Mathf.Lerp(from[j], to[j], t);
                if (!Valid(interpolated, Vector3.Lerp(a, b, t), Quaternion.Slerp(ar, br, t))) return false;
            }
            return true;
        }
    }
}







