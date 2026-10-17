using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Closed-form inverse kinematics for a six-axis arm with a vertical base axis,
    /// parallel shoulder/elbow axes, and a concurrent spherical wrist.
    /// Geometry is measured from the bound RobotBaseSixAxisRig at its zero pose.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RobotBaseSixAxisRig))]
    public sealed class RobotBaseSixAxisAnalyticIK : MonoBehaviour
    {
        public enum ShoulderPosture
        {
            Automatic,
            Left,
            Right
        }

        public enum ElbowPosture
        {
            Automatic,
            Up,
            Down
        }

        public enum WristPosture
        {
            Automatic,
            NonFlip,
            Flip
        }

        private const float GeometryTolerance = 0.015f;
        private const float AxisTolerance = 0.025f;
        private const float Epsilon = 1e-6f;

        [SerializeField] private RobotBaseSixAxisRig rig;
        [Tooltip("The TCP local +Z axis follows this Transform's local +Z axis.")]
        [SerializeField] private Transform target;
        [SerializeField] private bool solveContinuously;
        [Tooltip("Keeps TCP local +Z aligned with Target local +Z.")]
        [SerializeField] private bool alignTcpZToTargetZ = true;
        [Tooltip("Tracks Target rotation around its local +Z axis. Enable this when J6 should follow Target roll.")]
        [SerializeField] private bool trackTargetRoll = true;
        [SerializeField, Min(0f)] private float positionTolerance = 0.001f;
        [SerializeField, Min(0f)] private float orientationToleranceDegrees = 0.25f;
        [SerializeField] private bool chooseNearestSolution = true;
        [Header("Analytical Branch Selection")]
        [SerializeField] private ShoulderPosture shoulderPosture = ShoulderPosture.Automatic;
        [Tooltip("Up selects the positive elbow-plane branch; Down selects the negative branch.")]
        [SerializeField] private ElbowPosture elbowPosture = ElbowPosture.Automatic;
        [SerializeField] private WristPosture wristPosture = WristPosture.Automatic;

        private readonly Vector3[] homeOrigins = new Vector3[RobotBaseSixAxisRig.JointCount];
        private readonly Vector3[] homeAxes = new Vector3[RobotBaseSixAxisRig.JointCount];
        private Quaternion homeToolRotation;
        private Vector3 wristCenter;
        private Vector3 tcpOffsetInTool;
        private Vector3 firstLink;
        private Vector3 secondLink;
        private float link1Length;
        private float link2Length;
        private float shoulderAxisOffset;
        private float elbowAxisSign;
        private float wristRollSign;
        private float[] anglesToRestore;
        private bool calibrated;

        public Transform Target { get => target; set => target = value; }
        /// <summary>When enabled, Update continuously overrides external joint motion.</summary>
        public bool SolveContinuously { get => solveContinuously; set => solveContinuously = value; }
        public bool IsCalibrated => calibrated;
        public string CalibrationStatus { get; private set; } = "Not calibrated";

        private void Reset() => rig = GetComponent<RobotBaseSixAxisRig>();

        private void Awake()
        {
            if (rig == null) rig = GetComponent<RobotBaseSixAxisRig>();
            CalibrateFromRig();
        }

        private void OnEnable()
        {
            if (!calibrated) CalibrateFromRig();
        }

        private void Update()
        {
            if (solveContinuously && target != null)
                SolveNow(target.position, GetTargetOrientation(), out _);
        }

        /// <summary>
        /// Captures the actual imported pivot layout at zero joint coordinates and checks
        /// the geometric conditions required by this closed-form solver.
        /// </summary>
        [ContextMenu("Calibrate Analytical IK From Joint Pivots")]
        public bool CalibrateFromRig()
        {
            calibrated = false;
            if (rig == null) rig = GetComponent<RobotBaseSixAxisRig>();
            if (rig == null || !rig.IsReady)
                return FailCalibration("RobotBaseSixAxisRig is missing or has unbound joints.");

            anglesToRestore = rig.GetJointAnglesDegrees();
            rig.RestoreHH020AnalyticalReferencePose();
            rig.SetJointAngles(new float[RobotBaseSixAxisRig.JointCount]);
            Transform[] joints = rig.Joints;
            Transform tcp = rig.ToolFrame != null ? rig.ToolFrame : joints[5];
            Transform baseFrame = rig.Base;

            for (int i = 0; i < RobotBaseSixAxisRig.JointCount; i++)
            {
                homeOrigins[i] = baseFrame.InverseTransformPoint(joints[i].position);
                homeAxes[i] = baseFrame.InverseTransformDirection(rig.GetWorldAxis(i)).normalized;
                if (homeAxes[i].sqrMagnitude < 0.99f)
                    return FailCalibration($"J{i + 1} has an invalid rotation axis.");
            }

            // The HH020 has a concurrent wrist: axes J4, J5, and J6 share one point.
            if (!TryIntersectThreeLines(homeOrigins[3], homeAxes[3],
                    homeOrigins[4], homeAxes[4], homeOrigins[5], homeAxes[5], out wristCenter))
                return FailCalibration("Could not calculate the J4-J6 wrist-axis intersection.");

            float wristError = Mathf.Max(
                DistanceToLine(wristCenter, homeOrigins[3], homeAxes[3]),
                DistanceToLine(wristCenter, homeOrigins[4], homeAxes[4]),
                DistanceToLine(wristCenter, homeOrigins[5], homeAxes[5]));
            if (wristError > GeometryTolerance)
                return FailCalibration($"J4-J6 axes miss their common wrist center by {wristError:F4} m. Check pivot and axis setup.");

            Vector3 baseAxis = homeAxes[0];
            Vector3 shoulderAxis = homeAxes[1];
            Vector3 elbowAxis = homeAxes[2];
            if (Mathf.Abs(Vector3.Dot(baseAxis, shoulderAxis)) > AxisTolerance)
                return FailCalibration("J1 must be perpendicular to J2 for this HH020 shoulder/elbow solution.");
            if (Mathf.Abs(Vector3.Dot(shoulderAxis, elbowAxis)) < 1f - AxisTolerance)
                return FailCalibration("J2 and J3 must be parallel for this HH020 shoulder/elbow solution.");
            float j4J6Dot = Mathf.Abs(Vector3.Dot(homeAxes[3], homeAxes[5]));
            float j4J5Dot = Mathf.Abs(Vector3.Dot(homeAxes[3], homeAxes[4]));
            if (j4J6Dot < 1f - AxisTolerance || j4J5Dot > AxisTolerance)
                return FailCalibration(
                    $"J4 and J6 must be parallel, with J5 perpendicular, for the spherical-wrist solution " +
                    $"(abs dots J4/J6={j4J6Dot:F4}, J4/J5={j4J5Dot:F4}).");

            Vector3 shoulderToElbow = homeOrigins[2] - homeOrigins[1];
            Vector3 elbowToWrist = wristCenter - homeOrigins[2];
            shoulderToElbow -= shoulderAxis * Vector3.Dot(shoulderToElbow, shoulderAxis);
            elbowToWrist -= shoulderAxis * Vector3.Dot(elbowToWrist, shoulderAxis);
            link1Length = shoulderToElbow.magnitude;
            link2Length = elbowToWrist.magnitude;
            if (link1Length < Epsilon || link2Length < Epsilon)
                return FailCalibration("The shoulder/elbow link lengths are zero. Check J2, J3, and wrist pivots.");

            // Keep the measured vectors in the zero-pose shoulder/elbow plane.
            firstLink = shoulderToElbow;
            secondLink = elbowToWrist;
            shoulderAxisOffset = Vector3.Dot(wristCenter - homeOrigins[0], shoulderAxis);
            elbowAxisSign = Mathf.Sign(Vector3.Dot(shoulderAxis, elbowAxis));
            wristRollSign = Mathf.Sign(Vector3.Dot(homeAxes[3], homeAxes[5]));
            homeToolRotation = Quaternion.Inverse(baseFrame.rotation) * tcp.rotation;
            Vector3 tcpPosition = baseFrame.InverseTransformPoint(tcp.position);
            tcpOffsetInTool = Quaternion.Inverse(homeToolRotation) * (tcpPosition - wristCenter);

            calibrated = true;
            CalibrationStatus = $"Ready. Link lengths: {link1Length:F3} m, {link2Length:F3} m; wrist-axis error: {wristError:F4} m.";
            RestoreAnglesAfterCalibration();
            return true;
        }

        /// <summary>Returns every reachable, limit-respecting analytical branch.</summary>
        public bool SolveAll(Vector3 targetPosition, Quaternion targetRotation, out List<float[]> solutions)
        {
            solutions = new List<float[]>(8);
            if (!calibrated && !CalibrateFromRig()) return false;

            Transform baseFrame = rig.Base;
            Vector3 goalPosition = baseFrame.InverseTransformPoint(targetPosition);
            Quaternion goalRotation = Quaternion.Inverse(baseFrame.rotation) * targetRotation;
            Vector3 goalWrist = goalPosition - goalRotation * tcpOffsetInTool;

            Vector3 baseAxis = homeAxes[0];
            Vector3 shoulderAxis = homeAxes[1];
            Vector3 baseToGoal = goalWrist - homeOrigins[0];
            Vector3 projectedGoal = baseToGoal - baseAxis * Vector3.Dot(baseToGoal, baseAxis);
            float projectedDistance = projectedGoal.magnitude;
            if (projectedDistance < Epsilon || Mathf.Abs(shoulderAxisOffset) > projectedDistance + GeometryTolerance)
                return false;

            Vector3 projectedDirection = projectedGoal / projectedDistance;
            Vector3 sideDirection = Vector3.Cross(baseAxis, projectedDirection).normalized;
            float along = shoulderAxisOffset / projectedDistance;
            if (Mathf.Abs(along) > 1f + AxisTolerance) return false;
            along = Mathf.Clamp(along, -1f, 1f);
            float side = Mathf.Sqrt(Mathf.Max(0f, 1f - along * along));

            var baseAngles = new float[2];
            baseAngles[0] = SignedAngleAround(baseAxis, shoulderAxis,
                projectedDirection * along + sideDirection * side);
            baseAngles[1] = SignedAngleAround(baseAxis, shoulderAxis,
                projectedDirection * along - sideDirection * side);

            Vector3 linkAxis = shoulderAxis;
            Vector3 linkX = firstLink.normalized;
            Vector3 linkY = Vector3.Cross(linkAxis, linkX).normalized;
            float linkDelta = Mathf.Atan2(Vector3.Dot(secondLink, linkY), Vector3.Dot(secondLink, linkX));
            // Use the actual shoulder-to-wrist radius for the elbow law of cosines.
            for (int baseBranch = 0; baseBranch < baseAngles.Length; baseBranch++)
            {
                if (!AllowsShoulderBranch(baseBranch)) continue;
                float q1 = baseAngles[baseBranch];
                Quaternion rotateBase = Quaternion.AngleAxis(q1 * Mathf.Rad2Deg, baseAxis);
                Vector3 shoulder = homeOrigins[0] + rotateBase * (homeOrigins[1] - homeOrigins[0]);
                Vector3 shoulderToWrist = Quaternion.Inverse(rotateBase) * (goalWrist - shoulder);
                Vector3 planarGoal = shoulderToWrist - linkAxis * Vector3.Dot(shoulderToWrist, linkAxis);
                float radiusSquared = planarGoal.sqrMagnitude;
                float elbowCosine = (radiusSquared - link1Length * link1Length - link2Length * link2Length) /
                                    (2f * link1Length * link2Length);
                if (elbowCosine < -1f - AxisTolerance || elbowCosine > 1f + AxisTolerance) continue;
                elbowCosine = Mathf.Clamp(elbowCosine, -1f, 1f);

                float goalAngle = Mathf.Atan2(Vector3.Dot(planarGoal, linkY), Vector3.Dot(planarGoal, linkX));
                float elbowMagnitude = Mathf.Acos(elbowCosine);
                for (int elbowBranch = 0; elbowBranch < 2; elbowBranch++)
                {
                    if (!AllowsElbowBranch(elbowBranch)) continue;
                    float q3Effective = elbowBranch == 0 ? elbowMagnitude : -elbowMagnitude;
                    float q2 = goalAngle - Mathf.Atan2(link2Length * Mathf.Sin(q3Effective),
                        link1Length + link2Length * Mathf.Cos(q3Effective));
                    float q3 = (q3Effective - linkDelta) / elbowAxisSign;

                    Quaternion firstThree = AxisRotation(homeAxes[0], q1) *
                                            AxisRotation(homeAxes[1], q2) *
                                            AxisRotation(homeAxes[2], q3);
                    Quaternion orientationDelta = goalRotation * Quaternion.Inverse(homeToolRotation);
                    Quaternion wrist = Quaternion.Inverse(firstThree) * orientationDelta;
                    if (!TryDecomposeWrist(wrist, out float q4a, out float q5a, out float q6a,
                            out float q4b, out float q5b, out float q6b))
                        continue;

                    if (AllowsWristBranch(0))
                        AddLimitedSolution(solutions, q1, q2, q3, q4a, q5a, q6a);
                    if (AllowsWristBranch(1))
                        AddLimitedSolution(solutions, q1, q2, q3, q4b, q5b, q6b);
                }
            }

            return solutions.Count > 0;
        }

        /// <summary>Selects a valid branch, applies it, and checks the resulting FK pose.</summary>
        public bool SolveNow(Vector3 targetPosition, Quaternion targetRotation, out float[] selectedSolution)
        {
            selectedSolution = null;
            if (!SolveAll(targetPosition, targetRotation, out List<float[]> solutions))
            {
                if (calibrated) CalibrationStatus = "Target is unreachable or no branch satisfies the configured joint limits.";
                return false;
            }

            float[] current = rig.GetJointAnglesDegrees();
            float[] bestSolution = null;
            float bestCost = float.PositiveInfinity;
            for (int i = 0; i < solutions.Count; i++)
            {
                if (!rig.SetJointAngles(solutions[i])) continue;
                Transform candidateTcp = rig.ToolFrame != null ? rig.ToolFrame : rig.Joints[5];
                float candidatePositionError = Vector3.Distance(candidateTcp.position, targetPosition);
                float candidateRotationError = Quaternion.Angle(candidateTcp.rotation, targetRotation);
                if (candidatePositionError > positionTolerance || candidateRotationError > orientationToleranceDegrees)
                    continue;
                if (!chooseNearestSolution)
                {
                    bestSolution = (float[])solutions[i].Clone();
                    break;
                }

                float cost = 0f;
                for (int joint = 0; joint < RobotBaseSixAxisRig.JointCount; joint++)
                {
                    float weight = joint == 5 ? 0.1f : 1f;
                    float delta = Mathf.DeltaAngle(current[joint], solutions[i][joint]);
                    cost += weight * delta * delta;
                }
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestSolution = (float[])solutions[i].Clone();
                }
            }

            rig.SetJointAngles(current);
            if (bestSolution == null)
            {
                CalibrationStatus = $"No {BranchSelectionLabel()} analytical branch passed forward-kinematics closure. " +
                                    "Check the selected posture, pivot placement, axis directions, and TCP frame.";
                return false;
            }

            rig.SetJointAngles(bestSolution);
            selectedSolution = bestSolution;
            CalibrationStatus = $"Solved with {solutions.Count} valid {BranchSelectionLabel()} analytical branch(es).";
            return true;
        }

        public bool SolveNow(out float[] selectedSolution)
        {
            if (target == null)
            {
                selectedSolution = null;
                return false;
            }
            return SolveNow(target.position, GetTargetOrientation(), out selectedSolution);
        }

        private Quaternion GetTargetOrientation()
        {
            if (target == null) return Quaternion.identity;
            if (!alignTcpZToTargetZ || trackTargetRoll) return target.rotation;

            Vector3 targetZ = target.forward;
            Vector3 stableUp = Vector3.ProjectOnPlane(Vector3.up, targetZ);
            if (stableUp.sqrMagnitude < Epsilon)
                stableUp = Vector3.ProjectOnPlane(target.up, targetZ);
            return Quaternion.LookRotation(targetZ, stableUp.normalized);
        }

        private bool AllowsShoulderBranch(int branch) =>
            shoulderPosture == ShoulderPosture.Automatic ||
            (shoulderPosture == ShoulderPosture.Left ? branch == 0 : branch == 1);

        private bool AllowsElbowBranch(int branch) =>
            elbowPosture == ElbowPosture.Automatic ||
            (elbowPosture == ElbowPosture.Up ? branch == 0 : branch == 1);

        private bool AllowsWristBranch(int branch) =>
            wristPosture == WristPosture.Automatic ||
            (wristPosture == WristPosture.NonFlip ? branch == 0 : branch == 1);

        private string BranchSelectionLabel() =>
            $"{shoulderPosture} shoulder / {elbowPosture} elbow / {wristPosture} wrist";

        [ContextMenu("Solve Current Target Now")]
        private void SolveTargetFromContextMenu()
        {
            if (!SolveNow(out _))
                Debug.LogWarning($"Analytical IK did not reach the target. {CalibrationStatus}", this);
        }

        private bool TryDecomposeWrist(Quaternion wrist, out float q4a, out float q5a, out float q6a,
            out float q4b, out float q5b, out float q6b)
        {
            Vector3 xAxis = homeAxes[3].normalized;
            Vector3 yAxis = homeAxes[4] - xAxis * Vector3.Dot(xAxis, homeAxes[4]);
            if (yAxis.sqrMagnitude < Epsilon)
            {
                q4a = q5a = q6a = q4b = q5b = q6b = 0f;
                return false;
            }
            yAxis.Normalize();
            Vector3 zAxis = Vector3.Cross(xAxis, yAxis).normalized;
            Quaternion basis = Quaternion.LookRotation(zAxis, yAxis);
            Quaternion local = Quaternion.Inverse(basis) * wrist * basis;

            Vector3 firstColumn = local * Vector3.right;
            Vector3 secondColumn = local * Vector3.up;
            float cosine = Mathf.Clamp(firstColumn.x, -1f, 1f);
            float middle = Mathf.Acos(cosine);
            float sine = Mathf.Sin(middle);
            float first, third;
            if (Mathf.Abs(sine) > 1e-5f)
            {
                first = Mathf.Atan2(firstColumn.y, -firstColumn.z);
                third = Mathf.Atan2(secondColumn.x, (local * Vector3.forward).x);
            }
            else
            {
                // At wrist singularities q4 and q6 are coupled. Keep q4 at zero and
                // assign the observable combined roll to q6.
                first = 0f;
                float combined = Mathf.Atan2(secondColumn.z, secondColumn.y);
                third = cosine >= 0f ? combined : -combined;
            }

            q4a = first;
            q5a = middle;
            q6a = third / wristRollSign;
            q4b = first + Mathf.PI;
            q5b = -middle;
            q6b = (third + Mathf.PI) / wristRollSign;
            return true;
        }

        private void AddLimitedSolution(List<float[]> solutions,
            float q1, float q2, float q3, float q4, float q5, float q6)
        {
            float[] radians = { q1, q2, q3, q4, q5, q6 };
            var result = new float[RobotBaseSixAxisRig.JointCount];
            Vector2[] limits = rig.AngleLimitsDegrees;
            float[] current = rig.GetJointAnglesDegrees();
            for (int i = 0; i < result.Length; i++)
            {
                float degrees = radians[i] * Mathf.Rad2Deg;
                if (!TryEquivalentWithinLimits(degrees, current[i], limits[i], out result[i])) return;
            }

            for (int i = 0; i < solutions.Count; i++)
            {
                bool duplicate = true;
                for (int j = 0; j < result.Length; j++)
                    if (Mathf.Abs(Mathf.DeltaAngle(result[j], solutions[i][j])) > 0.01f)
                    {
                        duplicate = false;
                        break;
                    }
                if (duplicate) return;
            }
            solutions.Add(result);
        }

        private static bool TryEquivalentWithinLimits(float degrees, float current, Vector2 limits, out float best)
        {
            best = 0f;
            float distance = float.PositiveInfinity;
            bool found = false;
            for (int turn = -2; turn <= 2; turn++)
            {
                float candidate = degrees + turn * 360f;
                if (candidate < limits.x - 0.001f || candidate > limits.y + 0.001f) continue;
                float candidateDistance = Mathf.Abs(candidate - current);
                if (candidateDistance >= distance) continue;
                best = Mathf.Clamp(candidate, limits.x, limits.y);
                distance = candidateDistance;
                found = true;
            }
            return found;
        }

        private bool FailCalibration(string message)
        {
            CalibrationStatus = message;
            RestoreAnglesAfterCalibration();
            Debug.LogError($"Analytical IK calibration failed: {message}", this);
            return false;
        }

        private void RestoreAnglesAfterCalibration()
        {
            if (anglesToRestore == null) return;
            float[] saved = anglesToRestore;
            anglesToRestore = null;
            if (rig != null) rig.SetJointAngles(saved);
        }

        private static float SignedAngleAround(Vector3 axis, Vector3 from, Vector3 to)
        {
            from = Vector3.ProjectOnPlane(from, axis).normalized;
            to = Vector3.ProjectOnPlane(to, axis).normalized;
            return Mathf.Atan2(Vector3.Dot(axis, Vector3.Cross(from, to)), Vector3.Dot(from, to));
        }

        private static Quaternion AxisRotation(Vector3 axis, float radians) =>
            Quaternion.AngleAxis(radians * Mathf.Rad2Deg, axis.normalized);

        private static float DistanceToLine(Vector3 point, Vector3 linePoint, Vector3 lineDirection) =>
            Vector3.Cross(point - linePoint, lineDirection.normalized).magnitude;

        private static bool TryIntersectThreeLines(Vector3 p0, Vector3 u0,
            Vector3 p1, Vector3 u1, Vector3 p2, Vector3 u2, out Vector3 intersection)
        {
            var matrix = new double[3, 3];
            var rhs = new double[3];
            AddLineConstraint(p0, u0, matrix, rhs);
            AddLineConstraint(p1, u1, matrix, rhs);
            AddLineConstraint(p2, u2, matrix, rhs);
            if (!Solve3x3(matrix, rhs, out double[] solution))
            {
                intersection = Vector3.zero;
                return false;
            }
            intersection = new Vector3((float)solution[0], (float)solution[1], (float)solution[2]);
            return true;
        }

        private static void AddLineConstraint(Vector3 point, Vector3 axis, double[,] matrix, double[] rhs)
        {
            axis.Normalize();
            double[] p = { point.x, point.y, point.z };
            double[] u = { axis.x, axis.y, axis.z };
            double dot = p[0] * u[0] + p[1] * u[1] + p[2] * u[2];
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++)
                    matrix[row, col] += (row == col ? 1d : 0d) - u[row] * u[col];
                rhs[row] += p[row] - u[row] * dot;
            }
        }

        private static bool Solve3x3(double[,] matrix, double[] rhs, out double[] solution)
        {
            var augmented = new double[3, 4];
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++) augmented[row, col] = matrix[row, col];
                augmented[row, 3] = rhs[row];
            }

            for (int pivot = 0; pivot < 3; pivot++)
            {
                int best = pivot;
                for (int row = pivot + 1; row < 3; row++)
                    if (Math.Abs(augmented[row, pivot]) > Math.Abs(augmented[best, pivot])) best = row;
                if (Math.Abs(augmented[best, pivot]) < 1e-12)
                {
                    solution = null;
                    return false;
                }
                if (best != pivot)
                    for (int col = pivot; col < 4; col++)
                        (augmented[pivot, col], augmented[best, col]) =
                            (augmented[best, col], augmented[pivot, col]);

                double divisor = augmented[pivot, pivot];
                for (int col = pivot; col < 4; col++) augmented[pivot, col] /= divisor;
                for (int row = 0; row < 3; row++)
                {
                    if (row == pivot) continue;
                    double factor = augmented[row, pivot];
                    for (int col = pivot; col < 4; col++)
                        augmented[row, col] -= factor * augmented[pivot, col];
                }
            }

            solution = new[] { augmented[0, 3], augmented[1, 3], augmented[2, 3] };
            return true;
        }
    }
}
