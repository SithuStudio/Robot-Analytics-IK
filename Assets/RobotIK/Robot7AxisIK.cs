using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Damped least-squares pose IK for six- or seven-joint Robot7AxisRig configurations.
    /// The solver is iterative and respects the configured joint limits.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Robot7AxisRig))]
    public sealed class Robot7AxisIK : MonoBehaviour
    {
        [SerializeField] private Robot7AxisRig rig;
        [SerializeField] private Transform target;
        [SerializeField] private bool solveContinuously = true;
        [SerializeField, Min(1)] private int iterationsPerFrame = 8;
        [SerializeField, Min(0.0001f)] private float positionTolerance = 0.002f;
        [SerializeField, Range(0.1f, 10f)] private float orientationToleranceDegrees = 1f;
        [SerializeField, Range(0.1f, 20f)] private float orientationWeight = 1f;
        [SerializeField, Min(0.0001f)] private float damping = 0.08f;
        [SerializeField, Range(0.1f, 45f)] private float maxJointStepDegrees = 8f;

        private float[] angles = new float[Robot7AxisRig.MaximumJointCount];
        private readonly Vector3[] axes = new Vector3[Robot7AxisRig.MaximumJointCount];
        private readonly Vector3[] origins = new Vector3[Robot7AxisRig.MaximumJointCount];
        private readonly double[,] normal = new double[6, 6];
        private readonly double[,] jacobian = new double[6, Robot7AxisRig.MaximumJointCount];
        private readonly double[] error = new double[6];
        private readonly double[] solution = new double[6];
        private readonly double[,] augmented = new double[6, 7];
        private bool initialized;

        public Transform Target { get => target; set => target = value; }
        public float[] JointAnglesDegrees => rig != null ? rig.GetJointAnglesDegrees() : (float[])angles.Clone();

        private void Reset() => rig = GetComponent<Robot7AxisRig>();

        private void Awake()
        {
            if (rig == null) rig = GetComponent<Robot7AxisRig>();
            InitializeAngles();
        }

        private void OnEnable()
        {
            if (rig == null) rig = GetComponent<Robot7AxisRig>();
            if (rig != null && rig.IsReady) InitializeAngles();
        }

        private void Update()
        {
            if (!solveContinuously || target == null || !EnsureRig()) return;
            for (int i = 0; i < iterationsPerFrame; i++)
            {
                if (IsPoseReached(rig.CurrentPose, target.position, target.rotation)) break;
                if (!SolveStep(target.position, target.rotation)) break;
            }
        }

        /// <summary>Sets joint coordinates in degrees, applying configured limits.</summary>
        public void SetJointAngles(float[] degrees)
        {
            if (!EnsureRig() || degrees == null || degrees.Length != rig.JointCount)
            {
                Debug.LogError("SetJointAngles requires one value for each built robot joint.", this);
                return;
            }

            var clamped = new float[rig.JointCount];
            for (int i = 0; i < rig.JointCount; i++)
                clamped[i] = Mathf.Clamp(degrees[i], rig.Joints[i].minimumAngle, rig.Joints[i].maximumAngle);
            rig.SetJointAngles(clamped);
            angles = clamped;
            initialized = true;
        }

        /// <summary>Runs the configured iteration count immediately toward the target pose.</summary>
        public bool SolveNow()
        {
            if (target == null || !EnsureRig()) return false;
            bool converged = false;
            for (int i = 0; i < Mathf.Max(1, iterationsPerFrame); i++)
            {
                converged = IsPoseReached(rig.CurrentPose, target.position, target.rotation);
                if (converged) break;
                if (!SolveStep(target.position, target.rotation)) break;
                converged = IsPoseReached(rig.CurrentPose, target.position, target.rotation);
                if (converged) break;
            }
            return converged;
        }

        /// <summary>Performs one DLS iteration toward a world-space pose.</summary>
        public bool SolveStep(Vector3 goalPosition, Quaternion goalRotation)
        {
            if (!EnsureRig()) return false;
            if (!initialized) InitializeAngles();
            Pose toolPose = rig.CurrentPose;

            Vector3 positionError = goalPosition - toolPose.position;
            Quaternion rotationError = goalRotation * Quaternion.Inverse(toolPose.rotation);
            rotationError.ToAngleAxis(out float angle, out Vector3 rotationAxis);
            if (angle > 180f) angle -= 360f;
            Vector3 angularError = Mathf.Abs(angle) < 0.0001f
                ? Vector3.zero
                : rotationAxis.normalized * (angle * Mathf.Deg2Rad);

            if (IsPoseReached(toolPose, goalPosition, goalRotation))
                return true;

            Transform[] frames = rig.JointFrames;
            for (int i = 0; i < frames.Length; i++)
            {
                axes[i] = frames[i].up;
                origins[i] = frames[i].position;
            }

            // Solve (J J^T + lambda^2 I)y = error, then delta = J^T y.
            // The damped 6x6 solve handles both six-axis and redundant seven-axis arms.
            error[0] = positionError.x;
            error[1] = positionError.y;
            error[2] = positionError.z;
            error[3] = angularError.x * orientationWeight;
            error[4] = angularError.y * orientationWeight;
            error[5] = angularError.z * orientationWeight;
            System.Array.Clear(normal, 0, normal.Length);
            System.Array.Clear(jacobian, 0, jacobian.Length);
            for (int joint = 0; joint < rig.JointCount; joint++)
            {
                Vector3 linear = Vector3.Cross(axes[joint], toolPose.position - origins[joint]);
                jacobian[0, joint] = linear.x;
                jacobian[1, joint] = linear.y;
                jacobian[2, joint] = linear.z;
                jacobian[3, joint] = axes[joint].x * orientationWeight;
                jacobian[4, joint] = axes[joint].y * orientationWeight;
                jacobian[5, joint] = axes[joint].z * orientationWeight;
            }

            for (int row = 0; row < 6; row++)
                for (int col = 0; col < 6; col++)
                    for (int joint = 0; joint < rig.JointCount; joint++)
                        normal[row, col] += jacobian[row, joint] * jacobian[col, joint];

            double lambdaSquared = damping * damping;
            for (int i = 0; i < 6; i++) normal[i, i] += lambdaSquared;
            if (!SolveLinearSystem(normal, error, augmented, solution)) return false;

            for (int joint = 0; joint < rig.JointCount; joint++)
            {
                double delta = 0;
                for (int row = 0; row < 6; row++) delta += jacobian[row, joint] * solution[row];
                float stepDegrees = Mathf.Clamp((float)(delta * Mathf.Rad2Deg),
                    -maxJointStepDegrees, maxJointStepDegrees);
                DHJoint rowData = rig.Joints[joint];
                angles[joint] = Mathf.Clamp(angles[joint] + stepDegrees,
                    rowData.minimumAngle, rowData.maximumAngle);
            }

            if (!rig.SetJointAngles(angles)) return false;
            angles = rig.GetJointAnglesDegrees();
            return true;
        }

        private bool IsPoseReached(Pose toolPose, Vector3 goalPosition, Quaternion goalRotation)
        {
            float orientationError = Quaternion.Angle(toolPose.rotation, goalRotation);
            return (goalPosition - toolPose.position).sqrMagnitude <= positionTolerance * positionTolerance &&
                   orientationError <= orientationToleranceDegrees;
        }

        private void InitializeAngles()
        {
            if (rig == null || !rig.IsReady) return;
            angles = rig.GetJointAnglesDegrees();
            initialized = true;
        }

        private bool EnsureRig()
        {
            if (rig == null) rig = GetComponent<Robot7AxisRig>();
            if (rig == null || rig.JointCount < Robot7AxisRig.MinimumJointCount ||
                rig.JointCount > Robot7AxisRig.MaximumJointCount || !rig.RefreshJointFrames() ||
                rig.JointFrames.Length != rig.JointCount)
            {
                Debug.LogWarning("Build a six- or seven-axis DH rig before running IK.", this);
                return false;
            }
            return true;
        }

        private static bool SolveLinearSystem(double[,] matrix, double[] rhs,
            double[,] augmented, double[] solution)
        {
            const int size = 6;
            for (int row = 0; row < size; row++)
            {
                for (int col = 0; col < size; col++) augmented[row, col] = matrix[row, col];
                augmented[row, size] = rhs[row];
            }

            for (int pivot = 0; pivot < size; pivot++)
            {
                int best = pivot;
                for (int row = pivot + 1; row < size; row++)
                    if (System.Math.Abs(augmented[row, pivot]) > System.Math.Abs(augmented[best, pivot]))
                        best = row;
                if (System.Math.Abs(augmented[best, pivot]) < 1e-12) return false;

                if (best != pivot)
                    for (int col = pivot; col <= size; col++)
                        (augmented[pivot, col], augmented[best, col]) =
                            (augmented[best, col], augmented[pivot, col]);

                double divisor = augmented[pivot, pivot];
                for (int col = pivot; col <= size; col++) augmented[pivot, col] /= divisor;
                for (int row = 0; row < size; row++)
                {
                    if (row == pivot) continue;
                    double factor = augmented[row, pivot];
                    for (int col = pivot; col <= size; col++)
                        augmented[row, col] -= factor * augmented[pivot, col];
                }
            }

            for (int row = 0; row < size; row++) solution[row] = augmented[row, size];
            return true;
        }
    }
}
