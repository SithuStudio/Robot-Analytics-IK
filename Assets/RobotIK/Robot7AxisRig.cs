using System;
using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Standard DH row mapped into Unity's Y-up coordinates. Lengths a and d are millimeters;
    /// angles are degrees. The transform is Ry(theta) Ty(d) Tx(a) Rx(alpha).
    /// </summary>
    [Serializable]
    public sealed class DHJoint
    {
        public string jointName = "Joint";
        public float a;
        public float alpha;
        public float d;
        public float thetaOffset;
        public float minimumAngle = -180f;
        public float maximumAngle = 180f;
        public float initialAngle;
        public GameObject linkVisual;
    }

    /// <summary>Six- or seven-axis DH forward kinematics and joint hierarchy binding.</summary>
    [DisallowMultipleComponent]
    public sealed class Robot7AxisRig : MonoBehaviour
    {
        public const int DefaultJointCount = 7;
        public const int MinimumJointCount = 6;
        public const int MaximumJointCount = 7;

        [SerializeField] private Transform baseFrame;
        [SerializeField] private Transform toolFrame;
        [SerializeField] private DHJoint[] joints = CreateDefaultJoints(DefaultJointCount);
        [SerializeField, HideInInspector] private Transform generatedRoot;
        [Header("TCP Offset")]
        [SerializeField] private Vector3 toolOffsetMillimeters;
        [SerializeField] private Vector3 toolRotationOffsetDegrees;
        [Header("Scene Gizmos")]
        [SerializeField] private bool showGizmos = true;
        [SerializeField] private bool showGizmoLabels = true;
        [SerializeField, Min(0.005f)] private float jointGizmoRadius = 0.025f;
        [SerializeField] private Color jointGizmoColor = new Color(1f, 0.35f, 0.1f, 1f);
        [SerializeField] private Color linkGizmoColor = new Color(0.1f, 0.8f, 1f, 1f);
        [Header("DH Length Units")]
        [SerializeField, Min(0.000001f), Tooltip("Unity units per DH millimeter. Use 0.001 when 1 Unity unit represents 1 meter; use 1 when 1 Unity unit represents 1 millimeter.")]
        private float unityUnitsPerMillimeter = 0.001f;

        [NonSerialized] private Transform[] jointFrames = Array.Empty<Transform>();
        [NonSerialized] private float[] jointAngles = Array.Empty<float>();
        [NonSerialized] private Vector3[] jointOrigins = Array.Empty<Vector3>();
        [NonSerialized] private Quaternion[] jointRotations = Array.Empty<Quaternion>();
        [NonSerialized] private Pose currentPose;
        [NonSerialized] private bool anglesInitialized;

        public Transform BaseFrame => baseFrame != null ? baseFrame : transform;
        public Transform ToolFrame => toolFrame;
        public DHJoint[] Joints => joints;
        public int JointCount => joints != null ? joints.Length : 0;
        public Transform[] JointFrames
        {
            get
            {
                RefreshJointFrames();
                return jointFrames;
            }
        }
        public Pose CurrentPose => currentPose;
        public bool IsReady => HasValidConfiguration() && JointFrames.Length == JointCount;
        public float UnityUnitsPerMillimeter => Mathf.Max(0.000001f, unityUnitsPerMillimeter);

        private static DHJoint[] CreateDefaultJoints(int count)
        {
            var result = new DHJoint[count];
            for (int i = 0; i < result.Length; i++)
                result[i] = new DHJoint { jointName = $"J{i + 1}" };
            return result;
        }

        private void Reset()
        {
            baseFrame = transform;
            if (joints == null || joints.Length < MinimumJointCount || joints.Length > MaximumJointCount)
                joints = CreateDefaultJoints(DefaultJointCount);
        }

        private void OnEnable()
        {
            RefreshJointFrames();
            if (IsReady)
            {
                InitializeJointAngles();
                ApplyForwardKinematics();
            }
        }

        /// <summary>
        /// Uses an existing nested J1..Jn hierarchy when present. Otherwise creates a DH
        /// transform chain under the base. Existing child joint objects are never duplicated.
        /// </summary>
        [ContextMenu("Build or Bind DH Rig")]
        public void BuildRig()
        {
            if (!HasValidConfiguration())
            {
                Debug.LogError("Robot rig requires six or seven DH joint rows.", this);
                return;
            }

            if (generatedRoot == null && TryFindManualJointFrames(out Transform[] manualFrames))
            {
                jointFrames = manualFrames;
                ResizeRuntimeBuffers();
                InitializeJointAngles();
                ApplyForwardKinematics();
                return;
            }

            if (generatedRoot != null)
            {
#if UNITY_EDITOR
                if (!Application.isPlaying) DestroyImmediate(generatedRoot.gameObject);
                else Destroy(generatedRoot.gameObject);
#else
                Destroy(generatedRoot.gameObject);
#endif
            }

            var root = new GameObject($"Robot{JointCount}Axis_Rig");
            generatedRoot = root.transform;
            generatedRoot.SetParent(BaseFrame, false);
            generatedRoot.localPosition = Vector3.zero;
            generatedRoot.localRotation = Quaternion.identity;

            var frames = new Transform[JointCount];
            Transform parent = generatedRoot;
            float scale = UnityUnitsPerMillimeter;
            for (int i = 0; i < JointCount; i++)
            {
                DHJoint row = joints[i];
                if (row == null) row = joints[i] = new DHJoint { jointName = $"J{i + 1}" };
                NormalizeLimits(row);

                var pivot = new GameObject($"{row.jointName}_Pivot").transform;
                pivot.SetParent(parent, false);
                pivot.localRotation = Quaternion.identity;
                frames[i] = pivot;

                var dhLink = new GameObject($"{row.jointName}_DH_Link").transform;
                dhLink.SetParent(pivot, false);
                dhLink.localPosition = new Vector3(row.a * scale, row.d * scale, 0f);
                dhLink.localRotation = Quaternion.AngleAxis(row.alpha, Vector3.right);

                if (row.linkVisual != null)
                {
#if UNITY_EDITOR
                    GameObject visual = Application.isPlaying
                        ? Instantiate(row.linkVisual, dhLink)
                        : (GameObject)UnityEditor.PrefabUtility.InstantiatePrefab(row.linkVisual, dhLink);
#else
                    GameObject visual = Instantiate(row.linkVisual, dhLink);
#endif
                    visual.name = $"{row.jointName}_Visual";
                    visual.transform.localPosition = Vector3.zero;
                    visual.transform.localRotation = Quaternion.identity;
                }
                parent = dhLink;
            }

            var tcp = new GameObject("ToolFrame").transform;
            tcp.SetParent(parent, false);
            tcp.localPosition = toolOffsetMillimeters * scale;
            tcp.localRotation = Quaternion.Euler(toolRotationOffsetDegrees);
            toolFrame = tcp;
            jointFrames = frames;
            ResizeRuntimeBuffers();
            InitializeJointAngles();
            ApplyForwardKinematics();
        }

        /// <summary>Refreshes frame references from the generated rig or nested J1..Jn objects.</summary>
        public bool RefreshJointFrames()
        {
            if (!HasValidConfiguration()) return false;

            if (generatedRoot != null)
            {
                if (!CacheGeneratedFrames()) return false;
            }
            else if (jointFrames == null || jointFrames.Length != JointCount || ContainsNull(jointFrames))
            {
                if (!TryFindManualJointFrames(out jointFrames)) return false;
            }

            ResizeRuntimeBuffers();
            return true;
        }

        /// <summary>Returns the FK pose for the supplied joint angles without changing the rig.</summary>
        public Pose EvaluateForwardKinematics(float[] anglesDegrees)
        {
            if (!HasValidConfiguration() || anglesDegrees == null || anglesDegrees.Length != JointCount)
                throw new ArgumentException("Forward kinematics requires one angle for every DH row.", nameof(anglesDegrees));

            Vector3 position = BaseFrame.position;
            Quaternion rotation = BaseFrame.rotation;
            float scale = UnityUnitsPerMillimeter;
            for (int i = 0; i < JointCount; i++)
            {
                DHJoint row = joints[i];
                Quaternion jointRotation = rotation * Quaternion.AngleAxis(
                    anglesDegrees[i] + row.thetaOffset, Vector3.up);
                position += jointRotation * new Vector3(row.a * scale, row.d * scale, 0f);
                rotation = jointRotation * Quaternion.AngleAxis(row.alpha, Vector3.right);
            }

            position += rotation * (toolOffsetMillimeters * scale);
            rotation *= Quaternion.Euler(toolRotationOffsetDegrees);
            return new Pose(position, rotation);
        }

        /// <summary>Gets a copy of the current joint angles in degrees.</summary>
        public float[] GetJointAnglesDegrees()
        {
            EnsureJointAngleBuffer();
            return (float[])jointAngles.Clone();
        }

        /// <summary>Clamps, stores, and applies joint angles, then refreshes the FK pose.</summary>
        public bool SetJointAngles(float[] anglesDegrees)
        {
            if (!IsReady || anglesDegrees == null || anglesDegrees.Length != JointCount)
                return false;

            for (int i = 0; i < JointCount; i++)
                jointAngles[i] = Mathf.Clamp(anglesDegrees[i], joints[i].minimumAngle, joints[i].maximumAngle);
            anglesInitialized = true;
            ApplyForwardKinematics();
            return true;
        }

        private void ApplyForwardKinematics()
        {
            if (!RefreshJointFrames()) return;
            EnsureJointAngleBuffer();
            if (!anglesInitialized) InitializeJointAngles();

            if (generatedRoot != null)
            {
                for (int i = 0; i < JointCount; i++)
                    jointFrames[i].localRotation = Quaternion.AngleAxis(
                        jointAngles[i] + joints[i].thetaOffset, Vector3.up);
                currentPose = toolFrame != null
                    ? new Pose(toolFrame.position, toolFrame.rotation)
                    : EvaluateForwardKinematics(jointAngles);
                return;
            }

            ComputeForwardFrames(jointAngles, jointOrigins, jointRotations, out currentPose);
            for (int i = 0; i < JointCount; i++)
            {
                jointFrames[i].position = jointOrigins[i];
                jointFrames[i].rotation = jointRotations[i];
            }
            if (toolFrame != null)
            {
                toolFrame.SetPositionAndRotation(currentPose.position, currentPose.rotation);
            }
        }

        private void ComputeForwardFrames(float[] anglesDegrees, Vector3[] origins,
            Quaternion[] rotations, out Pose tcpPose)
        {
            Vector3 position = BaseFrame.position;
            Quaternion rotation = BaseFrame.rotation;
            float scale = UnityUnitsPerMillimeter;
            for (int i = 0; i < JointCount; i++)
            {
                DHJoint row = joints[i];
                Quaternion jointRotation = rotation * Quaternion.AngleAxis(
                    anglesDegrees[i] + row.thetaOffset, Vector3.up);
                origins[i] = position;
                rotations[i] = jointRotation;
                position += jointRotation * new Vector3(row.a * scale, row.d * scale, 0f);
                rotation = jointRotation * Quaternion.AngleAxis(row.alpha, Vector3.right);
            }

            position += rotation * (toolOffsetMillimeters * scale);
            rotation *= Quaternion.Euler(toolRotationOffsetDegrees);
            tcpPose = new Pose(position, rotation);
        }

        private bool CacheGeneratedFrames()
        {
            var frames = new Transform[JointCount];
            Transform current = generatedRoot;
            for (int i = 0; i < JointCount; i++)
            {
                if (current.childCount == 0) return false;
                frames[i] = current.GetChild(0);
                if (frames[i].childCount == 0) return false;
                current = frames[i].GetChild(0);
            }
            jointFrames = frames;
            if (toolFrame == null && current.childCount > 0) toolFrame = current.GetChild(0);
            return true;
        }

        private bool TryFindManualJointFrames(out Transform[] frames)
        {
            frames = new Transform[JointCount];
            Transform searchRoot = BaseFrame;
            for (int i = 0; i < JointCount; i++)
            {
                string expectedName = joints[i] != null && !string.IsNullOrEmpty(joints[i].jointName)
                    ? joints[i].jointName
                    : $"J{i + 1}";
                frames[i] = FindDescendant(searchRoot, expectedName);
                if (frames[i] == null)
                {
                    frames = Array.Empty<Transform>();
                    return false;
                }
                searchRoot = frames[i];
            }
            return true;
        }

        private void InitializeJointAngles()
        {
            EnsureJointAngleBuffer();
            for (int i = 0; i < JointCount; i++)
            {
                NormalizeLimits(joints[i]);
                jointAngles[i] = Mathf.Clamp(joints[i].initialAngle,
                    joints[i].minimumAngle, joints[i].maximumAngle);
            }
            anglesInitialized = true;
        }

        private void EnsureJointAngleBuffer()
        {
            if (jointAngles == null || jointAngles.Length != JointCount)
                ResizeRuntimeBuffers();
            if (jointAngles == null || jointAngles.Length != JointCount)
                jointAngles = new float[JointCount];
        }

        private void ResizeRuntimeBuffers()
        {
            if (jointAngles == null || jointAngles.Length != JointCount)
                jointAngles = new float[JointCount];
            if (jointOrigins == null || jointOrigins.Length != JointCount)
                jointOrigins = new Vector3[JointCount];
            if (jointRotations == null || jointRotations.Length != JointCount)
                jointRotations = new Quaternion[JointCount];
        }

        private bool HasValidConfiguration() =>
            joints != null && JointCount >= MinimumJointCount && JointCount <= MaximumJointCount;

        private static void NormalizeLimits(DHJoint row)
        {
            if (row.minimumAngle > row.maximumAngle)
                (row.minimumAngle, row.maximumAngle) = (row.maximumAngle, row.minimumAngle);
        }

        private static bool ContainsNull(Transform[] values)
        {
            for (int i = 0; i < values.Length; i++)
                if (values[i] == null) return true;
            return false;
        }

        private void OnDrawGizmos()
        {
            if (!showGizmos || !HasValidConfiguration()) return;
            Transform[] frames = JointFrames;
            if (frames == null || frames.Length != JointCount) return;

            EnsureJointAngleBuffer();
            if (!anglesInitialized)
                for (int i = 0; i < JointCount; i++)
                    jointAngles[i] = joints[i] != null ? joints[i].initialAngle : 0f;
            ComputeForwardFrames(jointAngles, jointOrigins, jointRotations, out Pose gizmoPose);

            for (int i = 0; i < frames.Length; i++)
            {
                Transform joint = frames[i];
                if (joint == null) continue;
                Vector3 jointPosition = anglesInitialized ? joint.position : jointOrigins[i];
                Quaternion jointRotation = anglesInitialized ? joint.rotation : jointRotations[i];
                Gizmos.color = jointGizmoColor;
                Gizmos.DrawSphere(jointPosition, jointGizmoRadius);
                Gizmos.DrawLine(jointPosition, jointPosition + jointRotation * Vector3.up * (jointGizmoRadius * 3f));

                if (showGizmoLabels)
                    DrawLabel(jointPosition + Vector3.up * (jointGizmoRadius * 1.5f),
                        $"J{i + 1}  {FormatPosition(jointPosition)} mm");

                Vector3 end = i + 1 < frames.Length
                    ? (anglesInitialized ? frames[i + 1].position : jointOrigins[i + 1])
                    : (anglesInitialized ? currentPose.position : gizmoPose.position);
                Vector3 start = jointPosition;
                Gizmos.color = linkGizmoColor;
                Gizmos.DrawLine(start, end);
                if (showGizmoLabels)
                {
                    float length = Vector3.Distance(start, end) / UnityUnitsPerMillimeter;
                    DrawLabel((start + end) * 0.5f, $"L{i + 1}: {length:F1} mm");
                }
            }
        }

        private string FormatPosition(Vector3 position)
        {
            Vector3 localMm = BaseFrame.InverseTransformPoint(position) / UnityUnitsPerMillimeter;
            return $"({localMm.x:F1}, {localMm.y:F1}, {localMm.z:F1})";
        }

        private static Transform FindDescendant(Transform parent, string objectName)
        {
            for (int i = 0; i < parent.childCount; i++)
            {
                Transform child = parent.GetChild(i);
                if (child.name == objectName) return child;
                Transform nested = FindDescendant(child, objectName);
                if (nested != null) return nested;
            }
            return null;
        }

        private static void DrawLabel(Vector3 position, string text)
        {
#if UNITY_EDITOR
            UnityEditor.Handles.Label(position, text);
#endif
        }
    }
}
