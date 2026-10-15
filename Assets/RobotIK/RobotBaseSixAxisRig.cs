using System;
using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Joint binding for an imported six-axis robot hierarchy. Attach this to the
    /// Robot Base object; the imported pivot positions and mesh hierarchy are preserved.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RobotBaseSixAxisRig : MonoBehaviour
    {
        public const int JointCount = 6;

        [SerializeField] private Transform[] joints = new Transform[JointCount];
        [SerializeField] private Vector3[] localAxes =
        {
            Vector3.up, Vector3.up, Vector3.up,
            Vector3.up, Vector3.up, Vector3.up
        };
        [SerializeField] private float[] zeroOffsetsDegrees = new float[JointCount];
        [SerializeField] private Vector2[] angleLimitsDegrees =
        {
            new Vector2(-185f, 185f), new Vector2(-70f, 195f),
            new Vector2(-80f, 180f), new Vector2(-195f, 195f),
            new Vector2(-135f, 135f), new Vector2(-360f, 360f)
        };
        [SerializeField] private Transform toolFrame;
        [SerializeField] private bool restoreHh020ReferencePoseOnAwake = true;

        [NonSerialized] private Quaternion[] restLocalRotations = new Quaternion[JointCount];
        [NonSerialized] private float[] currentAnglesDegrees = new float[JointCount];
        [NonSerialized] private bool initialized;

        public Transform Base => transform;
        public Transform ToolFrame => toolFrame;
        public Transform[] Joints => joints;
        public Vector3[] LocalAxes => localAxes;
        public float[] ZeroOffsetsDegrees => zeroOffsetsDegrees;
        public Vector2[] AngleLimitsDegrees => angleLimitsDegrees;
        public bool RestoresHH020ReferencePose => restoreHh020ReferencePoseOnAwake;
        public bool IsReady => ValidateConfiguration(false);

        private void Reset()
        {
            FindJointTransforms();
            FindToolFrame();
            InitializeRestPose();
        }

        private void Awake()
        {
            if (joints == null || joints.Length != JointCount || ContainsNull(joints))
                FindJointTransforms();
            if (restoreHh020ReferencePoseOnAwake)
                RestoreHH020AnalyticalReferencePose();
            else if (!initialized)
                InitializeRestPose();
        }

        private void OnValidate()
        {
            EnsureArraySizes();
            if (joints != null && joints.Length == JointCount && ContainsNull(joints))
                FindJointTransforms();
        }

        /// <summary>Auto-fills J1..J6 by exact descendant name under Robot Base.</summary>
        [ContextMenu("Bind J1-J6 Under Robot Base")]
        public void FindJointTransforms()
        {
            EnsureArraySizes();
            for (int i = 0; i < JointCount; i++)
                joints[i] = FindDescendant(transform, $"J{i + 1}");
            FindToolFrame();
            InitializeRestPose();
        }

        /// <summary>
        /// Restores the imported HH020's known analytical reference pose. This avoids
        /// treating an Inspector-posed joint as a fixed mesh-frame rotation.
        /// </summary>
        [ContextMenu("Restore HH020 Analytical Reference Pose")]
        public void RestoreHH020AnalyticalReferencePose()
        {
            EnsureArraySizes();
            if (joints == null || joints.Length != JointCount || ContainsNull(joints))
            {
                for (int i = 0; i < JointCount; i++)
                    joints[i] = FindDescendant(transform, $"J{i + 1}");
            }
            if (ContainsNull(joints))
            {
                initialized = false;
                return;
            }

            localAxes[0] = Vector3.up;
            localAxes[1] = Vector3.forward;
            localAxes[2] = Vector3.forward;
            localAxes[3] = Vector3.right;
            localAxes[4] = Vector3.forward;
            localAxes[5] = Vector3.right;

            for (int i = 0; i < JointCount; i++)
            {
                zeroOffsetsDegrees[i] = 0f;
                joints[i].localRotation = i == 5
                    ? Quaternion.Euler(-90f, 0f, 0f)
                    : Quaternion.identity;
            }

            initialized = false;
            InitializeRestPose();
        }

        /// <summary>Returns J1 through J6 to the HH020 zero joint coordinates.</summary>
        [ContextMenu("Reset All Joints")]
        public void ResetAllJoints()
        {
            RestoreHH020AnalyticalReferencePose();
            SetJointAngles(new float[JointCount]);
        }

        /// <summary>Applies revolute coordinates in degrees without moving pivot positions.</summary>
        public bool SetJointAngles(float[] anglesDegrees)
        {
            if (!ValidateConfiguration(true) || anglesDegrees == null || anglesDegrees.Length != JointCount)
                return false;

            if (!initialized) InitializeRestPose();
            for (int i = 0; i < JointCount; i++)
            {
                Vector2 limits = angleLimitsDegrees[i];
                float value = Mathf.Clamp(anglesDegrees[i], limits.x, limits.y);
                currentAnglesDegrees[i] = value;
                joints[i].localRotation = restLocalRotations[i] *
                    Quaternion.AngleAxis(value + zeroOffsetsDegrees[i], localAxes[i].normalized);
            }
            return true;
        }

        public float[] GetJointAnglesDegrees()
        {
            if (!initialized) InitializeRestPose();
            return (float[])currentAnglesDegrees.Clone();
        }

        /// <summary>Gets a world-space rotation axis at the current joint configuration.</summary>
        public Vector3 GetWorldAxis(int jointIndex)
        {
            if (!ValidateConfiguration(true) || jointIndex < 0 || jointIndex >= JointCount)
                throw new ArgumentOutOfRangeException(nameof(jointIndex));
            return joints[jointIndex].TransformDirection(localAxes[jointIndex].normalized);
        }

        private void InitializeRestPose()
        {
            EnsureArraySizes();
            if (joints == null || joints.Length != JointCount || ContainsNull(joints))
            {
                initialized = false;
                return;
            }

            for (int i = 0; i < JointCount; i++)
            {
                restLocalRotations[i] = joints[i].localRotation *
                    Quaternion.Inverse(Quaternion.AngleAxis(zeroOffsetsDegrees[i], localAxes[i].normalized));
                currentAnglesDegrees[i] = 0f;
            }
            initialized = true;
        }

        private bool ValidateConfiguration(bool logError)
        {
            EnsureArraySizes();
            bool valid = joints != null && joints.Length == JointCount && !ContainsNull(joints);
            for (int i = 0; valid && i < JointCount; i++)
                valid = localAxes[i].sqrMagnitude > 0.000001f &&
                        angleLimitsDegrees[i].x <= angleLimitsDegrees[i].y;
            if (!valid && logError)
                Debug.LogError("Robot Base rig requires J1-J6 transforms, non-zero local joint axes, and valid angle limits.", this);
            return valid;
        }

        private void EnsureArraySizes()
        {
            if (joints == null || joints.Length != JointCount) Array.Resize(ref joints, JointCount);
            if (localAxes == null || localAxes.Length != JointCount)
            {
                Array.Resize(ref localAxes, JointCount);
                for (int i = 0; i < JointCount; i++)
                    if (localAxes[i].sqrMagnitude < 0.000001f) localAxes[i] = Vector3.up;
            }
            if (zeroOffsetsDegrees == null || zeroOffsetsDegrees.Length != JointCount)
                Array.Resize(ref zeroOffsetsDegrees, JointCount);
            if (angleLimitsDegrees == null || angleLimitsDegrees.Length != JointCount)
                Array.Resize(ref angleLimitsDegrees, JointCount);
            if (restLocalRotations == null || restLocalRotations.Length != JointCount)
                restLocalRotations = new Quaternion[JointCount];
            if (currentAnglesDegrees == null || currentAnglesDegrees.Length != JointCount)
                currentAnglesDegrees = new float[JointCount];
        }

        private void FindToolFrame()
        {
            if (toolFrame != null) return;
            toolFrame = FindDescendant(transform, "ToolFrame") ?? FindDescendant(transform, "flange");
        }

        private static Transform FindDescendant(Transform parent, string targetName)
        {
            for (int i = 0; i < parent.childCount; i++)
            {
                Transform child = parent.GetChild(i);
                if (child.name == targetName) return child;
                Transform nested = FindDescendant(child, targetName);
                if (nested != null) return nested;
            }
            return null;
        }

        private static bool ContainsNull(Transform[] values)
        {
            if (values == null) return true;
            for (int i = 0; i < values.Length; i++)
                if (values[i] == null) return true;
            return false;
        }
    }
}
