using System;
using UnityEngine;

namespace RobotIK
{
    [Serializable]
    public struct TrackAllowedContact
    {
        public Collider First, Second;
    }

    /// <summary>Setup/preview adapter for a calibrated, locked E1. Attach to Robot Base.</summary>
    [ExecuteAlways, DisallowMultipleComponent, RequireComponent(typeof(RobotBaseSixAxisRig))]
    [AddComponentMenu("Robot IK/Linear Track Rig (Locked E1)")]
    public sealed class RobotLinearTrackRig : MonoBehaviour
    {
        [SerializeField] private bool trackEnabled;
        [SerializeField] private Transform trackRoot;
        [SerializeField] private Transform carriage;
        [SerializeField] private TrackPoseProfile profile = new TrackPoseProfile();
        [SerializeField] private float selectedE1Millimeters;
        [Tooltip("Carriage-only collision geometry. Do not include moving arm joints here.")]
        [SerializeField] private Collider[] carriageColliders = Array.Empty<Collider>();
        [SerializeField] private Collider[] stationaryRailColliders = Array.Empty<Collider>();
        [Tooltip("Only explicitly listed mechanical contact pairs are excluded from track collision checks.")]
        [SerializeField] private TrackAllowedContact[] allowedContacts = Array.Empty<TrackAllowedContact>();
        private object lockOwner;
        private StateStamp appliedState;
        private EvaluationScope evaluation;
        private StationSelectionScope stationSelection;
        private bool stopHeld;
        public event Action Changed;
        public bool TrackEnabled => trackEnabled;
        public bool IsLocked => lockOwner != null;
        public bool IsStopHeld => stopHeld;
        public float SelectedE1Millimeters => selectedE1Millimeters;
        public float CurrentE1Millimeters => evaluation != null ? evaluation.E1Millimeters : appliedState?.E1Millimeters ?? selectedE1Millimeters;
        public float MinimumMillimeters => profile?.Home.E1Millimeters ?? 0;
        public float MaximumMillimeters => profile?.End.E1Millimeters ?? 0;
        public TrackPoseProfile Profile => profile?.Copy();
        public Transform TrackRoot => trackRoot;
        public Transform Carriage => carriage;
        public string LastError { get; private set; }
        public int Revision { get; private set; }
        public Collider[] CarriageColliders => (Collider[])(carriageColliders ?? Array.Empty<Collider>()).Clone();
        public Collider[] StationaryRailColliders => (Collider[])(stationaryRailColliders ?? Array.Empty<Collider>()).Clone();

        public bool TryConfigure(Transform root, Transform movingCarriage, TrackPoseProfile data, out string error)
        {
            if (!Editable(out error)) return false;
            Transform oldRoot = trackRoot, oldCarriage = carriage;
            TrackPoseProfile oldProfile = profile; float oldE1 = selectedE1Millimeters;
            trackRoot = root; carriage = movingCarriage; profile = data?.Copy(); selectedE1Millimeters = profile?.Home.E1Millimeters ?? 0;
            if (!ValidateConfiguration(out error))
            { trackRoot = oldRoot; carriage = oldCarriage; profile = oldProfile; selectedE1Millimeters = oldE1; return false; }
            if (trackEnabled) return TryApplySelected(out error);
            AnnounceChange(); return true;
        }

        public bool TrySetEnabled(bool enabled, out string error)
        {
            if (!Editable(out error)) return false;
            bool old = trackEnabled; trackEnabled = enabled;
            if (enabled && !TryApplySelected(out error)) { trackEnabled = old; return false; }
            if (!enabled) { stopHeld = false; if (old) { appliedState = null; AnnounceChange(); } }
            return true;
        }

        public bool TrySetPosition(float millimeters, out string error)
        {
            if (!Editable(out error)) return false;
            if (!trackEnabled) { error = "Enable the track before selecting E1."; return false; }
            float old = selectedE1Millimeters; selectedE1Millimeters = millimeters;
            if (TryApplySelected(out error)) return true;
            selectedE1Millimeters = old; return false;
        }

        public bool TrySetProfile(TrackPoseProfile data, out string error)
        {
            if (!Editable(out error) || !LinearTrackAxis.ValidateProfile(data, out error)) return false;
            TrackPoseProfile old = profile; profile = data.Copy();
            if (trackEnabled && !TryApplySelected(out error)) { profile = old; return false; }
            if (!trackEnabled) AnnounceChange();
            return true;
        }

        public bool TryApplySelected(out string error)
        {
            if (!Editable(out error)) return false;
            if (!trackEnabled) return true;
            if (!ValidateConfiguration(out error)) { LastError = error; return false; }
            var evaluator = new LinearTrackAxis(profile);
            if (!evaluator.TryEvaluate(selectedE1Millimeters, out Pose pose, out error)) { LastError = error; return false; }
            bool changed = appliedState == null || !appliedState.Matches(this, out _);
            carriage.SetLocalPositionAndRotation(pose.position, pose.rotation);
            Physics.SyncTransforms();
            appliedState = new StateStamp(this);
            stopHeld = false;
            LastError = null;
            if (changed) AnnounceChange();
            return true;
        }

        public bool ValidateConfiguration(out string error)
        {
            error = null;
            if (!LinearTrackAxis.ValidateProfile(profile, out error)) return false;
            if (trackRoot == null || carriage == null || carriage.parent != trackRoot || !transform.IsChildOf(carriage) || transform == carriage)
            { error = "Assign TrackRoot and its direct child Carriage. Robot Base must be below Carriage through a fixed mount; no automatic reparenting is performed."; return false; }
            if (!isActiveAndEnabled || !trackRoot.gameObject.activeInHierarchy || !carriage.gameObject.activeInHierarchy)
            { error = "Track component, TrackRoot and Carriage must be active."; return false; }
            if (!UnitScale(trackRoot.lossyScale) || !UnitScale(carriage.localScale) || !UnitScale(transform.lossyScale))
            { error = "TrackRoot, Carriage and robot mounting hierarchy must use unit scale (1,1,1)."; return false; }
            for (Transform mount = transform; mount != carriage; mount = mount.parent)
                if (!UnitScale(mount.localScale))
                { error = "Every fixed mounting transform must use unit scale (1,1,1)."; return false; }
            if (!LinearTrackAxis.Finite(trackRoot.position) || !LinearTrackAxis.Finite(transform.position))
            { error = "Track reference and mounting transforms must be finite."; return false; }
            if (!new LinearTrackAxis(profile).TryEvaluate(selectedE1Millimeters, out _, out error)) return false;
            foreach (var c in carriageColliders ?? Array.Empty<Collider>())
                if (c != null && (!c.transform.IsChildOf(carriage) || c.transform == transform || c.transform.IsChildOf(transform)))
                { error = "Carriage colliders must belong to Carriage, outside the six-axis robot hierarchy."; return false; }
            foreach (var c in stationaryRailColliders ?? Array.Empty<Collider>())
                if (c != null && (c.transform == carriage || c.transform.IsChildOf(carriage)))
                { error = "Stationary rail colliders cannot belong to the moving carriage."; return false; }
            return true;
        }

        public bool TryAcquire(object owner, out StateStamp stamp, out string error)
        {
            stamp = null;
            if (owner == null) { error = "A track lock owner is required."; return false; }
            if (!trackEnabled) { error = "Track is disabled."; return false; }
            if (IsLocked) { error = "E1 is already locked by a motion operation."; return false; }
            if (!TryApplySelected(out error)) return false;
            lockOwner = owner; stamp = new StateStamp(this); return true;
        }
        public void Release(object owner) { if (ReferenceEquals(lockOwner, owner)) lockOwner = null; }

        // Only the operation owning the track lock can drive a coordinated trajectory.
        // This updates the expected pose without emitting a user scene-edit event.
        internal bool TryApplyOwnedPosition(object owner, float e1, out StateStamp stamp, out string error)
        {
            stamp = null; error = null;
            if (!IsLocked || !ReferenceEquals(owner, lockOwner) || evaluation != null || stationSelection != null)
            { error = "Coordinated track motion requires the owning planner's exclusive lock."; return false; }
            if (!trackEnabled || !ValidateConfiguration(out error)) { error ??= "Track is disabled."; return false; }
            if (!new LinearTrackAxis(profile).TryEvaluate(e1, out Pose pose, out error)) return false;
            carriage.SetLocalPositionAndRotation(pose.position, pose.rotation); selectedE1Millimeters = e1;
            Physics.SyncTransforms(); stopHeld = false; appliedState = stamp = new StateStamp(this);
            return true;
        }

        // A collision query may evaluate another E1 without selecting it, firing scene-edit
        // events, changing revision, or releasing the planner's operation lock.
        internal bool TryBeginEvaluation(object owner, float e1, out EvaluationScope scope, out string error)
        {
            scope = null; error = null;
            if (evaluation != null || (IsLocked && !ReferenceEquals(owner, lockOwner)))
            { error = "Track evaluation is owned by another operation."; return false; }
            if (!trackEnabled || !ValidateConfiguration(out error)) { error ??= "Track is disabled."; return false; }
            if (!new LinearTrackAxis(profile).TryEvaluate(e1, out Pose pose, out error)) return false;
            scope = new EvaluationScope(this, e1); evaluation = scope;
            carriage.SetLocalPositionAndRotation(pose.position, pose.rotation);
            return true;
        }

        internal void HoldForStop()
        {
            evaluation?.Dispose();
            if (!trackEnabled) return;
            // Inspector writes to the selection do not become a new physical pose on Stop.
            if (appliedState != null) selectedE1Millimeters = appliedState.E1Millimeters;
            stopHeld = true;
        }

        internal bool IsAppliedPoseCurrent(out string error)
        {
            error = "Apply the selected E1 setup pose before checking combined states.";
            return appliedState != null && appliedState.Matches(this, out error);
        }

        internal bool TryBeginStationSelection(object owner, out StationSelectionScope scope, out string error)
        {
            scope = null; error = null;
            if (!ReferenceEquals(owner, lockOwner) || !IsLocked || evaluation != null || stationSelection != null)
            { error = "Station selection requires the owning planner's exclusive track lock."; return false; }
            if (!ValidateConfiguration(out error)) return false;
            scope = new StationSelectionScope(this, owner); stationSelection = scope; return true;
        }

        internal sealed class StationSelectionScope : IDisposable
        {
            private RobotLinearTrackRig rig;
            private readonly object owner;
            private readonly Transform carriage;
            private readonly Vector3 position;
            private readonly Quaternion rotation;
            private readonly float selected;
            private readonly StateStamp stamp;
            private readonly bool held;
            private bool committed;
            internal StationSelectionScope(RobotLinearTrackRig rig, object owner)
            {
                this.rig = rig; this.owner = owner; carriage = rig.carriage; position = carriage.localPosition; rotation = carriage.localRotation;
                selected = rig.selectedE1Millimeters; stamp = rig.appliedState; held = rig.stopHeld;
            }
            internal bool TrySelect(float e1, out StateStamp current, out string error)
            {
                current = null; error = null;
                if (rig == null || !ReferenceEquals(owner, rig.lockOwner) || rig.evaluation != null)
                { error = "Station search was stopped or lost its track lock."; return false; }
                if (!rig.ValidateConfiguration(out error) || !new LinearTrackAxis(rig.profile).TryEvaluate(e1, out Pose pose, out error)) return false;
                rig.selectedE1Millimeters = e1; carriage.SetLocalPositionAndRotation(pose.position, pose.rotation);
                Physics.SyncTransforms(); rig.stopHeld = false;
                rig.appliedState = current = new StateStamp(rig);
                return true;
            }
            internal void Commit()
            {
                if (rig == null) throw new InvalidOperationException("Station search is no longer active.");
                committed = true;
                if (!selected.Equals(rig.selectedE1Millimeters)) rig.Revision++;
            }
            public void Dispose()
            {
                if (rig == null) return;
                if (!committed)
                {
                    if (carriage != null) carriage.SetLocalPositionAndRotation(position, rotation);
                    rig.selectedE1Millimeters = selected; rig.appliedState = stamp; rig.stopHeld = held;
                    Physics.SyncTransforms();
                }
                rig.stationSelection = null; rig = null;
            }
        }

        internal sealed class EvaluationScope : IDisposable
        {
            private RobotLinearTrackRig rig;
            private readonly Transform carriage;
            private readonly Vector3 position;
            private readonly Quaternion rotation;
            internal readonly float E1Millimeters;
            internal EvaluationScope(RobotLinearTrackRig rig, float e1)
            { this.rig = rig; E1Millimeters = e1; carriage = rig.carriage; position = carriage.localPosition; rotation = carriage.localRotation; }
            public void Dispose()
            {
                if (rig == null) return;
                if (carriage != null) carriage.SetLocalPositionAndRotation(position, rotation);
                rig.evaluation = null; rig = null;
            }
        }
        public bool IsCarriageCollider(Collider collider) => Array.IndexOf(carriageColliders ?? Array.Empty<Collider>(), collider) >= 0;
        public bool AllowsContact(Collider a, Collider b)
        {
            foreach (var pair in allowedContacts ?? Array.Empty<TrackAllowedContact>())
                if ((pair.First == a && pair.Second == b) || (pair.First == b && pair.Second == a)) return true;
            return false;
        }
        private bool Editable(out string error)
        {
            error = IsLocked || evaluation != null || (GetComponent<RobotBaseSixAxisMotionPlanner>()?.IsBusy ?? false)
                ? "E1 and calibration are locked during planning/playback. Stop first." : null;
            return error == null;
        }
        private void AnnounceChange() { Revision++; Changed?.Invoke(); }
        private void Update()
        {
            if (IsLocked || evaluation != null || stopHeld) return;
            if (!trackEnabled)
            {
                if (appliedState != null) { appliedState = null; AnnounceChange(); }
                return;
            }
            if (appliedState == null || !appliedState.Matches(this, out _)) TryApplySelected(out _);
        }
        [ContextMenu("Apply selected E1 (setup preview)")]
        private void ApplyFromInspector() { if (!TryApplySelected(out string error)) Debug.LogWarning(error, this); }
        private static bool UnitScale(Vector3 v) => (v - Vector3.one).sqrMagnitude < 1e-8f;

        /// <summary>Full plan stamp, not a hash: detects profile, reference, mount, E1 and collision-reference changes.</summary>
        public sealed class StateStamp
        {
            public readonly RobotLinearTrackRig Rig;
            public readonly float E1Millimeters;
            private readonly TrackPoseProfile profile;
            private readonly Transform root, carriage;
            private readonly Matrix4x4 rootWorld, baseWorld;
            private readonly Vector3 localPosition, localScale;
            private readonly Quaternion localRotation;
            private readonly Collider[] bodies, rails;
            private readonly TrackAllowedContact[] contacts;
            internal StateStamp(RobotLinearTrackRig rig)
            {
                Rig = rig; E1Millimeters = rig.selectedE1Millimeters; profile = rig.profile.Copy(); root = rig.trackRoot; carriage = rig.carriage;
                rootWorld = root.localToWorldMatrix; baseWorld = rig.transform.localToWorldMatrix;
                localPosition = carriage.localPosition; localRotation = carriage.localRotation; localScale = carriage.localScale;
                bodies = rig.CarriageColliders; rails = rig.StationaryRailColliders;
                contacts = (TrackAllowedContact[])(rig.allowedContacts ?? Array.Empty<TrackAllowedContact>()).Clone();
            }
            public bool Matches(RobotLinearTrackRig rig, out string error)
            {
                error = "Track E1, calibration, reference, mount or collision configuration changed. Reset & plan again.";
                if (rig == null || rig != Rig || !rig.trackEnabled || !rig.isActiveAndEnabled || root == null || carriage == null ||
                    rig.trackRoot != root || rig.carriage != carriage || !rig.selectedE1Millimeters.Equals(E1Millimeters) || !profile.SameAs(rig.profile) ||
                    !root.gameObject.activeInHierarchy || !carriage.gameObject.activeInHierarchy ||
                    !SameMatrix(rootWorld, root.localToWorldMatrix) || !SameMatrix(baseWorld, rig.transform.localToWorldMatrix) ||
                    !localPosition.Equals(carriage.localPosition) || !SameRotation(localRotation, carriage.localRotation) || !localScale.Equals(carriage.localScale) ||
                    !SameArray(bodies, rig.carriageColliders) || !SameArray(rails, rig.stationaryRailColliders)) return false;
                var current = rig.allowedContacts ?? Array.Empty<TrackAllowedContact>();
                if (current.Length != contacts.Length) return false;
                for (int i = 0; i < current.Length; i++) if (current[i].First != contacts[i].First || current[i].Second != contacts[i].Second) return false;
                error = null; return true;
            }
            private static bool SameArray(Collider[] a, Collider[] b)
            {
                if (a.Length != (b?.Length ?? 0)) return false;
                for (int i = 0; i < a.Length; i++) if (a[i] != b[i]) return false;
                return true;
            }
            private static bool SameMatrix(Matrix4x4 a, Matrix4x4 b)
            { for (int i = 0; i < 16; i++) if (!LinearTrackAxis.Finite(b[i]) || Mathf.Abs(a[i] - b[i]) > 0.00001f) return false; return true; }
            private static bool SameRotation(Quaternion a, Quaternion b)
            {
                // Transform writes can normalize a quaternion by a few ULPs. Compare
                // components (including q/-q equivalence), rather than exact bits or
                // Quaternion.Angle's coarser float-dot resolution near zero degrees.
                float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z, dw = a.w - b.w;
                float sx = a.x + b.x, sy = a.y + b.y, sz = a.z + b.z, sw = a.w + b.w;
                return Mathf.Min(dx * dx + dy * dy + dz * dz + dw * dw, sx * sx + sy * sy + sz * sz + sw * sw) <= 1e-12f;
            }
        }
    }
}
