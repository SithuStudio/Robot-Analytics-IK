using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        private long collisionContextRevision;
        private CollisionContext collisionContext;
        private StateEvaluation activeStateEvaluation;
        public ArmTrackState LastStoppedState { get; private set; }
        public ArmTrackState CurrentCombinedState => CaptureCombinedState();

        /// <summary>Step 2 adapter: each stored six-axis waypoint carries the same locked E1.</summary>
        public IReadOnlyList<ArmTrackState> PlannedCombinedPath
        {
            get
            {
                if (coordinatedPath.Count > 0) return Array.AsReadOnly(coordinatedPath.ToArray());
                var states = new ArmTrackState[path.Count];
                float? e1 = plannedTrackState != null ? plannedTrackState.E1Millimeters : (float?)null;
                for (int i = 0; i < path.Count; i++) states[i] = new ArmTrackState(path[i], e1);
                return Array.AsReadOnly(states);
            }
        }

        private ArmTrackState CaptureCombinedState()
        {
            if (rig == null) rig = GetComponent<RobotBaseSixAxisRig>();
            if (rig == null || !rig.IsReady) return null;
            var track = LinearTrack;
            return new ArmTrackState(rig.GetJointAnglesDegrees(), track != null && track.TrackEnabled ? track.CurrentE1Millimeters : (float?)null);
        }

        private ArmTrackState AtPlanningStation(float[] angles) => new ArmTrackState(angles,
            plannedTrackState != null ? plannedTrackState.E1Millimeters :
            LinearTrack != null && LinearTrack.TrackEnabled ? LinearTrack.SelectedE1Millimeters : (float?)null);

        /// <summary>Collision-only inspection of a complete configuration. Restores the current physical pose on every exit.
        /// This is not a swept motion check or automatic station selection.</summary>
        public ArmTrackCollisionCheck CheckCombinedState(ArmTrackState state)
        {
            if (IsBusy || IsRrtSearchInProgress) return InvalidCombinedState("Stop planning or motion before inspecting a combined state.");
            if (rig == null) rig = GetComponent<RobotBaseSixAxisRig>();
            if (rig == null || !rig.IsReady) return InvalidCombinedState("Six-axis rig is not ready.");
            SubscribeTrack();
            var track = LinearTrack;
            if (track != null && track.TrackEnabled)
            {
                if (track.transform != transform) return InvalidCombinedState("Attach the track adapter to this Robot Base.");
                if (!track.ValidateConfiguration(out string setupError) || !track.IsAppliedPoseCurrent(out setupError)) return InvalidCombinedState(setupError);
                if (!ValidateTrackTargets(track)) return InvalidCombinedState(Status);
            }
            CacheColliders();
            if (robotColliders == null || robotColliders.Length == 0 || obstacleColliders.Length == 0)
                return InvalidCombinedState("Collision geometry is missing.");
            if (collisionContext == null || !collisionContext.Matches(this))
            {
                if (path.Count > 0) NotifySceneEdited();
                CaptureCollisionContext();
            }
            return EvaluateCombinedState(state);
        }

        private static ArmTrackCollisionCheck InvalidCombinedState(string reason) => new ArmTrackCollisionCheck(false, false, default, reason);

        private string ValidateCombinedState(ArmTrackState state)
        {
            if (state == null) return "Combined state is missing.";
            var track = LinearTrack;
            if (state.HasTrackAxis != (track != null && track.TrackEnabled)) return "Combined state track mode does not match the current rig.";
            if (state.HasTrackAxis && (state.E1Millimeters < track.MinimumMillimeters || state.E1Millimeters > track.MaximumMillimeters))
                return "Combined E1 is outside the calibrated track range.";
            for (int i = 0; i < ArmTrackState.ArmJointCount; i++)
            {
                var limit = rig.AngleLimitsDegrees[i];
                if (!LinearTrackAxis.Finite(limit.x) || !LinearTrackAxis.Finite(limit.y) || limit.x > limit.y) return $"J{i + 1} has invalid configured limits.";
                if (state[i] < limit.x || state[i] > limit.y) return $"J{i + 1} is outside its configured joint limits.";
            }
            return null;
        }

        private ArmTrackCollisionCheck EvaluateCombinedState(ArmTrackState state)
        {
            string error = ValidateCombinedState(state);
            if (error != null) { CaptureInvalidState(state, error); return InvalidCombinedState(error); }
            AssertCollisionContext();
            var key = new ArmTrackStateKey(state, collisionContextRevision);
            if (LastPlanReport?.Outcome == PlanReportOutcome.Running) LastPlanReport.ConfigurationQueries++;
            if (rrtCollisionCache.TryGetValue(key, out var cached))
            { if (LastPlanReport?.Outcome == PlanReportOutcome.Running) LastPlanReport.CacheHits++; return cached; }
            using (var evaluation = new StateEvaluation(this, state))
            {
                Transform tcp = GetCurrentTcp();
                bool collision = IsInCollision(out string pair);
                if (!evaluation.IsActive) throw new OperationCanceledException("Combined-state evaluation was stopped.");
                var check = new ArmTrackCollisionCheck(true, collision, new Pose(tcp.position, tcp.rotation), pair);
                rrtCollisionCache[key] = check;
                return check;
            }
        }

        private void InvalidateCollisionContext()
        {
            unchecked { collisionContextRevision++; }
            collisionContext = null;
            rrtCollisionCache.Clear(); rrtTcpTravelCache.Clear();
        }
        private void CaptureCollisionContext()
        {
            InvalidateCollisionContext();
            collisionContext = new CollisionContext(this);
        }
        private void AssertCollisionContext()
        {
            if (collisionContext == null) { CacheColliders(); CaptureCollisionContext(); }
            if (!TrackContextMatches(out string error) || !collisionContext.Matches(this))
            {
                error ??= "Collision geometry, clearance or joint limits changed. Reset & plan again.";
                if (IsBusy) AbortTrackChange(error);
                throw new OperationCanceledException(error);
            }
        }

        // Main-thread transactions never yield. Stop disposes the active transaction
        // before capturing the held pose, so a late finally cannot reinstate a query pose.
        private sealed class StateEvaluation : IDisposable
        {
            private RobotBaseSixAxisMotionPlanner planner;
            private readonly float[] originalAngles;
            private RobotLinearTrackRig.EvaluationScope trackScope;
            internal bool IsActive => planner != null;
            internal StateEvaluation(RobotBaseSixAxisMotionPlanner planner, ArmTrackState state)
            {
                if (planner.activeStateEvaluation != null) throw new InvalidOperationException("Nested combined-state queries are not supported.");
                this.planner = planner; originalAngles = planner.rig.GetJointAnglesDegrees();
                planner.activeStateEvaluation = this;
                try
                {
                    string error = planner.ValidateCombinedState(state);
                    if (error != null) throw new ArgumentException(error);
                    if (state.HasTrackAxis)
                    {
                        var track = planner.LinearTrack;
                        bool alreadyApplied = state.E1Millimeters.Equals(track.SelectedE1Millimeters) && track.IsAppliedPoseCurrent(out _);
                        if (!alreadyApplied && !track.TryBeginEvaluation(planner, state.E1Millimeters, out trackScope, out error))
                            throw new InvalidOperationException(error);
                    }
                    if (!planner.rig.SetJointAngles(state.JointAnglesDegrees)) throw new InvalidOperationException("Could not evaluate arm state.");
                    Physics.SyncTransforms();
                }
                catch { Dispose(); throw; }
            }
            public void Dispose()
            {
                if (planner == null) return;
                var owner = planner; planner = null;
                trackScope?.Dispose();
                if (owner.rig != null && owner.rig.IsReady) owner.rig.SetJointAngles(originalAngles);
                if (ReferenceEquals(owner.activeStateEvaluation, this)) owner.activeStateEvaluation = null;
                Physics.SyncTransforms();
            }
        }

        // Cache identity includes a generation captured from collision-relevant scene
        // data. Joint rotations are excluded because FK queries intentionally change them.
        private sealed class CollisionContext
        {
            private readonly RobotBaseSixAxisRig rig;
            private readonly Matrix4x4 baseWorld;
            private readonly Transform obstaclesRoot;
            private readonly Collider[] rootBodies;
            private readonly List<Collider> rootScratch = new List<Collider>();
            private readonly Collider[] armBodies, extras;
            private readonly float clearance, resolution, boxScale;
            private readonly bool selfCollision;
            private readonly Vector2[] limits;
            private readonly Vector3[] axes, origins, scales;
            private readonly float[] offsets;
            private readonly Transform[] joints, parents;
            private readonly List<ColliderStamp> colliders = new List<ColliderStamp>();
            internal CollisionContext(RobotBaseSixAxisMotionPlanner p)
            {
                rig = p.rig; baseWorld = rig != null ? rig.transform.localToWorldMatrix : Matrix4x4.identity;
                obstaclesRoot = p.obstaclesRoot; rootBodies = obstaclesRoot != null ? obstaclesRoot.GetComponentsInChildren<Collider>(true) : Array.Empty<Collider>();
                armBodies = (Collider[])(p.robotColliders ?? Array.Empty<Collider>()).Clone();
                extras = (Collider[])(p.additionalObstacleColliders ?? Array.Empty<Collider>()).Clone();
                clearance = p.minimumObstacleClearance; resolution = p.collisionResolutionDegrees; boxScale = p.robotBoxColliderScale; selfCollision = p.checkSelfCollision;
                limits = rig != null ? (Vector2[])rig.AngleLimitsDegrees.Clone() : Array.Empty<Vector2>();
                axes = rig != null ? (Vector3[])rig.LocalAxes.Clone() : Array.Empty<Vector3>();
                offsets = rig != null ? (float[])rig.ZeroOffsetsDegrees.Clone() : Array.Empty<float>();
                joints = rig != null ? (Transform[])rig.Joints.Clone() : Array.Empty<Transform>();
                parents = new Transform[joints.Length]; origins = new Vector3[joints.Length]; scales = new Vector3[joints.Length];
                for (int i = 0; i < joints.Length; i++) if (joints[i] != null)
                { parents[i] = joints[i].parent; origins[i] = joints[i].localPosition; scales[i] = joints[i].localScale; }
                var seen = new HashSet<Collider>();
                foreach (var group in new[] { p.collisionRobotBodies ?? armBodies, p.obstacleColliders ?? Array.Empty<Collider>() })
                    foreach (var c in group)
                    {
                        if (c == null || !seen.Add(c)) continue;
                        Transform frame = null;
                        if (rig != null && c.transform.IsChildOf(rig.transform))
                        { int index = p.GetOwningJointIndex(c.transform); frame = index >= 0 ? joints[index] : rig.transform; }
                        else if (p.LinearTrack != null && p.LinearTrack.IsCarriageCollider(c)) frame = p.LinearTrack.Carriage;
                        colliders.Add(new ColliderStamp(c, frame));
                    }
            }
            public bool Matches(RobotBaseSixAxisMotionPlanner p)
            {
                if (p.rig != rig || !SameMatrix(baseWorld, rig != null ? rig.transform.localToWorldMatrix : Matrix4x4.identity) ||
                    p.obstaclesRoot != obstaclesRoot || !Same(armBodies, p.robotColliders) || !Same(extras, p.additionalObstacleColliders) ||
                    !clearance.Equals(p.minimumObstacleClearance) || !resolution.Equals(p.collisionResolutionDegrees) || !boxScale.Equals(p.robotBoxColliderScale) || selfCollision != p.checkSelfCollision) return false;
                if (obstaclesRoot != null)
                {
                    obstaclesRoot.GetComponentsInChildren(true, rootScratch);
                    if (rootScratch.Count != rootBodies.Length) return false;
                    for (int i = 0; i < rootBodies.Length; i++) if (rootBodies[i] != rootScratch[i]) return false;
                }
                for (int i = 0; i < joints.Length; i++)
                    if (joints[i] == null || rig.Joints[i] != joints[i] || joints[i].parent != parents[i] || !origins[i].Equals(joints[i].localPosition) ||
                        !scales[i].Equals(joints[i].localScale) || !limits[i].Equals(rig.AngleLimitsDegrees[i]) || !axes[i].Equals(rig.LocalAxes[i]) || !offsets[i].Equals(rig.ZeroOffsetsDegrees[i])) return false;
                foreach (var c in colliders) if (!c.Matches()) return false;
                return true;
            }
            private static bool Same(Collider[] a, Collider[] b)
            {
                if (a.Length != (b?.Length ?? 0)) return false;
                for (int i = 0; i < a.Length; i++) if (a[i] != b[i]) return false;
                return true;
            }
            internal static bool SameMatrix(Matrix4x4 a, Matrix4x4 b)
            { for (int i = 0; i < 16; i++) if (!LinearTrackAxis.Finite(b[i]) || Mathf.Abs(a[i] - b[i]) > .00001f) return false; return true; }
        }

        private sealed class ColliderStamp
        {
            private readonly Collider collider;
            private readonly Transform frame;
            private readonly Transform parent;
            private readonly Matrix4x4 matrix;
            private readonly bool enabled, trigger, active;
            private readonly Vector3 center, size;
            private readonly float radius, height;
            private readonly int direction;
            private readonly Mesh mesh;
            private readonly Bounds meshBounds;
            private readonly bool convex;
            private readonly MeshColliderCookingOptions cooking;
            internal ColliderStamp(Collider c, Transform frame)
            {
                collider = c; this.frame = frame; parent = c.transform.parent; matrix = Matrix(); enabled = c.enabled; trigger = c.isTrigger; active = c.gameObject.activeInHierarchy;
                if (c is BoxCollider box) { center = box.center; size = box.size; }
                if (c is SphereCollider sphere) { center = sphere.center; radius = sphere.radius; }
                if (c is CapsuleCollider capsule) { center = capsule.center; radius = capsule.radius; height = capsule.height; direction = capsule.direction; }
                if (c is MeshCollider m) { mesh = m.sharedMesh; convex = m.convex; cooking = m.cookingOptions; meshBounds = mesh != null ? mesh.bounds : default; }
            }
            private Matrix4x4 Matrix() => frame != null ? frame.worldToLocalMatrix * collider.transform.localToWorldMatrix : collider.transform.localToWorldMatrix;
            internal bool Matches()
            {
                if (collider == null || collider.transform.parent != parent || collider.enabled != enabled || collider.isTrigger != trigger || collider.gameObject.activeInHierarchy != active || !CollisionContext.SameMatrix(matrix, Matrix())) return false;
                if (collider is BoxCollider box) return center.Equals(box.center) && size.Equals(box.size);
                if (collider is SphereCollider sphere) return center.Equals(sphere.center) && radius.Equals(sphere.radius);
                if (collider is CapsuleCollider capsule) return center.Equals(capsule.center) && radius.Equals(capsule.radius) && height.Equals(capsule.height) && direction == capsule.direction;
                if (collider is MeshCollider m) return mesh == m.sharedMesh && convex == m.convex && cooking == m.cookingOptions && (mesh == null || meshBounds.Equals(mesh.bounds));
                return true;
            }
        }
    }
}
