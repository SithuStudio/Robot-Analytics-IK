using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        [Header("Automatic fixed E1 station (Reset & plan, MoveL)")]
        [SerializeField] private bool automaticTrackStation;
        [SerializeField, Min(1)] private float stationSpacingMillimeters = 250f;
        [SerializeField, Range(3, 128)] private int maximumStationCandidates = 25;
        [SerializeField, Range(1, 128)] private int maximumStationRouteAttempts = 8;
        [SerializeField, Min(.1f)] private float stationSearchSeconds = 30f;
        private StationSearchSession stationSearch;
        private bool exactStationPlan;
        private string stationSearchSummary = string.Empty;
        private readonly List<string> stationSearchLog = new List<string>();
        public bool AutomaticTrackStation => automaticTrackStation;
        public bool IsSelectingTrackStation => stationSearch != null;
        public string StationSearchSummary => stationSearchSummary;
        public IReadOnlyList<string> StationSearchLog => stationSearchLog.AsReadOnly();
        public float StationSpacingMillimeters => stationSpacingMillimeters;
        public int MaximumStationCandidates => maximumStationCandidates;
        public int MaximumStationRouteAttempts => maximumStationRouteAttempts;
        public float StationSearchSeconds => stationSearchSeconds;
        private bool AutomaticStationApplicable => automaticTrackStation && plannerAlgorithm == PlannerAlgorithm.CartesianLinear && LinearTrack != null && LinearTrack.TrackEnabled;
        private bool RequireMarkerOrientations => useMarkerOrientations || avoidOverextension || stationSearch != null || coordinatedSearch != null || exactStationPlan;
        private bool AllowAdaptiveGoal => adaptiveGoal && !avoidOverextension && stationSearch == null && coordinatedSearch == null && !exactStationPlan;

        public bool SetAutomaticStationOptions(bool enabled, float spacing, int candidates, int routes, float seconds)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(spacing) || !LinearTrackAxis.Finite(seconds) || spacing < 1 || seconds < .1f) return false;
            automaticTrackStation = enabled; stationSpacingMillimeters = spacing;
            maximumStationCandidates = Mathf.Clamp(candidates, 3, 128); maximumStationRouteAttempts = Mathf.Clamp(routes, 1, 128); stationSearchSeconds = seconds;
            NotifySceneEdited(); stationSearchSummary = string.Empty; stationSearchLog.Clear(); return true;
        }

        private bool CheckStationSearchGuard()
        {
            var session = stationSearch;
            if (session == null) return true;
            string reason = session.Expired ? "Automatic station search reached its time budget." : !session.TargetsUnchanged() ? "TCP targets or station-search settings changed during station selection." : null;
            if (reason == null) return true;
            float elapsed = (float)session.ElapsedSeconds;
            StopMotion();
            planningTimeSeconds = elapsed;
            stationSearchSummary = reason + " Original setup restored; no partial motion will execute.";
            Status = stationSearchSummary; CurrentMotionState = MotionState.Failed;
            CompleteDiagnosticReport(session.Expired ? PlanDiagnosticCode.TimeLimit : PlanDiagnosticCode.ConfigurationChanged, Status);
            return false;
        }

        private void RollbackStationSearch()
        {
            var session = stationSearch;
            if (session == null) return;
            stationSearch = null; session.Dispose();
            exactStationPlan = false;
        }

        private bool TryPreflightStation(float e1, out List<float[]> starts, out string reason)
        {
            starts = null; reason = null;
            AssertCollisionContext();
            var before = rig.GetJointAnglesDegrees();
            using (new StateEvaluation(this, new ArmTrackState(before, e1)))
            {
                if (!inverseKinematics.SolveAll(startPose.position, startPose.rotation, out var startSolutions))
                { reason = "Start has no exact IK solution."; return false; }
                starts = FilterPoseSolutions(startSolutions, startPose.position, startPose.rotation, out var startReport);
                if (starts.Count == 0) { reason = "Start: " + DescribePoseCheck(startPose, startReport); return false; }
                rig.SetJointAngles(before); Physics.SyncTransforms();
                if (!inverseKinematics.SolveAll(endTarget.position, endTarget.rotation, out var endSolutions))
                { reason = "End has no exact IK solution."; return false; }
                var ends = FilterPoseSolutions(endSolutions, endTarget.position, endTarget.rotation, out var endReport);
                if (ends.Count == 0) { reason = "End: " + DescribePoseCheck(endTarget, endReport); return false; }
                return true;
            }
        }

        private bool ExactStationEndpointsMatch()
        {
            if (path.Count < 2 || !stationSearch.TargetsUnchanged()) return false;
            var restore = rig.GetJointAnglesDegrees();
            try
            {
                bool At(float[] q, Transform target)
                {
                    if (ValidateCombinedState(new ArmTrackState(q, LinearTrack.SelectedE1Millimeters)) != null || !rig.SetJointAngles(q)) return false;
                    Physics.SyncTransforms(); var tcp = GetCurrentTcp();
                    return Vector3.Distance(tcp.position, target.position) <= positionClosureTolerance &&
                        Quaternion.Angle(tcp.rotation, target.rotation) <= rotationClosureToleranceDegrees && !IsInCollision();
                }
                if (!At(path[0], startPose) || !At(path[path.Count - 1], endTarget)) return false;
                // Keep numeric wrist coordinates continuous; never disguise a branch
                // jump with modulo-360 angle arithmetic. MoveL already enforces this.
                for (int i = 1; i < path.Count; i++)
                    for (int j = 0; j < 6; j++) if (Mathf.Abs(path[i][j] - path[i - 1][j]) > 5.0001f) return false;
                return !adaptiveGoalUsed;
            }
            finally { rig.SetJointAngles(restore); Physics.SyncTransforms(); }
        }

        private IEnumerator SelectStationAndPlanRoutine()
        {
            exactStationPlan = false; stationSearchLog.Clear(); stationSearchSummary = string.Empty;
            path.Clear(); motionProfile = null; estimatedTravelTimeSeconds = 0;
            lastPlanSummary = string.Empty; planningIterations = 0; planningTimeSeconds = 0;
            planningTimer = System.Diagnostics.Stopwatch.StartNew();
            ClearGridGizmoData(); ClearPathVisualization(); InvalidateEndpointChecks();
            if (!ValidateSetup()) yield break;
            CacheColliders();
            if (robotColliders == null || robotColliders.Length == 0 || obstacleColliders.Length == 0)
            { Fail("Collision geometry is missing. Assign robot and obstacle colliders before station selection."); yield break; }
            var track = LinearTrack;
            if (inverseKinematics != null) inverseKinematics.SolveContinuously = false;
            if (!track.TryBeginStationSelection(this, out var selection, out string setupError)) { Fail(setupError); yield break; }
            var session = new StationSearchSession(this, selection); stationSearch = session;
            bool found = false;
            int inspected = 0, routes = 0;
            int totalIterations = 0;
            try
            {
                var candidates = new List<float>(LinearTrackStationCandidates.Build(track.Profile, track.CurrentE1Millimeters, stationSpacingMillimeters, maximumStationCandidates));
                if (avoidOverextension && !strictStretchLimit && stretchScope == StretchScope.WholeTrajectory)
                {
                    var scores = new Dictionary<float, float>();
                    foreach (float candidate in candidates) scores[candidate] = ReachStationScore(candidate);
                    candidates.Sort((a, b) => { int comparison = scores[a].CompareTo(scores[b]); return comparison != 0 ? comparison : Mathf.Abs(a - track.CurrentE1Millimeters).CompareTo(Mathf.Abs(b - track.CurrentE1Millimeters)); });
                }
                foreach (float e1 in candidates)
                {
                    if (!CheckStationSearchGuard()) yield break;
                    CurrentMotionState = MotionState.Planning;
                    Status = $"Automatic station: checking E1 {e1:F1} mm ({++inspected}/{candidates.Count}). Start and End stay exact.";
                    diagnosticPhase = "Automatic station preflight"; diagnosticAttempt = inspected;
                    bool feasible = TryPreflightStation(e1, out var starts, out string reason);
                    if (!feasible) stationSearchLog.Add($"E1 {e1:F1} mm: {reason}");
                    yield return null;
                    if (stationSearch != session || !CheckStationSearchGuard()) yield break;
                    if (!feasible) continue;
                    if (routes >= maximumStationRouteAttempts) { stationSearchLog.Add("Full route attempt limit reached."); break; }
                    if (!selection.TrySelect(e1, out var stamp, out setupError)) { Fail(setupError); yield break; }
                    plannedTrackState = stamp; hasTrackContext = true; CaptureCollisionContext();
                    CurrentMotionState = MotionState.Planning; routes++;
                    // Reuse full scanner-guided MoveL, all Start postures, collision,
                    // singularity, waypoint, blending and no-jump checks. No trial executes.
                    yield return StartCoroutine(PlanAndMoveRoutine(endpointsAlreadyChecked: false, resetStartCandidates: starts, suppressExecution: true));
                    totalIterations += planningIterations;
                    if (stationSearch != session || !CheckStationSearchGuard()) yield break;
                    if (CurrentMotionState != MotionState.Failed && ExactStationEndpointsMatch())
                    {
                        selection.Commit(); found = true; exactStationPlan = true;
                        stationSearchSummary = $"Automatic station selected E1 {e1:F1} mm after {inspected} station check(s), {routes} full route attempt(s), {session.ElapsedSeconds:F2} s. Start and End exact; no wrist-branch jumps.";
                        stationSearchLog.Add(stationSearchSummary);
                        lastPlanSummary += " " + stationSearchSummary; Status = lastPlanSummary;
                        planningTimeSeconds = (float)session.ElapsedSeconds;
                        break;
                    }
                    stationSearchLog.Add($"E1 {e1:F1} mm: full MoveL route rejected. {Status}");
                    path.Clear(); ClearPathVisualization();
                    CurrentMotionState = MotionState.Planning;
                    yield return null;
                }
            }
            finally
            {
                if (stationSearch == session) stationSearch = null;
                session.Dispose();
                if (!found)
                {
                    exactStationPlan = false; path.Clear(); motionProfile = null; estimatedTravelTimeSeconds = 0;
                    ClearPathVisualization(); InvalidateEndpointChecks();
                    plannedTrackState = session.OriginalTrackState;
                }
            }
            if (!found)
            {
                planningTimeSeconds = (float)session.ElapsedSeconds; planningIterations = totalIterations;
                stationSearchSummary = $"No complete exact MoveL route at the tested stations ({inspected} checked, {routes} route attempts). Original E1 and arm pose restored. Search is bounded; untested stations may exist. No partial motion will execute.";
                Fail(stationSearchSummary, routes >= maximumStationRouteAttempts ? PlanDiagnosticCode.AttemptLimit : PlanDiagnosticCode.NoCompleteRoute); yield break;
            }
            CurrentMotionState = MotionState.Planning;
            planningIterations = totalIterations;
            rig.SetJointAngles(path[0]); Physics.SyncTransforms(); yield return null;
            yield return StartCoroutine(ExecutePath());
        }

        private sealed class StationSearchSession : IDisposable
        {
            private RobotBaseSixAxisMotionPlanner planner;
            private readonly RobotLinearTrackRig.StationSelectionScope selection;
            private readonly float[] originalAngles;
            internal readonly RobotLinearTrackRig.StateStamp OriginalTrackState;
            private readonly Transform[] targets;
            private readonly Pose[] poses;
            private readonly System.Diagnostics.Stopwatch clock = System.Diagnostics.Stopwatch.StartNew();
            private readonly float seconds, spacing;
            private readonly int candidates, routes;
            internal double ElapsedSeconds => clock.Elapsed.TotalSeconds;
            internal bool Expired => ElapsedSeconds >= seconds;
            internal StationSearchSession(RobotBaseSixAxisMotionPlanner p, RobotLinearTrackRig.StationSelectionScope selection)
            {
                planner = p; this.selection = selection; originalAngles = p.rig.GetJointAnglesDegrees(); OriginalTrackState = p.plannedTrackState;
                var all = new List<Transform> { p.startPose, p.endTarget };
                foreach (var point in p.intermediateWaypoints ?? Array.Empty<Transform>()) if (point != null) all.Add(point);
                targets = all.ToArray(); poses = new Pose[targets.Length];
                for (int i = 0; i < targets.Length; i++) poses[i] = new Pose(targets[i].position, targets[i].rotation);
                seconds = p.stationSearchSeconds; spacing = p.stationSpacingMillimeters; candidates = p.maximumStationCandidates; routes = p.maximumStationRouteAttempts;
            }
            internal bool TargetsUnchanged()
            {
                if (planner == null || planner.plannerAlgorithm != PlannerAlgorithm.CartesianLinear || !planner.automaticTrackStation ||
                    planner.startPose != targets[0] || planner.endTarget != targets[1] || !seconds.Equals(planner.stationSearchSeconds) ||
                    !spacing.Equals(planner.stationSpacingMillimeters) || candidates != planner.maximumStationCandidates || routes != planner.maximumStationRouteAttempts) return false;
                var all = new List<Transform> { planner.startPose, planner.endTarget };
                foreach (var point in planner.intermediateWaypoints ?? Array.Empty<Transform>()) if (point != null) all.Add(point);
                if (all.Count != targets.Length) return false;
                for (int i = 0; i < targets.Length; i++)
                    if (targets[i] == null || all[i] != targets[i] || !poses[i].position.Equals(targets[i].position) || Quaternion.Angle(poses[i].rotation, targets[i].rotation) > .001f) return false;
                return true;
            }
            public void Dispose()
            {
                if (planner == null) return;
                selection.Dispose();
                planner.rig.SetJointAngles(originalAngles); Physics.SyncTransforms(); planner = null;
            }
        }
    }
}
