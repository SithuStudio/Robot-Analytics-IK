using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        [Header("Coordinated MoveL (Reset & plan)")]
        [SerializeField] private bool coordinatedMoveL;
        [SerializeField, Min(1)] private float coordinatedCartesianStepMillimeters = 20;
        [SerializeField, Min(1)] private float coordinatedTrackStepMillimeters = 40;
        [SerializeField, Range(2, 64)] private int coordinatedBeamWidth = 12;
        [SerializeField, Min(.1f)] private float coordinatedSearchSeconds = 30;
        [SerializeField, Min(1)] private float coordinatedTrackSpeedMillimetersPerSecond = 250;
        private CoordinatedSearchSession coordinatedSearch;
        private CoordinatedTargets coordinatedTargets;
        private readonly List<ArmTrackState> coordinatedPath = new List<ArmTrackState>();
        private readonly List<Pose> coordinatedPoses = new List<Pose>();
        private CoordinatedMoveLTimeline coordinatedTimeline;
        private readonly Dictionary<ArmTrackStateKey, CoordinatedStateEvaluation> coordinatedChecks = new Dictionary<ArmTrackStateKey, CoordinatedStateEvaluation>();
        private string coordinatedSummary = string.Empty;
        public bool CoordinatedMoveLEnabled => coordinatedMoveL;
        public bool IsPlanningCoordinatedMoveL => coordinatedSearch != null;
        public string CoordinatedMoveLSummary => coordinatedSummary;
        public float CoordinatedCartesianStepMillimeters => coordinatedCartesianStepMillimeters;
        public float CoordinatedTrackStepMillimeters => coordinatedTrackStepMillimeters;
        public int CoordinatedBeamWidth => coordinatedBeamWidth;
        public float CoordinatedSearchSeconds => coordinatedSearchSeconds;
        public float CoordinatedTrackSpeedMillimetersPerSecond => coordinatedTrackSpeedMillimetersPerSecond;
        private bool CoordinatedApplicable => coordinatedMoveL && plannerAlgorithm == PlannerAlgorithm.CartesianLinear && LinearTrack != null && LinearTrack.TrackEnabled;

        public bool SetCoordinatedMoveLOptions(bool enabled, float cartesianStep, float trackStep, int beamWidth, float seconds, float trackSpeed)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(cartesianStep) || !LinearTrackAxis.Finite(trackStep) ||
                !LinearTrackAxis.Finite(seconds) || !LinearTrackAxis.Finite(trackSpeed) || cartesianStep < 1 || trackStep < 1 || seconds < .1f || trackSpeed < 1) return false;
            coordinatedMoveL = enabled; coordinatedCartesianStepMillimeters = cartesianStep; coordinatedTrackStepMillimeters = trackStep;
            coordinatedBeamWidth = Mathf.Clamp(beamWidth, 2, 64); coordinatedSearchSeconds = seconds; coordinatedTrackSpeedMillimetersPerSecond = trackSpeed;
            NotifySceneEdited(); return true;
        }
        private void ClearCoordinatedPlan()
        {
            ClearStagedPath();
            coordinatedPath.Clear(); coordinatedPoses.Clear(); coordinatedTimeline = null; coordinatedTargets = null;
            coordinatedChecks.Clear(); coordinatedSummary = string.Empty;
        }
        private void CancelCoordinatedSearch()
        {
            if (coordinatedSearch == null) return;
            planningTimeSeconds = (float)coordinatedSearch.Clock.Elapsed.TotalSeconds;
            planningIterations = coordinatedSearch.Result?.Checks ?? planningIterations;
            coordinatedSearch.Cancelled = true; coordinatedSearch = null;
        }
        private bool CheckCoordinatedGuard()
        {
            if (coordinatedSearch == null && coordinatedPath.Count == 0) return true;
            bool expired = coordinatedSearch != null && coordinatedSearch.Clock.Elapsed.TotalSeconds >= (coordinatedSearch.DeadlineSeconds ?? coordinatedSearchSeconds);
            if (!expired && coordinatedTargets != null && coordinatedTargets.Matches(this)) return true;
            string workflow = coordinatedTargets != null && coordinatedTargets.IsStaged ? "Track transfer" : "Coordinated MoveL";
            string reason = expired ? workflow + " search time budget reached." : workflow + " targets or search settings changed.";
            float elapsed = coordinatedSearch != null ? (float)coordinatedSearch.Clock.Elapsed.TotalSeconds : planningTimeSeconds;
            int checks = coordinatedSearch?.Result?.Checks ?? planningIterations;
            var startState = StartIkState; var endState = EndIkState; string startText = StartIkSummary, endText = EndIkSummary, reachability = trackReachabilitySummary;
            StopMotion(); NotifySceneEdited();
            planningTimeSeconds = elapsed; planningIterations = checks;
            if (expired) { StartIkState = startState; EndIkState = endState; StartIkSummary = startText; EndIkSummary = endText; trackReachabilitySummary = reachability; }
            coordinatedSummary = reason + $" {checks} checks completed. No partial trajectory will execute.";
            Status = coordinatedSummary; CurrentMotionState = MotionState.Failed;
            if (LastPlanReport != null) LastPlanReport.Stale = !expired;
            CompleteDiagnosticReport(expired ? PlanDiagnosticCode.TimeLimit : PlanDiagnosticCode.ConfigurationChanged, Status); return false;
        }
        private CoordinatedStateEvaluation EvaluateCoordinatedState(ArmTrackState state, CartesianLinearPlanner singularity)
        {
            string error = ValidateCombinedState(state);
            if (error != null) { CaptureInvalidState(state, error); return new CoordinatedStateEvaluation { Failure = error }; }
            AssertCollisionContext();
            var key = new ArmTrackStateKey(state, collisionContextRevision);
            if (LastPlanReport?.Outcome == PlanReportOutcome.Running) LastPlanReport.ConfigurationQueries++;
            if (coordinatedSearch != null && coordinatedChecks.TryGetValue(key, out var cached))
            { if (LastPlanReport?.Outcome == PlanReportOutcome.Running) LastPlanReport.CacheHits++; return cached; }
            using (var scope = new StateEvaluation(this, state))
            {
                var tcp = GetCurrentTcp(); var pose = new Pose(tcp.position, tcp.rotation);
                bool collision = IsInCollision(out string pair);
                string failure = collision ? pair : singularity.GetSingularityRejection(state.JointAnglesDegrees);
                if (!collision && failure != null) CaptureDiagnostic(PlanDiagnosticCode.Singularity, failure, state.JointAnglesDegrees, pose,
                    (float)singularity.LastSingularityRatio, (float)singularity.SingularityThreshold, "Jacobian min/max singular-value ratio");
                if (!scope.IsActive) throw new OperationCanceledException("Coordinated evaluation was stopped.");
                var check = new CoordinatedStateEvaluation { Valid = failure == null, Tcp = pose, Failure = failure };
                if (coordinatedSearch != null) coordinatedChecks[key] = check;
                return check;
            }
        }
        private IReadOnlyList<ArmTrackState> SolveCoordinatedCandidates(Pose pose, float e1)
        {
            AssertCollisionContext();
            using (new StateEvaluation(this, new ArmTrackState(rig.GetJointAnglesDegrees(), e1)))
            {
                if (!inverseKinematics.SolveAll(pose.position, pose.rotation, out var solved)) return Array.Empty<ArmTrackState>();
                var states = new List<ArmTrackState>(solved.Count);
                foreach (var q in solved) states.Add(new ArmTrackState(q, e1));
                return states;
            }
        }
        private IEnumerator PlanCoordinatedMoveLRoutine()
        {
            ClearCoordinatedPlan(); exactStationPlan = false; path.Clear(); motionProfile = null;
            ClearGridGizmoData(); ClearPathVisualization(); InvalidateEndpointChecks();
            planningIterations = 0; planningTimeSeconds = 0; estimatedTravelTimeSeconds = 0;
            if (!ValidateSetup()) yield break;
            CacheColliders();
            if (robotColliders == null || robotColliders.Length == 0 || obstacleColliders.Length == 0)
            { Fail("Collision geometry is required for coordinated MoveL."); yield break; }
            inverseKinematics.SolveContinuously = false;
            InitializeGoalSelection(endTarget.rotation);
            CancelGridSearch(); gridSearchCancellation = new System.Threading.CancellationTokenSource();
            var token = gridSearchCancellation.Token;
            coordinatedTargets = new CoordinatedTargets(this);
            var session = new CoordinatedSearchSession(); coordinatedSearch = session;
            var result = new CoordinatedMoveLPlanner.Result();
            session.Result = result; Status = "Searching continuous coordinated MoveL candidates. Start and End remain exact.";
            var singularity = new CartesianLinearPlanner(CreateAlgorithmWorld());
            var track = LinearTrack;
            var world = new CoordinatedMoveLPlanner.World
            {
                Solve = SolveCoordinatedCandidates, Evaluate = state => EvaluateCoordinatedState(state, singularity),
                Rejected = CaptureSearchRejection,
                LimitReached = CaptureSearchLimit,
                Penalty = avoidOverextension ? state => ReachPenalty(state, false) : (Func<ArmTrackState, float>)null,
                ShouldStop = () => session.Cancelled || token.IsCancellationRequested || session.Clock.Elapsed.TotalSeconds >= coordinatedSearchSeconds,
                Progress = text => { Status = text; planningTimeSeconds = (float)session.Clock.Elapsed.TotalSeconds; planningIterations = result.Checks; },
                StartStations = LinearTrackStationCandidates.Build(track.Profile, track.CurrentE1Millimeters, Mathf.Max(1, (track.MaximumMillimeters - track.MinimumMillimeters) / 8), 9),
                PreferredE1 = track.CurrentE1Millimeters, MinimumE1 = track.MinimumMillimeters, MaximumE1 = track.MaximumMillimeters,
                CartesianStepMillimeters = coordinatedCartesianStepMillimeters, TrackStepMillimeters = coordinatedTrackStepMillimeters,
                BeamWidth = coordinatedBeamWidth, PositionToleranceMeters = Mathf.Min(.001f, positionClosureTolerance),
                RotationToleranceDegrees = Mathf.Min(.25f, rotationClosureToleranceDegrees), JointSweepStepDegrees = Mathf.Min(.25f, collisionResolutionDegrees)
            };
            try { yield return StartCoroutine(new CoordinatedMoveLPlanner(world).Search(coordinatedTargets.Poses, result)); }
            finally { if (coordinatedSearch == session) coordinatedSearch = null; coordinatedChecks.Clear(); }
            planningTimeSeconds = (float)session.Clock.Elapsed.TotalSeconds; planningIterations = result.Checks;
            if (session.Cancelled || token.IsCancellationRequested) yield break;
            if (!result.Found || session.Clock.Elapsed.TotalSeconds >= coordinatedSearchSeconds || !coordinatedTargets.Matches(this))
            {
                string failure = session.Clock.Elapsed.TotalSeconds >= coordinatedSearchSeconds ? "Search time budget reached." : result.Failure ?? "Exact target snapshot changed.";
                ClearCoordinatedPlan();
                coordinatedSummary = "Coordinated MoveL failed: " + failure + " Original setup retained; no partial path will execute.";
                Fail(coordinatedSummary, session.Clock.Elapsed.TotalSeconds >= coordinatedSearchSeconds ? PlanDiagnosticCode.TimeLimit : result.StopCode); yield break;
            }
            coordinatedPath.AddRange(result.States); coordinatedPoses.AddRange(result.Poses);
            foreach (var state in coordinatedPath) path.Add(state.JointAnglesDegrees);
            exactStationPlan = true; adaptiveGoalUsed = false;
            requestedGoalPosition = plannedGoalPosition = endTarget.position; requestedGoalRotation = plannedGoalRotation = endTarget.rotation;
            coordinatedSummary = $"Coordinated MoveL: {coordinatedPath.Count} validated arm/E1 samples, {result.Checks} checks, {planningTimeSeconds:F2} s. Exact Start and End; no wrist-branch jumps.";
            lastPlanSummary = coordinatedSummary; Status = coordinatedSummary;
            DrawCoordinatedPath();
            yield return StartCoroutine(ExecuteCoordinatedPath());
        }
        private bool CoordinatedPoseValid(ArmTrackState state, Pose expected, CartesianLinearPlanner singularity, out string reason, bool enforceCartesian = true, bool transferReach = false)
        {
            var check = EvaluateCoordinatedState(state, singularity); reason = check.Failure;
            if (!check.Valid) return false;
            if (ReachRejects(state.JointAnglesDegrees, transferReach, out reason)) return false;
            ObserveReach(state.JointAnglesDegrees, transferReach);
            if (enforceCartesian && (Vector3.Distance(check.Tcp.position, expected.position) > Mathf.Min(.001f, positionClosureTolerance) ||
                Quaternion.Angle(check.Tcp.rotation, expected.rotation) > Mathf.Min(.25f, rotationClosureToleranceDegrees)))
            { reason = "Runtime combined interpolation departed from the validated Cartesian segment."; return false; }
            return true;
        }
        private bool ApplyCoordinatedState(ArmTrackState state, out string error)
        {
            error = ValidateCombinedState(state); if (error != null) return false;
            AssertCollisionContext();
            if (!LinearTrack.TryApplyOwnedPosition(this, state.E1Millimeters, out var stamp, out error)) return false;
            if (!rig.SetJointAngles(state.JointAnglesDegrees)) { error = "Could not apply coordinated arm coordinates."; return false; }
            Physics.SyncTransforms(); plannedTrackState = stamp; hasTrackContext = true;
            CaptureCollisionContext(); return true;
        }
        private void RejectCoordinatedMotion(string reason)
        {
            StopMotion(); NotifySceneEdited();
            Status = "Coordinated MoveL stopped: " + reason; CurrentMotionState = MotionState.Failed;
        }
        private IEnumerator ExecuteCoordinatedPath()
        {
            UpdateStretchFeedback();
            if (!CheckCoordinatedGuard() || coordinatedPath.Count < 2) yield break;
            if (!TrackContextMatches(out string contextError) || collisionContext == null || !collisionContext.Matches(this))
            { RejectCoordinatedMotion(contextError ?? "Collision context changed."); yield break; }
            inverseKinematics.SolveContinuously = false; BuildCoordinatedTimeline();
            var timeline = coordinatedTimeline;
            var singularity = new CartesianLinearPlanner(CreateAlgorithmWorld());
            if (!CoordinatedPoseValid(coordinatedPath[0], coordinatedPoses[0], singularity, out string reason, transferReach: stagedPath) || !ApplyCoordinatedState(coordinatedPath[0], out reason))
            { RejectCoordinatedMotion(reason); yield break; }
            CurrentMotionState = MotionState.Moving; Status = stagedPath ? "Executing validated retract and track transfer." : "Executing coordinated MoveL (arm + E1).";
            yield return null;
            float elapsed = 0;
            while (elapsed < timeline.DurationSeconds)
            {
                if (!CheckCoordinatedGuard()) yield break;
                elapsed = Mathf.Min(timeline.DurationSeconds, elapsed + Time.deltaTime);
                var state = timeline.Evaluate(elapsed, out Pose expected, out int index);
                bool enforceCartesian = !stagedPath || stagedCartesianEdges[index];
                if (!CoordinatedPoseValid(state, expected, singularity, out reason, enforceCartesian, stagedPath && stagedPhases[index] != TrackTransferPhase.FinalMoveL) || !ApplyCoordinatedState(state, out reason))
                { RejectCoordinatedMotion(reason); yield break; }
                if (stagedPath)
                {
                    currentTransferPhase = stagedPhases[index].ToString();
                    Status = stagedPhases[index] == TrackTransferPhase.FinalMoveL
                        ? $"E1 locked at {state.E1Millimeters:F1} mm. Executing final MoveL to exact End."
                        : $"{currentTransferPhase}: E1 {state.E1Millimeters:F1} mm. Following validated continuous arm posture.";
                }
                yield return null;
            }
            Status = "Motion complete. " + coordinatedSummary; CurrentMotionState = MotionState.Complete;
        }
        private void BuildCoordinatedTimeline()
        {
            coordinatedTimeline = new CoordinatedMoveLTimeline(coordinatedPath, coordinatedPoses, Mathf.Max(1, Linear) * .001f,
                Mathf.Max(1, RotationDegree), Mathf.Max(1, coordinatedTrackSpeedMillimetersPerSecond), stagedPath ? transferTrackAcceleration : 0);
            estimatedTravelTimeSeconds = coordinatedTimeline.DurationSeconds;
        }
        private void DrawCoordinatedPath()
        {
            if (coordinatedPath.Count < 2) return;
            BuildCoordinatedTimeline(); motionProfile = null; SmoothedPathCorners = 0;
            RetainedSharpCorners = Mathf.Max(0, (coordinatedTargets?.Poses.Count ?? 2) - 2);
            var positions = new List<Vector3>(); var rotations = new List<Quaternion>();
            foreach (var pose in coordinatedPoses) { positions.Add(pose.position); rotations.Add(pose.rotation); }
            gizmoPath = positions.ToArray();
            var markers = new List<Pose>();
            if (stagedPath) foreach (int index in transferWaypointIndices) markers.Add(coordinatedPoses[index]);
            else markers.AddRange(coordinatedTargets.Poses);
            gizmoWaypoints = new Vector3[markers.Count]; gizmoWaypointRotations = new Quaternion[markers.Count];
            for (int i = 0; i < markers.Count; i++) { gizmoWaypoints[i] = markers[i].position; gizmoWaypointRotations[i] = markers[i].rotation; }
            if (pathRenderer == null)
            {
                var line = new GameObject("Planned TCP Path"); line.transform.SetParent(transform, false); pathRenderer = line.AddComponent<LineRenderer>();
            }
            ConfigurePathRenderer(); pathRenderer.positionCount = positions.Count; pathRenderer.SetPositions(gizmoPath);
            UpdateWaypointLabels(new List<Vector3>(gizmoWaypoints));
        }
        private sealed class CoordinatedSearchSession
        { public bool Cancelled; public double? DeadlineSeconds; public CoordinatedMoveLPlanner.Result Result; public readonly System.Diagnostics.Stopwatch Clock = System.Diagnostics.Stopwatch.StartNew(); }
        private sealed class CoordinatedTargets
        {
            public readonly bool IsStaged;
            private readonly Transform[] targets;
            public readonly List<Pose> Poses = new List<Pose>();
            private readonly float cartesianStep, trackStep, budget, trackSpeed;
            private readonly int beam;
            private readonly float stationSpacing, approach, trackAcceleration, reductionPosition, reductionRotation, optimizationBudget;
            private readonly int stations, iterations, targetWaypoints;
            private readonly bool preferCorridor, requireCorridor;
            private readonly float corridorDrop, corridorBlend;
            internal CoordinatedTargets(RobotBaseSixAxisMotionPlanner p)
            {
                IsStaged = p.stagedTrackTransfer; stationSpacing = p.transferStationSpacingMillimeters; approach = p.transferApproachMillimeters;
                stations = p.transferMaximumStations; iterations = p.transferPostureIterations;
                trackAcceleration = p.transferTrackAcceleration;
                targetWaypoints = p.transferTargetWaypoints;
                preferCorridor = p.preferLowerTransferCorridor; requireCorridor = p.requireLowerTransferCorridor; corridorDrop = p.transferCorridorDropMm; corridorBlend = p.pathAccuracyMillimeters;
                reductionPosition = p.transferReductionPositionMm; reductionRotation = p.transferReductionRotationDegrees; optimizationBudget = p.transferOptimizationBudgetSeconds;
                var all = new List<Transform> { p.startPose };
                foreach (var point in p.intermediateWaypoints ?? Array.Empty<Transform>()) if (point != null) all.Add(point);
                all.Add(p.endTarget); targets = all.ToArray();
                foreach (var t in targets) Poses.Add(new Pose(t.position, t.rotation));
                cartesianStep = p.coordinatedCartesianStepMillimeters; trackStep = p.coordinatedTrackStepMillimeters; beam = p.coordinatedBeamWidth; budget = p.coordinatedSearchSeconds;
                trackSpeed = p.coordinatedTrackSpeedMillimetersPerSecond;
            }
            internal bool Matches(RobotBaseSixAxisMotionPlanner p)
            {
                if (p.plannerAlgorithm != PlannerAlgorithm.CartesianLinear || IsStaged != p.stagedTrackTransfer || (IsStaged ? !p.stagedTrackTransfer : !p.coordinatedMoveL) ||
                    (IsStaged && (preferCorridor != p.preferLowerTransferCorridor || requireCorridor != p.requireLowerTransferCorridor || !corridorDrop.Equals(p.transferCorridorDropMm) || !corridorBlend.Equals(p.pathAccuracyMillimeters))) ||
                    (IsStaged && (!reductionPosition.Equals(p.transferReductionPositionMm) || !reductionRotation.Equals(p.transferReductionRotationDegrees) || !optimizationBudget.Equals(p.transferOptimizationBudgetSeconds))) ||
                    (IsStaged && (!stationSpacing.Equals(p.transferStationSpacingMillimeters) || !approach.Equals(p.transferApproachMillimeters) || !trackAcceleration.Equals(p.transferTrackAcceleration) || targetWaypoints != p.transferTargetWaypoints || stations != p.transferMaximumStations || iterations != p.transferPostureIterations)) ||
                    !cartesianStep.Equals(p.coordinatedCartesianStepMillimeters) || !trackStep.Equals(p.coordinatedTrackStepMillimeters) || beam != p.coordinatedBeamWidth ||
                    !budget.Equals(p.coordinatedSearchSeconds) || !trackSpeed.Equals(p.coordinatedTrackSpeedMillimetersPerSecond)) return false;
                var all = new List<Transform> { p.startPose };
                foreach (var t in p.intermediateWaypoints ?? Array.Empty<Transform>()) if (t != null) all.Add(t);
                all.Add(p.endTarget); if (all.Count != targets.Length) return false;
                for (int i = 0; i < targets.Length; i++)
                    if (targets[i] == null || targets[i] != all[i] || !Poses[i].position.Equals(targets[i].position) || Quaternion.Angle(Poses[i].rotation, targets[i].rotation) > .001f) return false;
                return true;
            }
        }
    }
}
