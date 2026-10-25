using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        public enum TrackTransferPhase { Start, Retract, TrackTravel, Arrival, FinalMoveL }
        [Header("Staged track transfer then final MoveL")]
        [SerializeField] private bool stagedTrackTransfer;
        [SerializeField, Min(1)] private float transferStationSpacingMillimeters = 250;
        [SerializeField, Range(3, 65)] private int transferMaximumStations = 17;
        [SerializeField, Min(10)] private float transferApproachMillimeters = 100;
        [SerializeField, Range(100, 10000)] private int transferPostureIterations = 2000;
        [SerializeField, Min(1)] private float transferTrackAcceleration = 1000;
        [SerializeField, Range(2, 10000)] private int transferTargetWaypoints = 128;
        [SerializeField, Min(.01f)] private float transferReductionPositionMm = 1;
        [SerializeField, Min(.001f)] private float transferReductionRotationDegrees = .1f;
        [SerializeField, Min(.1f)] private float transferOptimizationBudgetSeconds = 2;
        [SerializeField] private bool preferLowerTransferCorridor = true;
        [SerializeField] private bool requireLowerTransferCorridor;
        [SerializeField, Min(10)] private float transferCorridorDropMm = 200;
        public bool PreferLowerTransferCorridor => preferLowerTransferCorridor;
        public bool RequireLowerTransferCorridor => requireLowerTransferCorridor;
        public float TransferCorridorDropMm => transferCorridorDropMm;
        public bool UsedLowerTransferCorridor { get; private set; }
        public bool SetLowerTransferCorridorOptions(bool prefer, bool require, float dropMm)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(dropMm) || dropMm < 10 || dropMm > 2000) return false;
            preferLowerTransferCorridor = prefer || require; requireLowerTransferCorridor = require; transferCorridorDropMm = dropMm; NotifySceneEdited(); return true;
        }
        public float TransferReductionPositionMm => transferReductionPositionMm;
        public float TransferReductionRotationDegrees => transferReductionRotationDegrees;
        public float TransferOptimizationBudgetSeconds => transferOptimizationBudgetSeconds;
        public int TransferWaypointsBeforeOptimization { get; private set; }
        public float TransferOptimizationTimeSeconds { get; private set; }
        public string TransferOptimizationSummary { get; private set; } = string.Empty;
        public bool SetTransferOptimizationOptions(float positionMm, float rotationDegrees, float seconds)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(positionMm) || positionMm < .01f ||
                !LinearTrackAxis.Finite(rotationDegrees) || rotationDegrees < .001f || !LinearTrackAxis.Finite(seconds) || seconds < .1f || seconds > 30) return false;
            transferReductionPositionMm = positionMm; transferReductionRotationDegrees = rotationDegrees; transferOptimizationBudgetSeconds = seconds;
            NotifySceneEdited(); return true;
        }
        private int[] transferWaypointIndices = Array.Empty<int>();
        public int TransferTargetWaypoints => transferTargetWaypoints;
        public int PlannedWaypointCount => stagedPath ? transferWaypointIndices.Length : path.Count;
        public IReadOnlyList<ArmTrackState> PlannedTransferWaypoints
        {
            get { var points = new List<ArmTrackState>(); foreach (int i in transferWaypointIndices) points.Add(coordinatedPath[i]); return points.AsReadOnly(); }
        }
        public bool SetTransferTargetWaypoints(int count)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || count < 2 || count > 10000) return false;
            transferTargetWaypoints = count; NotifySceneEdited(); return true;
        }
        private readonly List<bool> stagedCartesianEdges = new List<bool>();
        private readonly List<TrackTransferPhase> stagedPhases = new List<TrackTransferPhase>();
        private readonly List<string> stagedTransferLog = new List<string>();
        private bool stagedPath;
        private string currentTransferPhase = string.Empty;
        public bool StagedTrackTransferEnabled => stagedTrackTransfer;
        public string CurrentTrackTransferPhase => currentTransferPhase;
        public IReadOnlyList<TrackTransferPhase> PlannedTransferPhases => stagedPhases.AsReadOnly();
        public IReadOnlyList<string> TrackTransferLog => stagedTransferLog.AsReadOnly();
        public float TransferStationSpacingMillimeters => transferStationSpacingMillimeters;
        public int TransferMaximumStations => transferMaximumStations;
        public float TransferApproachMillimeters => transferApproachMillimeters;
        public int TransferPostureIterations => transferPostureIterations;
        public float TransferTrackAcceleration => transferTrackAcceleration;
        public bool SetTransferTrackAcceleration(float millimetersPerSecondSquared)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(millimetersPerSecondSquared) || millimetersPerSecondSquared < 1) return false;
            transferTrackAcceleration = millimetersPerSecondSquared; NotifySceneEdited(); return true;
        }
        private bool StagedTransferApplicable => stagedTrackTransfer && plannerAlgorithm == PlannerAlgorithm.CartesianLinear && LinearTrack != null && LinearTrack.TrackEnabled;
        public bool SetStagedTransferOptions(bool enabled, float stationSpacing, int stations, float approach, int iterations, float seconds, float trackSpeed)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(stationSpacing) || stationSpacing < 1 ||
                !LinearTrackAxis.Finite(approach) || approach < 10 || !LinearTrackAxis.Finite(seconds) || seconds < .1f || !LinearTrackAxis.Finite(trackSpeed) || trackSpeed < 1) return false;
            stagedTrackTransfer = enabled; transferStationSpacingMillimeters = stationSpacing; transferMaximumStations = Mathf.Clamp(stations, 3, 65);
            transferApproachMillimeters = approach; transferPostureIterations = Mathf.Clamp(iterations, 100, 10000);
            coordinatedSearchSeconds = seconds; coordinatedTrackSpeedMillimetersPerSecond = trackSpeed;
            NotifySceneEdited(); return true;
        }
        private void ClearStagedPath()
        { stagedPath = false; transferWaypointIndices = Array.Empty<int>(); stagedCartesianEdges.Clear(); stagedPhases.Clear(); currentTransferPhase = string.Empty;
            UsedLowerTransferCorridor = false;
            TransferWaypointsBeforeOptimization = 0; TransferOptimizationTimeSeconds = 0; TransferOptimizationSummary = string.Empty; }
        private TrackTransferPhase TransferPhase(ArmTrackState state, ArmTrackState first, float destination)
            => Mathf.Abs(state.E1Millimeters - destination) < .001f ? TrackTransferPhase.Arrival : Mathf.Abs(state.E1Millimeters - first.E1Millimeters) < .001f ? TrackTransferPhase.Retract : TrackTransferPhase.TrackTravel;
        private void PreserveAuthoredWaypoints(List<Pose> poses, List<int> phaseIds)
        {
            int marker = 1000;
            foreach (var target in intermediateWaypoints ?? Array.Empty<Transform>())
            {
                if (target == null) continue;
                for (int i = 0; i < poses.Count; i++)
                    if (Vector3.Distance(poses[i].position, target.position) < .000001f && Quaternion.Angle(poses[i].rotation, target.rotation) < .001f)
                    { phaseIds[i] = marker++; break; }
            }
        }
        private void TransferLog(string text)
        { if (stagedTransferLog.Count < 80) stagedTransferLog.Add(text); }
        private List<ArmTrackState> ValidTransferIk(Pose pose, float e1, Func<ArmTrackState, CoordinatedStateEvaluation> evaluate, out int rawCount)
        {
            var candidates = SolveCoordinatedCandidates(pose, e1); rawCount = candidates.Count;
            var valid = new List<ArmTrackState>();
            foreach (var state in candidates)
            {
                var check = evaluate(state);
                if (check.Valid && Vector3.Distance(check.Tcp.position, pose.position) <= Mathf.Min(.001f, positionClosureTolerance) &&
                    Quaternion.Angle(check.Tcp.rotation, pose.rotation) <= Mathf.Min(.25f, rotationClosureToleranceDegrees)) valid.Add(state);
            }
            var reference = rig.GetJointAnglesDegrees();
            valid.Sort((a, b) => JointDifference(a, reference).CompareTo(JointDifference(b, reference)));
            return valid;
        }
        private static double JointDifference(ArmTrackState state, float[] q)
        { double sum = 0; for (int i = 0; i < 6; i++) sum += Math.Abs(state[i] - q[i]) * (i >= 3 ? 2 : 1); return sum; }
        private List<float[]> TransportPostures(ArmTrackState start)
        {
            var postures = new List<float[]> { start.JointAnglesDegrees };
            foreach (float shoulder in new[] { 45f, 0f, -20f })
            {
                var q = start.JointAnglesDegrees;
                q[1] = Mathf.Clamp(shoulder, rig.AngleLimitsDegrees[1].x, rig.AngleLimitsDegrees[1].y);
                q[2] = Mathf.Clamp(90, rig.AngleLimitsDegrees[2].x, rig.AngleLimitsDegrees[2].y);
                q[4] = Mathf.Clamp(Mathf.Sign(Mathf.Sin((start[4] + rig.ZeroOffsetsDegrees[4]) * Mathf.Deg2Rad)) * 60 - rig.ZeroOffsetsDegrees[4], rig.AngleLimitsDegrees[4].x, rig.AngleLimitsDegrees[4].y);
                postures.Add(q);
            }
            return postures;
        }
        private List<Pose> TransferApproaches(Pose target)
        {
            var poses = new List<Pose>();
            if (preferLowerTransferCorridor)
                foreach (float distance in new[] { transferApproachMillimeters * .001f, transferApproachMillimeters * .002f })
                    poses.Add(new Pose(target.position - Vector3.up * distance, target.rotation));
            if (requireLowerTransferCorridor) return poses;
            foreach (float distance in new[] { transferApproachMillimeters * .001f, transferApproachMillimeters * .002f })
            {
                foreach (var axis in new[] { -(target.rotation * Vector3.forward), target.rotation * Vector3.forward, Vector3.up, Vector3.down, Vector3.right, Vector3.left, Vector3.forward, Vector3.back })
                    poses.Add(new Pose(target.position + axis * distance, target.rotation));
            }
            return poses;
        }
        private sealed class TransferStation
        { public float E1; public List<ArmTrackState> EndSolutions; }
        private IEnumerator PlanStagedTransferRoutine()
        {
            ClearCoordinatedPlan(); stagedTransferLog.Clear(); exactStationPlan = false;
            path.Clear(); motionProfile = null; ClearGridGizmoData(); ClearPathVisualization(); InvalidateEndpointChecks();
            planningTimeSeconds = 0; planningIterations = 0; estimatedTravelTimeSeconds = 0;
            if (!ValidateSetup()) yield break;
            CacheColliders();
            if (robotColliders == null || robotColliders.Length == 0 || obstacleColliders.Length == 0)
            { Fail("Track transfer requires robot, carriage/rail and obstacle collision geometry."); yield break; }
            inverseKinematics.SolveContinuously = false; InitializeGoalSelection(endTarget.rotation);
            CancelGridSearch(); gridSearchCancellation = new System.Threading.CancellationTokenSource(); var token = gridSearchCancellation.Token;
            coordinatedTargets = new CoordinatedTargets(this);
            var session = new CoordinatedSearchSession { Result = new CoordinatedMoveLPlanner.Result() }; coordinatedSearch = session;
            var singularity = new CartesianLinearPlanner(CreateAlgorithmWorld());
            var clock = System.Diagnostics.Stopwatch.StartNew();
            double Deadline() => session.DeadlineSeconds ?? coordinatedSearchSeconds;
            bool Stopped() => session.Cancelled || token.IsCancellationRequested || session.Clock.Elapsed.TotalSeconds >= Deadline() || session.Result.Checks >= 100000;
            CoordinatedStateEvaluation Evaluate(ArmTrackState state)
            {
                session.Result.Checks++; planningIterations = session.Result.Checks; planningTimeSeconds = (float)session.Clock.Elapsed.TotalSeconds;
                return EvaluateCoordinatedState(state, singularity);
            }
            var stations = LinearTrackStationCandidates.Build(LinearTrack.Profile, LinearTrack.CurrentE1Millimeters, transferStationSpacingMillimeters, transferMaximumStations);
            var starts = new List<ArmTrackState>(); var destinations = new List<TransferStation>();
            TrackTransferPlanner.Result transfer = null; CoordinatedMoveLPlanner.Result final = null;
            float selectedE1 = 0; bool found = false; int connections = 0;
            List<Pose> selectedCorridorAnchors = null;
            try
            {
                // Establish exact, collision-free End feasibility independently of a
                // connecting route; retain multiple destination stations.
                foreach (float e1 in stations)
                {
                    if (Stopped()) break;
                    Status = $"Track transfer: checking exact End IK and clearance at E1 {e1:F1} mm...";
                    diagnosticPhase = "End IK station scan";
                    var ends = ValidTransferIk(new Pose(endTarget.position, endTarget.rotation), e1, Evaluate, out int raw);
                    TransferLog($"End E1 {e1:F1} mm: {raw} IK, {ends.Count} collision-free nonsingular exact solutions.");
                    if (ends.Count > 0) destinations.Add(new TransferStation { E1 = e1, EndSolutions = ends });
                    yield return null;
                }
                EndIkState = destinations.Count > 0 ? EndpointIkState.Valid : EndpointIkState.Invalid;
                EndIkSummary = destinations.Count > 0 ? $"End: Valid — exact collision-free IK at {destinations.Count} candidate track stations" : "End: No validated exact pose at sampled track stations";
                if (avoidOverextension && stretchScope == StretchScope.WholeTrajectory)
                    destinations.Sort((a, b) =>
                    {
                        float Score(TransferStation station) { float best = float.PositiveInfinity; foreach (var state in station.EndSolutions) best = Mathf.Min(best, ReachPenalty(state, false)); return best; }
                        int order = Score(a).CompareTo(Score(b)); return order != 0 ? order : Mathf.Abs(a.E1 - LinearTrack.CurrentE1Millimeters).CompareTo(Mathf.Abs(b.E1 - LinearTrack.CurrentE1Millimeters));
                    });
                if (destinations.Count > 0)
                    foreach (float e1 in stations)
                    {
                        if (Stopped()) break;
                        diagnosticPhase = "Start IK station scan";
                        var roots = ValidTransferIk(new Pose(startPose.position, startPose.rotation), e1, Evaluate, out int raw);
                        TransferLog($"Start E1 {e1:F1} mm: {raw} IK, {roots.Count} validated solutions.");
                        if (roots.Count > 0)
                        {
                            float Risk(List<ArmTrackState> list) { float risk = float.PositiveInfinity; foreach (var state in list) risk = Mathf.Min(risk, ReachPenalty(state, false)); return risk; }
                            if (starts.Count == 0 || Risk(roots) < Risk(starts)) { starts.Clear(); starts.AddRange(roots); }
                            if (!ReachAllowsTrackSearch || Risk(roots) <= .000001f) break;
                        }
                        yield return null;
                    }
                StartIkState = starts.Count > 0 ? EndpointIkState.Valid : EndpointIkState.Invalid;
                StartIkSummary = starts.Count > 0 ? $"Start: Valid — {starts.Count} exact collision-free IK at E1 {starts[0].E1Millimeters:F1} mm" : "Start: No validated exact pose at sampled track stations";
                var approaches = TransferApproaches(new Pose(endTarget.position, endTarget.rotation));
                foreach (var destination in destinations)
                {
                    if (found || Stopped() || starts.Count == 0 || connections >= 12) break;
                    foreach (var approach in approaches)
                    {
                        if (found || Stopped() || connections >= 12) break;
                        var arrivalStates = ValidTransferIk(approach, destination.E1, Evaluate, out _);
                        foreach (var arrival in arrivalStates)
                        {
                            if (found || Stopped() || connections >= 12) break;
                            connections++;
                            diagnosticAttempt = connections; diagnosticPhase = "Final MoveL";
                            if (LastPlanReport != null) LastPlanReport.Connections = connections;
                            Status = $"Track transfer: validating approach and final MoveL at E1 {destination.E1:F1} mm (connection {connections}/12)...";
                            var finalPoses = new List<Pose> { approach };
                            foreach (var t in intermediateWaypoints ?? Array.Empty<Transform>()) if (t != null) finalPoses.Add(new Pose(t.position, t.rotation));
                            finalPoses.Add(new Pose(endTarget.position, endTarget.rotation));
                            var finalWorld = new CoordinatedMoveLPlanner.World
                            {
                                StartStations = new[] { destination.E1 }, MinimumE1 = destination.E1, MaximumE1 = destination.E1, PreferredE1 = destination.E1,
                                BeamWidth = coordinatedBeamWidth, CartesianStepMillimeters = coordinatedCartesianStepMillimeters, TrackStepMillimeters = coordinatedTrackStepMillimeters,
                                PositionToleranceMeters = Mathf.Min(.001f, positionClosureTolerance), RotationToleranceDegrees = Mathf.Min(.25f, rotationClosureToleranceDegrees),
                                JointSweepStepDegrees = Mathf.Min(.25f, collisionResolutionDegrees), ShouldStop = Stopped,
                                Penalty = state => ReachPenalty(state, false),
                                Evaluate = Evaluate,
                                Rejected = CaptureSearchRejection,
                                LimitReached = CaptureSearchLimit,
                                Solve = (pose, e1) => pose.position.Equals(approach.position) && Quaternion.Angle(pose.rotation, approach.rotation) < .001f
                                    ? new[] { arrival } : SolveCoordinatedCandidates(pose, e1),
                                Progress = text => Status = "Final approach: " + text
                            };
                            final = new CoordinatedMoveLPlanner.Result();
                            var finalSearch = new CoordinatedMoveLPlanner(finalWorld).Search(finalPoses, final);
                            while (finalSearch.MoveNext()) yield return finalSearch.Current;
                            if (!final.Found) { TransferLog($"E1 {destination.E1:F1}: final MoveL rejected: {final.Failure}"); continue; }
                            int rootAttempts = 0;
                            var rootTrials = starts.GetRange(0, Math.Min(4, starts.Count)); int corridorRoots = rootTrials.Count;
                            if (preferLowerTransferCorridor && !requireLowerTransferCorridor) rootTrials.AddRange(new List<ArmTrackState>(rootTrials));
                            foreach (var start in rootTrials)
                            {
                                if (Stopped()) break; rootAttempts++;
                                bool tryCorridor = preferLowerTransferCorridor && rootAttempts <= corridorRoots;
                                bool tryFree = !preferLowerTransferCorridor || rootAttempts > corridorRoots;
                                diagnosticPhase = tryCorridor ? "Lower transfer corridor" : "Free-posture transfer";
                                var world = new TrackTransferPlanner.World
                                {
                                    Evaluate = state => EvaluateTransferReach(state, Evaluate), ShouldStop = Stopped, JointLimits = (Vector2[])rig.AngleLimitsDegrees.Clone(),
                                    Rejected = CaptureSearchRejection,
                                    LimitReached = CaptureSearchLimit,
                                    Penalty = state => ReachPenalty(state, true),
                                    WristZeroOffsetDegrees = rig.ZeroOffsetsDegrees[4], JointStepDegrees = Mathf.Min(.25f, collisionResolutionDegrees),
                                    MaximumIterations = transferPostureIterations, Seed = randomSeed + connections * 101 + rootAttempts,
                                    RefineAutomatically = false,
                                    CanRefine = () => !Stopped() && session.Result.Checks < 99000 && session.Clock.Elapsed.TotalSeconds < coordinatedSearchSeconds - .75,
                                    Progress = text => Status = $"E1 {start.E1Millimeters:F1} → {destination.E1:F1} mm: {text}"
                                };
                                transfer = new TrackTransferPlanner.Result();
                                List<Pose> corridorAnchors = null;
                                if (tryCorridor && !Stopped())
                                {
                                    var startTcp = new Pose(startPose.position, startPose.rotation);
                                    var geometry = new LowerTransferCorridor(startTcp, approach, transferCorridorDropMm, pathAccuracyMillimeters);
                                    var horizontal = approach.position - startTcp.position; horizontal.y = 0;
                                    if (horizontal.sqrMagnitude > .0001f)
                                    {
                                        var attemptClock = System.Diagnostics.Stopwatch.StartNew();
                                        double attemptBudget = Math.Min(5, Math.Max(.5, (Deadline() - session.Clock.Elapsed.TotalSeconds) * .2));
                                        float schedulePower = 1;
                                        float Station(Pose pose)
                                        { float f = geometry.TrackFraction(pose); return f < .00001f ? start.E1Millimeters : f > .99999f ? arrival.E1Millimeters : Mathf.Lerp(start.E1Millimeters, arrival.E1Millimeters, Mathf.Pow(f, schedulePower)); }
                                        var corridorWorld = new CoordinatedMoveLPlanner.World
                                        {
                                            Rejected = CaptureSearchRejection,
                                            LimitReached = CaptureSearchLimit,
                                            StartStations = new[] { start.E1Millimeters }, MinimumE1 = Mathf.Min(start.E1Millimeters, arrival.E1Millimeters), MaximumE1 = Mathf.Max(start.E1Millimeters, arrival.E1Millimeters),
                                            PreferredE1 = start.E1Millimeters, StationForPose = Station, WristZeroOffsetDegrees = rig.ZeroOffsetsDegrees[4],
                                            BeamWidth = Mathf.Min(4, coordinatedBeamWidth), CartesianStepMillimeters = Mathf.Min(20, coordinatedCartesianStepMillimeters),
                                            TrackStepMillimeters = Mathf.Max(coordinatedTrackStepMillimeters, Mathf.Abs(arrival.E1Millimeters - start.E1Millimeters) * .04f / horizontal.magnitude),
                                            PositionToleranceMeters = Mathf.Min(.001f, positionClosureTolerance), RotationToleranceDegrees = Mathf.Min(.25f, rotationClosureToleranceDegrees),
                                            JointSweepStepDegrees = Mathf.Min(.25f, collisionResolutionDegrees), Evaluate = state => EvaluateTransferReach(state, Evaluate),
                                            Penalty = state => ReachPenalty(state, true),
                                            ShouldStop = () => Stopped() || attemptClock.Elapsed.TotalSeconds > attemptBudget,
                                            Solve = (pose, e1) => Vector3.Distance(pose.position, startTcp.position) < .000001f && Quaternion.Angle(pose.rotation, startTcp.rotation) < .001f ? new[] { start } :
                                                Vector3.Distance(pose.position, approach.position) < .000001f && Quaternion.Angle(pose.rotation, approach.rotation) < .001f ? new[] { arrival } : SolveCoordinatedCandidates(pose, e1),
                                            Progress = text => Status = "Lower transfer corridor: " + text
                                        };
                                        CoordinatedMoveLPlanner.Result corridor = null; float bestPeak = float.PositiveInfinity;
                                        foreach (float power in ReachPolicy.Applies(true) ? new[] { 1f, .8f, 1.25f } : new[] { 1f })
                                        {
                                            if (Stopped() || attemptClock.Elapsed.TotalSeconds > attemptBudget) break;
                                            schedulePower = power;
                                            var candidate = new CoordinatedMoveLPlanner.Result();
                                            var searchCorridor = new CoordinatedMoveLPlanner(corridorWorld).Search(geometry.Samples, candidate);
                                            while (searchCorridor.MoveNext()) yield return searchCorridor.Current;
                                            if (!candidate.Found) { TransferLog("Lower corridor schedule rejected: " + candidate.Failure); continue; }
                                            float peak = avoidOverextension ? PeakReach(candidate.States) : 0;
                                            if (peak < bestPeak) { corridor = candidate; bestPeak = peak; }
                                            if (!ReachPolicy.Applies(true) || peak <= ReachPolicy.Limit) break;
                                        }
                                        if (corridor != null)
                                        {
                                            transfer.Found = true; transfer.States.AddRange(corridor.States); transfer.Poses.AddRange(corridor.Poses);
                                            transfer.Checks = corridor.Checks; corridorAnchors = geometry.ProgramAnchors;
                                            TransferLog($"Validated lower corridor: drop {transferCorridorDropMm:F0} mm, rounded shoulders {pathAccuracyMillimeters:F0} mm. Prescribed TCP motion; E1 progresses continuously.");
                                        }
                                        else
                                        {
                                            TransferLog("Lower corridor rejected: no complete validated E1 schedule within the attempt budget.");
                                            if (attemptClock.Elapsed.TotalSeconds > attemptBudget)
                                                CaptureSearchLimit(PlanDiagnosticCode.TimeLimit, $"This lower-corridor connection attempt reached its {attemptBudget:G9} s local budget. Other candidates may remain.");
                                        }
                                    }
                                }
                                if (!transfer.Found && tryFree && !requireLowerTransferCorridor)
                                {
                                    TransferLog(preferLowerTransferCorridor ? "Lower corridor unavailable for this connection; trying validated free-posture transfer." : "Free-posture transfer selected.");
                                    var search = new TrackTransferPlanner(world).Search(start, arrival, TransportPostures(start), transfer);
                                    while (search.MoveNext()) yield return search.Current;
                                }
                                if (transfer.Found && !Stopped())
                                {
                                    found = true; selectedE1 = destination.E1;
                                    selectedCorridorAnchors = corridorAnchors;
                                    // Once a complete solution exists, optimization has
                                    // its own bounded window; search timeout must not
                                    // discard that solution just because refinement ends.
                                    session.DeadlineSeconds = session.Clock.Elapsed.TotalSeconds + transferOptimizationBudgetSeconds + 5;
                                    var beforeStates = new List<ArmTrackState>(transfer.States); var beforePoses = new List<Pose>(transfer.Poses); var beforePhases = new List<int>();
                                    for (int i = 0; i < transfer.States.Count; i++) beforePhases.Add(i == 0 ? 0 : (int)TransferPhase(transfer.States[i], transfer.States[0], selectedE1));
                                    for (int i = 1; i < final.States.Count; i++) { beforeStates.Add(final.States[i]); beforePoses.Add(final.Poses[i]); beforePhases.Add(4); }
                                    PreserveAuthoredWaypoints(beforePoses, beforePhases);
                                    TransferWaypointsBeforeOptimization = TrackTransferWaypoints.SelectBeforeOptimization(beforeStates, beforePoses, beforePhases, transferTargetWaypoints).Length;
                                    var optimizeClock = System.Diagnostics.Stopwatch.StartNew();
                                    world.CanRefine = () => !Stopped() && session.Result.Checks < 99000 && optimizeClock.Elapsed.TotalSeconds < transferOptimizationBudgetSeconds;
                                    if (corridorAnchors == null)
                                    {
                                        var optimize = new TrackTransferPlanner(world).RefineBidirectional(transfer);
                                        while (optimize.MoveNext()) yield return optimize.Current;
                                    }
                                    else TransferLog("Lower corridor preserved: joint-space shortcutting skipped to retain retract / level travel / upward approach.");
                                    // Revalidate the accepted dense trajectory before publication.
                                    // Unchanged samples use the exact-context collision cache.
                                    foreach (var section in new[] { transfer.States, final.States })
                                        foreach (var state in section)
                                        {
                                            if (Stopped()) break;
                                            var check = EvaluateCoordinatedState(state, singularity);
                                            if (check.Valid && ReferenceEquals(section, transfer.States) && ReachRejects(state.JointAnglesDegrees, true, out string reachReason)) check = new CoordinatedStateEvaluation { Valid = false, Failure = reachReason, Tcp = check.Tcp };
                                            if (!check.Valid) { found = false; TransferLog("Post-optimization validation rejected: " + check.Failure); break; }
                                            if (clock.Elapsed.TotalMilliseconds >= 3) { yield return null; clock.Restart(); }
                                        }
                                    TransferOptimizationTimeSeconds = (float)optimizeClock.Elapsed.TotalSeconds;
                                    break;
                                }
                                TransferLog($"E1 {destination.E1:F1}: transfer rejected: {transfer.Failure}");
                            }
                        }
                        if (clock.Elapsed.TotalMilliseconds >= 3) { yield return null; clock.Restart(); }
                    }
                }
            }
            finally { if (coordinatedSearch == session) coordinatedSearch = null; coordinatedChecks.Clear(); }
            planningIterations = session.Result.Checks; planningTimeSeconds = (float)session.Clock.Elapsed.TotalSeconds;
            if (session.Cancelled || token.IsCancellationRequested) yield break;
            if (!found || session.Clock.Elapsed.TotalSeconds >= Deadline() || !coordinatedTargets.Matches(this))
            {
                string reason = destinations.Count == 0 ? "No validated exact End configuration at the sampled E1 values." : starts.Count == 0 ? "No validated exact Start configuration." :
                    session.Clock.Elapsed.TotalSeconds >= Deadline() ? "Search or validation time budget reached." : "No complete transfer plus final MoveL within the bounded candidate search.";
                coordinatedSummary = "Track transfer failed: " + reason + $" {planningIterations} checks; {connections} connections. Original setup retained; no partial motion.";
                Fail(coordinatedSummary + "\n" + string.Join("\n", stagedTransferLog),
                    session.Result.Checks >= 100000 ? PlanDiagnosticCode.CheckLimit : session.Clock.Elapsed.TotalSeconds >= Deadline() ? PlanDiagnosticCode.TimeLimit :
                    connections >= 12 ? PlanDiagnosticCode.AttemptLimit : PlanDiagnosticCode.NoCompleteRoute); yield break;
            }
            stagedPath = true;
            UsedLowerTransferCorridor = selectedCorridorAnchors != null;
            for (int i = 0; i < transfer.States.Count; i++)
            {
                var state = transfer.States[i]; coordinatedPath.Add(state); coordinatedPoses.Add(transfer.Poses[i]); stagedCartesianEdges.Add(i == 0 || UsedLowerTransferCorridor);
                stagedPhases.Add(i == 0 ? TrackTransferPhase.Start : TransferPhase(state, transfer.States[0], selectedE1));
            }
            for (int i = 1; i < final.States.Count; i++)
            { coordinatedPath.Add(final.States[i]); coordinatedPoses.Add(final.Poses[i]); stagedCartesianEdges.Add(true); stagedPhases.Add(TrackTransferPhase.FinalMoveL); }
            foreach (var state in coordinatedPath) path.Add(state.JointAnglesDegrees);
            var waypointPhases = new List<int>(); foreach (var phase in stagedPhases) waypointPhases.Add((int)phase);
            PreserveAuthoredWaypoints(coordinatedPoses, waypointPhases);
            var reductionClock = System.Diagnostics.Stopwatch.StartNew();
            transferWaypointIndices = TrackTransferWaypoints.Select(coordinatedPath, coordinatedPoses, waypointPhases, transferTargetWaypoints, transferReductionPositionMm, transferReductionRotationDegrees);
            if (UsedLowerTransferCorridor)
            {
                // Rounded corridor legs are motion primitives. Show their on-path
                // anchors, not every dense IK state used to execute those primitives.
                var anchors = new SortedSet<int> { 0, coordinatedPath.Count - 1 };
                foreach (var anchor in selectedCorridorAnchors)
                {
                    int closest = 0; float distance = float.PositiveInfinity;
                    for (int i = 0; i < transfer.Poses.Count; i++)
                    { float d = (transfer.Poses[i].position - anchor.position).sqrMagnitude; if (d < distance) { distance = d; closest = i; } }
                    anchors.Add(closest);
                }
                foreach (int index in transferWaypointIndices)
                    if (index > transfer.States.Count || index == transfer.States.Count && waypointPhases[index] >= 1000) anchors.Add(index);
                transferWaypointIndices = new int[anchors.Count]; anchors.CopyTo(transferWaypointIndices);
            }
            TransferOptimizationTimeSeconds += (float)reductionClock.Elapsed.TotalSeconds;
            var phaseCounts = new int[5]; foreach (int i in transferWaypointIndices) phaseCounts[(int)stagedPhases[i]]++;
            TransferOptimizationSummary = $"Waypoints {TransferWaypointsBeforeOptimization} → {PlannedWaypointCount}; preferred maximum {transferTargetWaypoints}; optimization {TransferOptimizationTimeSeconds:F2} s. " +
                $"Start {phaseCounts[0]}, retract {phaseCounts[1]}, track travel {phaseCounts[2]}, arrival {phaseCounts[3]}, final MoveL {phaseCounts[4]}. " +
                (PlannedWaypointCount > transferTargetWaypoints ? "Required poses, phase boundaries or tolerance limits prevent further reduction." : "Within preferred maximum; no padding.");
            TransferLog(TransferOptimizationSummary);
            exactStationPlan = true; adaptiveGoalUsed = false; requestedGoalPosition = plannedGoalPosition = endTarget.position; requestedGoalRotation = plannedGoalRotation = endTarget.rotation;
            coordinatedSummary = $"Track transfer → final MoveL: destination E1 {selectedE1:F1} mm, {path.Count} validated samples, {planningIterations} checks, {planningTimeSeconds:F2} s. {transfer.RemovedControlPoints} unnecessary transfer corners removed. Continuous E1 timing, acceleration {transferTrackAcceleration:F0} mm/s². Exact Start/End; no wrist flips.";
            coordinatedSummary += $" Planned waypoints {PlannedWaypointCount} (target {transferTargetWaypoints}); dense validation samples {coordinatedPath.Count}.";
            coordinatedSummary += " " + TransferOptimizationSummary;
            coordinatedSummary += UsedLowerTransferCorridor ? " Lower corridor used: retract, rounded level transfer, upward approach; all dense IK/collision checks retained." : " Free-posture transfer used; TCP corridor shape is not guaranteed.";
            TransferLog(coordinatedSummary); lastPlanSummary = coordinatedSummary; Status = coordinatedSummary;
            DrawCoordinatedPath(); yield return StartCoroutine(ExecuteCoordinatedPath());
        }
    }
}
