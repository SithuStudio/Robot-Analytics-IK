using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        private sealed class TransitSearchJob
        {
            public int Id, Queries, NewChecks;
            public ArmTrackState Start, Arrival;
            public Pose Approach;
            public string Phase = "Final MoveL";
            public IEnumerator Search;
            public bool Done;
            public bool Visited;
            public float Progress;
            public readonly HashSet<string> VisitedGuides = new HashSet<string>();
            public CoordinatedMoveLPlanner.Result Final;
        }
        private sealed class TransitSearchCandidate
        {
            public IEnumerator Search;
            public CoordinatedMoveLPlanner.Result Cartesian;
            public TrackTransferPlanner.Result Transfer;
            public ObstacleAwareTransitCorridor Corridor;
            public string Name;
            public bool Done;
            public int Visits;
            public bool Found => Cartesian?.Found == true || Transfer?.Found == true;
        }
        private sealed class TransitBest
        {
            public TransitSearchJob Job;
            public TrackTransferPlanner.Result Transfer;
            public List<Pose> Anchors;
            public string Name;
            public double Cost = double.PositiveInfinity;
            public int Improvements;
            public float Margin = float.NaN;
        }
        private Vector3 CalibratedTrackDirection()
        {
            var track = LinearTrack; var axis = new LinearTrackAxis(track.Profile);
            if (axis.TryEvaluate(track.MinimumMillimeters, out var a, out _) && axis.TryEvaluate(track.MaximumMillimeters, out var b, out _))
                return track.TrackRoot.TransformVector(b.position - a.position).normalized;
            return track.TrackRoot.forward;
        }
        private List<Bounds> TransitGuideObstacles()
        {
            var guides = new List<Bounds>();
            foreach (var collider in obstacleColliders)
            {
                if (collider == null || !collider.enabled || !collider.gameObject.activeInHierarchy || collider.isTrigger || LinearTrack.IsCarriageCollider(collider)) continue;
                // Hollow/concave car-body bounds must not be interpreted as solid boxes.
                if (collider is MeshCollider mesh && !mesh.convex) continue;
                guides.Add(collider.bounds);
            }
            return guides;
        }
        private IEnumerator PlanObstacleAwareTransferRoutine()
        {
            ClearCoordinatedPlan(); stagedTransferLog.Clear(); exactStationPlan = false;
            path.Clear(); motionProfile = null; ClearGridGizmoData(); ClearPathVisualization(); InvalidateEndpointChecks();
            planningTimeSeconds = 0; planningIterations = 0; estimatedTravelTimeSeconds = 0;
            if (!ValidateSetup()) yield break;
            CacheColliders();
            if (robotColliders == null || robotColliders.Length == 0 || obstacleColliders.Length == 0)
            { Fail("Obstacle-aware transfer requires robot, carriage/rail and obstacle collision geometry.", PlanDiagnosticCode.InvalidSetup); yield break; }
            inverseKinematics.SolveContinuously = false; InitializeGoalSelection(endTarget.rotation);
            CancelGridSearch(); gridSearchCancellation = new System.Threading.CancellationTokenSource(); var token = gridSearchCancellation.Token;
            coordinatedTargets = new CoordinatedTargets(this);
            var session = new CoordinatedSearchSession { Result = new CoordinatedMoveLPlanner.Result(), DeadlineSeconds = coordinatedSearchSeconds + transferOptimizationBudgetSeconds + 5 };
            coordinatedSearch = session;
            var singularity = new CartesianLinearPlanner(CreateAlgorithmWorld());
            var best = new TransitBest(); var jobs = new List<TransitSearchJob>(); var deferred = new Queue<TransitSearchJob>();
            var stations = LinearTrackStationCandidates.Build(LinearTrack.Profile, LinearTrack.CurrentE1Millimeters, transferStationSpacingMillimeters, transferMaximumStations);
            var destinations = new List<TransferStation>(); var starts = new List<ArmTrackState>();
            var guides = TransitGuideObstacles(); var direction = CalibratedTrackDirection();
            // Analytic IK depends on the frozen target/base geometry, not the parent
            // frontier node. Reuse it across roots, refinements and station jobs.
            var ikCache = new Dictionary<(Vector3, Quaternion, float), IReadOnlyList<ArmTrackState>>();
            var finalCache = new Dictionary<(ArmTrackStateKey, Vector3, Quaternion), CoordinatedMoveLPlanner.Result>();
            int ikRequests = 0, ikHits = 0;
            IReadOnlyList<ArmTrackState> Solve(Pose pose, float e1)
            {
                ikRequests++;
                var key = (pose.position, pose.rotation, e1);
                if (!ikCache.TryGetValue(key, out var states))
                {
                    states = SolveCoordinatedCandidates(pose, e1);
                    // Bound memory; eviction only causes recomputation, never a validity change.
                    if (ikCache.Count >= 16384) ikCache.Clear();
                    ikCache[key] = states;
                }
                else ikHits++;
                return states;
            }
            double improveUntil = double.PositiveInfinity;
            bool Cancelled() => session.Cancelled || token.IsCancellationRequested;
            bool SearchStopped() => Cancelled() || session.Clock.Elapsed.TotalSeconds >= Math.Min(coordinatedSearchSeconds, improveUntil) || session.Result.Checks >= 90000;
            CoordinatedStateEvaluation Evaluate(ArmTrackState state)
            {
                // Cached queries remain counted in the report, but do not exhaust the expensive-physics allowance.
                var key = new ArmTrackStateKey(state, collisionContextRevision);
                if (!coordinatedChecks.ContainsKey(key)) session.Result.Checks++;
                planningIterations = session.Result.Checks; planningTimeSeconds = (float)session.Clock.Elapsed.TotalSeconds;
                return EvaluateCoordinatedState(state, singularity);
            }
            try
            {
                foreach (float e1 in stations)
                {
                    if (SearchStopped()) break;
                    diagnosticPhase = "End IK station scan";
                    var ends = ValidTransferIk(new Pose(endTarget.position, endTarget.rotation), e1, Evaluate, out int raw);
                    TransferLog($"End E1 {e1:F1} mm: {raw} IK, {ends.Count} validated exact solutions.");
                    if (ends.Count > 0) destinations.Add(new TransferStation { E1 = e1, EndSolutions = ends });
                    yield return null;
                }
                EndIkState = destinations.Count > 0 ? EndpointIkState.Valid : EndpointIkState.Invalid;
                EndIkSummary = destinations.Count > 0 ? $"End: Valid — exact collision-free IK at {destinations.Count} candidate track stations" : "End: No validated exact pose at sampled stations";
                if (avoidOverextension && stretchScope == StretchScope.WholeTrajectory)
                    destinations.Sort((a, b) =>
                    {
                        float Score(TransferStation station) { float cost = float.PositiveInfinity; foreach (var state in station.EndSolutions) cost = Mathf.Min(cost, ReachPenalty(state, false)); return cost; }
                        int compare = Score(a).CompareTo(Score(b)); return compare != 0 ? compare : Mathf.Abs(a.E1 - LinearTrack.CurrentE1Millimeters).CompareTo(Mathf.Abs(b.E1 - LinearTrack.CurrentE1Millimeters));
                    });
                foreach (float e1 in stations)
                {
                    if (SearchStopped() || destinations.Count == 0) break;
                    diagnosticPhase = "Start IK station scan";
                    var roots = ValidTransferIk(new Pose(startPose.position, startPose.rotation), e1, Evaluate, out _);
                    if (roots.Count > 0) { starts.AddRange(roots); break; }
                    yield return null;
                }
                StartIkState = starts.Count > 0 ? EndpointIkState.Valid : EndpointIkState.Invalid;
                StartIkSummary = starts.Count > 0 ? $"Start: Valid — {starts.Count} exact collision-free IK at E1 {starts[0].E1Millimeters:F1} mm" : "Start: No validated exact pose at sampled stations";
                var approaches = TransferApproaches(new Pose(endTarget.position, endTarget.rotation));
                var arrivals = new Dictionary<string, List<ArmTrackState>>();
                int jobLimit = Math.Min(64, Math.Max(8, destinations.Count * Math.Min(4, starts.Count)));
                // Interleave station before returning to another root/approach. Search
                // enumerators keep their frontier when suspended, rather than restarting.
                for (int approachIndex = 0; approachIndex < approaches.Count && jobs.Count < jobLimit && !SearchStopped(); approachIndex++)
                for (int rootIndex = 0; rootIndex < Math.Min(4, starts.Count) && jobs.Count < jobLimit && !SearchStopped(); rootIndex++)
                foreach (var station in destinations)
                {
                    if (SearchStopped() || jobs.Count >= jobLimit) break;
                    string key = approachIndex + ":" + station.E1.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
                    diagnosticPhase = "Approach IK preflight";
                    if (!arrivals.TryGetValue(key, out var options))
                    { options = ValidTransferIk(approaches[approachIndex], station.E1, Evaluate, out _); arrivals.Add(key, options); }
                    var ordered = new List<ArmTrackState>(options);
                    var root = starts[rootIndex]; float wristSign = Mathf.Sin((root[4] + rig.ZeroOffsetsDegrees[4]) * Mathf.Deg2Rad);
                    ordered.Sort((a,b) => JointDifference(a,root.JointAnglesDegrees).CompareTo(JointDifference(b,root.JointAnglesDegrees)));
                    int alternatives = 0;
                    foreach (var arrival in ordered)
                    {
                        if (jobs.Count >= jobLimit) break;
                        if (wristSign * Mathf.Sin((arrival[4] + rig.ZeroOffsetsDegrees[4]) * Mathf.Deg2Rad) <= 0) continue;
                        var job = new TransitSearchJob { Id = jobs.Count + 1, Start = starts[rootIndex], Arrival = arrival, Approach = approaches[approachIndex] };
                        job.Search = SearchTransitJob(job, guides, direction, Evaluate, Solve, finalCache, SearchStopped, best, () =>
                        {
                            if (double.IsPositiveInfinity(improveUntil)) improveUntil = session.Clock.Elapsed.TotalSeconds + Math.Min(.25, transferOptimizationBudgetSeconds);
                        });
                        if (alternatives++ == 0) jobs.Add(job); else if (deferred.Count < 64) deferred.Enqueue(job);
                    }
                    yield return null;
                }
                TransferLog($"Fair search prepared {jobs.Count} resumable connections across {destinations.Count} endpoint-feasible stations; clearance remains {minimumObstacleClearance * 1000:G9} mm. Search allowance 90000 uncached evaluations; 10000 reserved for refinement/validation.");
                var activeClock = System.Diagnostics.Stopwatch.StartNew(); int cursor = 0, remaining = jobs.Count, visited = 0, ticks = 0;
                while ((remaining > 0 || deferred.Count > 0 && jobs.Count < 64) && !SearchStopped())
                {
                    if (remaining == 0)
                    {
                        int count = Math.Min(jobLimit, Math.Min(deferred.Count, 64 - jobs.Count));
                        for (int i = 0; i < count; i++) { var next = deferred.Dequeue(); next.Id = jobs.Count + 1; jobs.Add(next); remaining++; }
                        TransferLog($"First connection wave finished; resuming {count} alternate arrival-posture candidates.");
                    }
                    var job = jobs[cursor]; cursor = (cursor + 1) % jobs.Count;
                    if (job.Done) continue;
                    // Every job gets a first visit, and one third of subsequent
                    // slices remain round-robin. Use the rest to finish frontiers
                    // that have already validated more of a complete corridor.
                    if (visited == jobs.Count && ++ticks % 3 != 0)
                    {
                        foreach (var candidate in jobs)
                            if (!candidate.Done && candidate.Progress > job.Progress) job = candidate;
                    }
                    diagnosticAttempt = job.Id; diagnosticPhase = job.Phase;
                    if (!job.Search.MoveNext()) { job.Done = true; remaining--; (job.Search as IDisposable)?.Dispose(); }
                    if (!job.Visited && job.Queries > 0) { job.Visited = true; visited++; }
                    if (LastPlanReport != null) LastPlanReport.Connections = visited;
                    Status = $"Obstacle-aware transfer: connection {job.Id}/{jobs.Count}, E1 {job.Arrival.E1Millimeters:F1} mm · {job.Phase} · {session.Result.Checks} new evaluations";
                    if (activeClock.Elapsed.TotalMilliseconds >= 3) { yield return null; activeClock.Restart(); }
                }
                TransferLog($"Analytic IK: {ikRequests - ikHits} solves, {ikHits} reused queries; {finalCache.Count} reusable validated final approaches. Completed-candidate progress is excluded from scheduling.");
                foreach (var job in jobs) TransferLog($"Connection {job.Id}: E1 {job.Start.E1Millimeters:F1} → {job.Arrival.E1Millimeters:F1} mm; {job.Queries} queries / {job.NewChecks} new evaluations; {job.Phase}; {(job.Done ? "finished" : "suspended at search stop")}. Visited: {string.Join(", ", job.VisitedGuides)}." );
                if (Cancelled()) yield break;
                if (best.Transfer == null)
                {
                    var code = session.Result.Checks >= 90000 ? PlanDiagnosticCode.CheckLimit : session.Clock.Elapsed.TotalSeconds >= coordinatedSearchSeconds ? PlanDiagnosticCode.TimeLimit : PlanDiagnosticCode.NoCompleteRoute;
                    Fail($"Obstacle-aware transfer found no complete validated connection. {session.Result.Checks} uncached checks, {jobs.Count} connection candidates. Clearance {minimumObstacleClearance * 1000:G9} mm was not relaxed. No partial motion.\n" + string.Join("\n", stagedTransferLog), code);
                    yield break;
                }
                // Stop all candidate search before using its separately reserved optimization allowance.
                foreach (var job in jobs) { (job.Search as IDisposable)?.Dispose(); job.Done = true; }
                diagnosticAttempt = best.Job.Id; diagnosticPhase = "Transfer refinement";
                var beforeStates = new List<ArmTrackState>(best.Transfer.States); var beforePoses = new List<Pose>(best.Transfer.Poses); var phases = new List<int>();
                for (int i = 0; i < best.Transfer.States.Count; i++) phases.Add(i == 0 ? 0 : (int)TransferPhase(best.Transfer.States[i], best.Transfer.States[0], best.Job.Arrival.E1Millimeters));
                for (int i = 1; i < best.Job.Final.States.Count; i++) { beforeStates.Add(best.Job.Final.States[i]); beforePoses.Add(best.Job.Final.Poses[i]); phases.Add(4); }
                PreserveAuthoredWaypoints(beforePoses, phases);
                TransferWaypointsBeforeOptimization = TrackTransferWaypoints.SelectBeforeOptimization(beforeStates, beforePoses, phases, transferTargetWaypoints).Length;
                var optimizationClock = System.Diagnostics.Stopwatch.StartNew();
                bool RefineStopped() => Cancelled() || session.Clock.Elapsed.TotalSeconds >= session.DeadlineSeconds || session.Result.Checks >= 100000;
                bool RefineAllowed() => !RefineStopped() && optimizationClock.Elapsed.TotalSeconds < transferOptimizationBudgetSeconds;
                var refinementWorld = TransferSearchWorld(Evaluate, RefineStopped, best.Job.Id);
                refinementWorld.CanRefine = RefineAllowed;
                refinementWorld.MaximumChecks = 10000;
                best.Transfer.Checks = 0;
                if (best.Anchors == null)
                {
                    Status = "Refining transfer: validated shortcuts and bounded corner blends...";
                    var shortcuts = new TrackTransferPlanner(refinementWorld).RefineBidirectional(best.Transfer);
                    while (shortcuts.MoveNext()) yield return shortcuts.Current;
                    var smoothing = new TrackTransferPlanner(refinementWorld).Smooth(best.Transfer, pathAccuracyMillimeters);
                    while (smoothing.MoveNext()) yield return smoothing.Current;
                }
                else TransferLog("Prescribed rounded transit corridor preserved; no joint-space shortcut replaces its TCP geometry.");
                TransferOptimizationTimeSeconds = (float)optimizationClock.Elapsed.TotalSeconds;
                if (Cancelled()) yield break;
                // Publish only after revalidating the complete selected path. Validation
                // failure discards it; no unvalidated smoothing/partial route can execute.
                var validationClock = System.Diagnostics.Stopwatch.StartNew();
                foreach (var section in new[] { best.Transfer.States, best.Job.Final.States })
                for (int i = 0; i < section.Count; i++)
                {
                    if (Cancelled()) yield break;
                    var state = section[i]; var check = EvaluateCoordinatedState(state, singularity);
                    if (!check.Valid || ReferenceEquals(section, best.Transfer.States) && ReachRejects(state.JointAnglesDegrees, true, out _))
                    { Fail("Selected obstacle-aware transfer failed full post-refinement validation; no partial motion."); yield break; }
                    if (validationClock.Elapsed.TotalMilliseconds >= 3) { yield return null; validationClock.Restart(); }
                }
                planningIterations = session.Result.Checks; planningTimeSeconds = (float)session.Clock.Elapsed.TotalSeconds;
                TransferLog($"Selected {best.Name}, E1 {best.Job.Arrival.E1Millimeters:F1} mm, cost {best.Cost:F4}, {best.Improvements} feasible improvements. Sampled convex-pair clearance {(LinearTrackAxis.Finite(best.Margin) ? (best.Margin * 1000).ToString("G9") + " mm" : "unavailable")}; requested clearance unchanged.");
                coordinatedSearch = null; coordinatedChecks.Clear();
                yield return StartCoroutine(PublishStagedTransfer(best.Transfer, best.Job.Final, best.Job.Arrival.E1Millimeters, best.Anchors, best.Name));
            }
            finally
            {
                foreach (var job in jobs) (job.Search as IDisposable)?.Dispose();
                foreach (var job in deferred) (job.Search as IDisposable)?.Dispose();
                if (coordinatedSearch == session) coordinatedSearch = null;
                coordinatedChecks.Clear();
            }
        }
        private TrackTransferPlanner.World TransferSearchWorld(Func<ArmTrackState, CoordinatedStateEvaluation> evaluate, Func<bool> stop, int seed)
            => new TrackTransferPlanner.World
            {
                Evaluate = state => EvaluateTransferReach(state, evaluate), ShouldStop = stop, JointLimits = (Vector2[])rig.AngleLimitsDegrees.Clone(),
                Rejected = CaptureSearchRejection, LimitReached = CaptureSearchLimit, Penalty = state => ReachPenalty(state, true),
                WristZeroOffsetDegrees = rig.ZeroOffsetsDegrees[4], JointStepDegrees = Mathf.Min(.25f, collisionResolutionDegrees),
                MaximumIterations = transferPostureIterations, MaximumChecks = 30000, Seed = randomSeed + seed * 101,
                RefineAutomatically = false, Progress = text => Status = text
            };
        private IEnumerator SearchTransitJob(TransitSearchJob job, List<Bounds> obstacles, Vector3 direction,
            Func<ArmTrackState, CoordinatedStateEvaluation> evaluate, Func<Pose, float, IReadOnlyList<ArmTrackState>> solve,
            Dictionary<(ArmTrackStateKey, Vector3, Quaternion), CoordinatedMoveLPlanner.Result> finalCache,
            Func<bool> outerStop, TransitBest best, Action found, bool corridorsOnly = false)
        {
            bool Stop() => outerStop() || job.NewChecks >= 30000;
            CoordinatedStateEvaluation Evaluate(ArmTrackState state)
            {
                job.Queries++;
                if (!coordinatedChecks.ContainsKey(new ArmTrackStateKey(state, collisionContextRevision))) job.NewChecks++;
                return evaluate(state);
            }
            var tasks = new List<TransitSearchCandidate>();
            try
            {
                job.Phase = "Final MoveL"; diagnosticPhase = job.Phase;
                var poses = new List<Pose> { job.Approach };
                foreach (var marker in intermediateWaypoints ?? Array.Empty<Transform>()) if (marker != null) poses.Add(new Pose(marker.position, marker.rotation));
                poses.Add(new Pose(endTarget.position, endTarget.rotation));
                var finalWorld = new CoordinatedMoveLPlanner.World
                {
                    StartStations = new[] { job.Arrival.E1Millimeters }, MinimumE1 = job.Arrival.E1Millimeters, MaximumE1 = job.Arrival.E1Millimeters, PreferredE1 = job.Arrival.E1Millimeters,
                    BeamWidth = coordinatedBeamWidth, CartesianStepMillimeters = coordinatedCartesianStepMillimeters, TrackStepMillimeters = coordinatedTrackStepMillimeters,
                    JointSweepStepDegrees = Mathf.Min(.25f, collisionResolutionDegrees), ShouldStop = Stop, Evaluate = Evaluate,
                    WristZeroOffsetDegrees = rig.ZeroOffsetsDegrees[4], MaximumChecks = 15000, Rejected = CaptureSearchRejection, LimitReached = CaptureSearchLimit,
                    PositionToleranceMeters = Mathf.Min(.001f, positionClosureTolerance), RotationToleranceDegrees = Mathf.Min(.25f, rotationClosureToleranceDegrees),
                    Penalty = state => ReachPenalty(state, false),
                    Solve = (pose, e1) => pose.position.Equals(job.Approach.position) && Quaternion.Angle(pose.rotation, job.Approach.rotation) < .001f ? new[] { job.Arrival } : solve(pose, e1)
                };
                var finalKey = (new ArmTrackStateKey(job.Arrival, collisionContextRevision), job.Approach.position, job.Approach.rotation);
                if (!finalCache.TryGetValue(finalKey, out job.Final))
                {
                    job.Final = new CoordinatedMoveLPlanner.Result();
                    var finalSearch = new CoordinatedMoveLPlanner(finalWorld).Search(poses, job.Final);
                    try { while (finalSearch.MoveNext()) yield return finalSearch.Current; } finally { (finalSearch as IDisposable)?.Dispose(); }
                    // Only complete, validated tails are reusable. A timeout or a
                    // failed branch is never cached as proof of infeasibility.
                    if (job.Final.Found) finalCache[finalKey] = job.Final;
                }
                if (!job.Final.Found) { job.Phase = "Final MoveL rejected: " + job.Final.Failure; yield break; }
                var startPoseExact = new Pose(startPose.position, startPose.rotation);
                var corridors = ObstacleAwareTransitCorridor.Build(startPoseExact, job.Approach, obstacles, direction, minimumObstacleClearance,
                    corridorsOnly ? transferApproachMillimeters : transferCorridorDropMm, pathAccuracyMillimeters, intermediateRotationRetry);
                void AddCorridor(ObstacleAwareTransitCorridor corridor, bool adaptive)
                {
                    Func<Pose, float> guide = pose => Mathf.Lerp(job.Start.E1Millimeters, job.Arrival.E1Millimeters, corridor.TrackFraction(pose));
                    var candidate = new TransitSearchCandidate { Corridor = corridor, Name = corridor.Name + (adaptive ? " / adaptive E1" : string.Empty), Cartesian = new CoordinatedMoveLPlanner.Result() };
                    var world = new CoordinatedMoveLPlanner.World
                    {
                        StartStations = new[] { job.Start.E1Millimeters }, MinimumE1 = Mathf.Min(job.Start.E1Millimeters, job.Arrival.E1Millimeters), MaximumE1 = Mathf.Max(job.Start.E1Millimeters, job.Arrival.E1Millimeters),
                        PreferredE1 = job.Start.E1Millimeters, StationForPose = adaptive ? null : guide, StationGuide = adaptive ? guide : null,
                        MonotonicE1Direction = Mathf.Sign(job.Arrival.E1Millimeters - job.Start.E1Millimeters),
                        // Match the former lower-corridor fast search. Adaptive
                        // guides retain a wider frontier for alternate E1 choices.
                        WristZeroOffsetDegrees = rig.ZeroOffsetsDegrees[4], BeamWidth = Mathf.Min(adaptive ? 8 : 4, coordinatedBeamWidth),
                        CartesianStepMillimeters = Mathf.Min(20, coordinatedCartesianStepMillimeters), TrackStepMillimeters = Mathf.Max(40, coordinatedTrackStepMillimeters),
                        PositionToleranceMeters = Mathf.Min(.001f, positionClosureTolerance), RotationToleranceDegrees = Mathf.Min(.25f, rotationClosureToleranceDegrees),
                        JointSweepStepDegrees = Mathf.Min(.25f, collisionResolutionDegrees), Evaluate = state => EvaluateTransferReach(state, Evaluate),
                        ShouldStop = Stop, MaximumChecks = adaptive ? 6000 : 4000, Rejected = CaptureSearchRejection, LimitReached = CaptureSearchLimit,
                        Penalty = state => ReachPenalty(state, true),
                        Solve = (pose, e1) => Vector3.Distance(pose.position, startPoseExact.position) < .000001f && Quaternion.Angle(pose.rotation, startPoseExact.rotation) < .001f ? new[] { job.Start } :
                            Vector3.Distance(pose.position, job.Approach.position) < .000001f && Quaternion.Angle(pose.rotation, job.Approach.rotation) < .001f ? new[] { job.Arrival } : solve(pose, e1)
                    };
                    candidate.Search = new CoordinatedMoveLPlanner(world).Search(corridor.Samples, candidate.Cartesian); tasks.Add(candidate);
                }
                // Try the previous lower geometry and outward departure options as
                // well as bounds-ranked lanes. Each gets a resumable slice before
                // any one guide can exhaust its local attempt budget.
                foreach (var corridor in corridors) AddCorridor(corridor,false);
                if (!corridorsOnly) foreach (var corridor in corridors) AddCorridor(corridor,true);
                var free = new TransitSearchCandidate { Name = "Free-posture transfer", Transfer = new TrackTransferPlanner.Result() };
                var freeWorld = TransferSearchWorld(Evaluate, Stop, job.Id); freeWorld.MaximumChecks = 20000;
                var transport = TransportPostures(job.Start);
                // Short local retreats can free the tool before the carriage travels.
                // These are posture proposals only: the complete departure, carriage
                // sweep and arrival still use dense physical validation.
                var outward = Vector3.Cross(Vector3.up, direction).normalized;
                if (!corridorsOnly) foreach (float side in new[] { -1f, 1f })
                foreach (float distance in new[] { .15f, .3f })
                {
                    var retreat = new Pose(startPoseExact.position + outward * (side * distance) - Vector3.up * .1f, startPoseExact.rotation);
                    foreach (var state in solve(retreat, job.Start.E1Millimeters))
                    {
                        if (Mathf.Sin((state[4] + rig.ZeroOffsetsDegrees[4]) * Mathf.Deg2Rad) * Mathf.Sin((job.Start[4] + rig.ZeroOffsetsDegrees[4]) * Mathf.Deg2Rad) <= 0) continue;
                        transport.Add(state.JointAnglesDegrees);
                        break;
                    }
                }
                free.Search = new TrackTransferPlanner(freeWorld).Search(job.Start, job.Arrival, transport, free.Transfer);
                if (!corridorsOnly) tasks.Insert(Math.Min(1,tasks.Count),free);
                int remaining = tasks.Count, cursor = 0, visited = 0, ticks = 0;
                while (remaining > 0 && !Stop())
                {
                    var candidate = tasks[cursor];
                    if (candidate.Done) { cursor = (cursor + 1) % tasks.Count; continue; }
                    // One short visit to every guide, then mix broad exploration,
                    // legacy free-posture search and the strongest Cartesian frontier.
                    // Suspend enumerators instead of restarting their work.
                    cursor = (cursor + 1) % tasks.Count;
                    if (visited == tasks.Count && ++ticks % 3 != 0)
                    {
                        foreach (var option in tasks)
                            if (!option.Done && (option.Cartesian?.ProgressFraction ?? 0) > (candidate.Cartesian?.ProgressFraction ?? 0)) candidate = option;
                        if (!corridorsOnly && (candidate.Cartesian?.ProgressFraction ?? 0) < .01f && !free.Done) candidate = free;
                    }
                    if (candidate.Visits++ == 0) visited++;
                    job.VisitedGuides.Add(candidate.Name);
                    job.Phase = candidate.Name; diagnosticPhase = job.Phase;
                    bool advanced = candidate.Search.MoveNext();
                    if (!advanced)
                    {
                        candidate.Done = true; remaining--; (candidate.Search as IDisposable)?.Dispose();
                        if (candidate.Found)
                        {
                            var transfer = candidate.Transfer ?? new TrackTransferPlanner.Result { Found = true };
                            if (candidate.Cartesian != null) { transfer.States.AddRange(candidate.Cartesian.States); transfer.Poses.AddRange(candidate.Cartesian.Poses); }
                            float margin = float.NaN;
                            var measure = MeasureTransitClearance(transfer.States, value => margin = value, Stop);
                            while (measure.MoveNext()) yield return measure.Current;
                            float peak = avoidOverextension ? stretchScope == StretchScope.TransferOnly ? PeakReach(transfer.States) : Mathf.Max(PeakReach(transfer.States), PeakReach(job.Final.States)) : 0;
                            double cost = ObstacleAwareTransitCorridor.Cost(transfer.States, transfer.Poses, peak, margin, maximumStretchPercent / 100) + ObstacleAwareTransitCorridor.Cost(job.Final.States, job.Final.Poses);
                            if (cost < best.Cost)
                            {
                                best.Job = job; best.Transfer = transfer; best.Anchors = candidate.Corridor?.ProgramAnchors;
                                best.Name = candidate.Name; best.Cost = cost; best.Margin = margin; best.Improvements++; found();
                                TransferLog($"Feasible candidate {candidate.Name}: E1 {job.Arrival.E1Millimeters:F1} mm, connection {job.Id}, cost {cost:F4}; continuing bounded quality comparison.");
                            }
                        }
                        else TransferLog($"Connection {job.Id} / {candidate.Name}: {candidate.Cartesian?.Failure ?? candidate.Transfer?.Failure}");
                    }
                    job.Progress = 0;
                    foreach (var option in tasks) if (!option.Done) job.Progress = Mathf.Max(job.Progress, option.Cartesian?.ProgressFraction ?? 0);
                    yield return null;
                }
            }
            finally { foreach (var task in tasks) (task.Search as IDisposable)?.Dispose(); }
        }
        private IEnumerator MeasureTransitClearance(IReadOnlyList<ArmTrackState> states, Action<float> result, Func<bool> stop)
        {
            float minimum = float.PositiveInfinity;
            for (int sample = 0; sample < 5 && !stop(); sample++)
            {
                using (new StateEvaluation(this, states[(states.Count - 1) * sample / 4]))
                {
                    int pairs = 0;
                    foreach (var body in collisionRobotBodies ?? robotColliders)
                    foreach (var obstacle in obstacleColliders)
                    {
                        if (body == null || obstacle == null || !body.enabled || !obstacle.enabled || body.isTrigger || obstacle.isTrigger || body == obstacle || !obstacle.gameObject.activeInHierarchy || !body.gameObject.activeInHierarchy || LinearTrack.AllowsContact(body, obstacle) || LinearTrack.IsCarriageCollider(body) && LinearTrack.IsCarriageCollider(obstacle)) continue;
                        if (obstacle is MeshCollider mesh && !mesh.convex) continue;
                        if (++pairs > 128) break;
                        float distance = GetColliderSeparation(body, obstacle);
                        if (LinearTrackAxis.Finite(distance)) minimum = Mathf.Min(minimum, distance);
                    }
                }
                yield return null;
            }
            result(LinearTrackAxis.Finite(minimum) ? minimum : float.NaN);
        }
    }
}
