using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        public enum TransferPlannerMethod { Current, Hybrid }
        [Header("Optional hybrid track transfer")]
        [SerializeField] private TransferPlannerMethod transferPlannerMethod;
        [SerializeField, Min(1)] private float hybridTotalSeconds = 60;
        [SerializeField, Min(0)] private float hybridQuickSeconds = 2;
        [SerializeField, Range(100, 100000)] private int hybridSearchTrials = 10000;
        [SerializeField, Min(0)] private float hybridSmoothingSeconds = 1;
        public TransferPlannerMethod SelectedTransferPlanner => transferPlannerMethod;
        public float HybridTotalSeconds => hybridTotalSeconds;
        public float HybridQuickSeconds => hybridQuickSeconds;
        public int HybridSearchTrials => hybridSearchTrials;
        public float HybridSmoothingSeconds => hybridSmoothingSeconds;
        public string HybridStage { get; private set; } = "Ready";
        public string HybridQuickFeedback { get; private set; } = "Pending";
        public string HybridSearchFeedback { get; private set; } = "If needed";
        public string HybridSmoothingFeedback { get; private set; } = "After solution";
        public bool SetTransferPlannerMethod(TransferPlannerMethod method)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !Enum.IsDefined(typeof(TransferPlannerMethod), method)) return false;
            if (transferPlannerMethod == method) return true;
            transferPlannerMethod = method; NotifySceneEdited(); ResetHybridFeedback(); return true;
        }
        public bool SetHybridTransferOptions(float total, float quick, int trials, float smoothing)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(total) || !LinearTrackAxis.Finite(quick) ||
                !LinearTrackAxis.Finite(smoothing) || total < 1 || total > 600 || quick < 0 || smoothing < 0 || quick + smoothing >= total ||
                trials < 100 || trials > 100000) return false;
            hybridTotalSeconds = total; hybridQuickSeconds = quick; hybridSearchTrials = trials; hybridSmoothingSeconds = smoothing;
            NotifySceneEdited(); ResetHybridFeedback(); return true;
        }
        private void ResetHybridFeedback()
        { HybridStage = "Ready"; HybridQuickFeedback = "Pending"; HybridSearchFeedback = "If needed"; HybridSmoothingFeedback = "After solution"; }

        private IEnumerator PlanHybridTransferRoutine()
        {
            ClearCoordinatedPlan(); stagedTransferLog.Clear(); exactStationPlan = false;
            path.Clear(); motionProfile = null; ClearGridGizmoData(); ClearPathVisualization(); InvalidateEndpointChecks(); ResetHybridFeedback();
            planningTimeSeconds = 0; planningIterations = 0; estimatedTravelTimeSeconds = 0;
            if (!ValidateSetup()) yield break;
            if (!LinearTrackAxis.Finite(hybridTotalSeconds) || !LinearTrackAxis.Finite(hybridQuickSeconds) || !LinearTrackAxis.Finite(hybridSmoothingSeconds) ||
                hybridTotalSeconds < 1 || hybridQuickSeconds < 0 || hybridSmoothingSeconds < 0 || hybridQuickSeconds + hybridSmoothingSeconds >= hybridTotalSeconds || hybridSearchTrials < 100)
            { Fail("Invalid hybrid search budgets.", PlanDiagnosticCode.InvalidSetup); yield break; }
            CacheColliders();
            if (robotColliders == null || robotColliders.Length == 0 || obstacleColliders.Length == 0)
            { Fail("Hybrid transfer requires robot and obstacle collision geometry.", PlanDiagnosticCode.InvalidSetup); yield break; }
            inverseKinematics.SolveContinuously = false; InitializeGoalSelection(endTarget.rotation);
            CancelGridSearch(); gridSearchCancellation = new System.Threading.CancellationTokenSource(); var token = gridSearchCancellation.Token;
            coordinatedTargets = new CoordinatedTargets(this);
            var session = new CoordinatedSearchSession { Result = new CoordinatedMoveLPlanner.Result(), DeadlineSeconds = hybridTotalSeconds };
            coordinatedSearch = session;
            var checker = new CartesianLinearPlanner(CreateAlgorithmWorld()); var best = new TransitBest();
            var jobs = new List<TransitSearchJob>();
            var finals = new Dictionary<(ArmTrackStateKey, Vector3, Quaternion), CoordinatedMoveLPlanner.Result>();
            var ik = new Dictionary<(Vector3, Quaternion, float), IReadOnlyList<ArmTrackState>>();
            bool Cancelled() => session.Cancelled || token.IsCancellationRequested;
            bool Expired() => session.Clock.Elapsed.TotalSeconds >= hybridTotalSeconds || session.Result.Checks >= 100000;
            double searchUntil = hybridTotalSeconds - hybridSmoothingSeconds - Math.Min(.5, hybridTotalSeconds * .05);
            bool SearchStop() => Cancelled() || session.Clock.Elapsed.TotalSeconds >= searchUntil || session.Result.Checks >= 90000;
            CoordinatedStateEvaluation Evaluate(ArmTrackState state)
            {
                if (!coordinatedChecks.ContainsKey(new ArmTrackStateKey(state, collisionContextRevision))) session.Result.Checks++;
                planningIterations = session.Result.Checks; planningTimeSeconds = (float)session.Clock.Elapsed.TotalSeconds;
                return EvaluateCoordinatedState(state, checker);
            }
            IReadOnlyList<ArmTrackState> Solve(Pose pose, float e1)
            {
                var key = (pose.position, pose.rotation, e1);
                if (!ik.TryGetValue(key, out var states)) { states = SolveCoordinatedCandidates(pose, e1); if (ik.Count >= 16384) ik.Clear(); ik[key] = states; }
                return states;
            }
            try
            {
                HybridStage = "Endpoint and approach checks";
                var stations = LinearTrackStationCandidates.Build(LinearTrack.Profile, LinearTrack.CurrentE1Millimeters, transferStationSpacingMillimeters, transferMaximumStations);
                var destinations = new List<TransferStation>(); var roots = new List<ArmTrackState>();
                foreach (float station in stations)
                {
                    if (SearchStop()) break;
                    diagnosticPhase = "Hybrid endpoint IK";
                    var ends = ValidTransferIk(new Pose(endTarget.position, endTarget.rotation), station, Evaluate, out _);
                    if (ends.Count > 0) destinations.Add(new TransferStation { E1 = station, EndSolutions = ends });
                    if (roots.Count == 0) roots.AddRange(ValidTransferIk(new Pose(startPose.position, startPose.rotation), station, Evaluate, out _));
                    yield return null;
                }
                StartIkState = roots.Count > 0 ? EndpointIkState.Valid : EndpointIkState.Invalid;
                EndIkState = destinations.Count > 0 ? EndpointIkState.Valid : EndpointIkState.Invalid;
                StartIkSummary = roots.Count > 0 ? $"Start: Valid — {roots.Count} exact collision-free IK at E1 {roots[0].E1Millimeters:F1} mm" : "Start: No validated exact IK at sampled stations";
                EndIkSummary = destinations.Count > 0 ? $"End: Valid — exact collision-free IK at {destinations.Count} candidate track stations" : "End: No validated exact IK at sampled stations";
                if (Cancelled()) yield break;
                if (roots.Count == 0 || destinations.Count == 0) { Fail("Hybrid endpoint validation failed. " + StartIkSummary + "; " + EndIkSummary); yield break; }
                var endPose = new Pose(endTarget.position, endTarget.rotation);
                var approaches = new List<Pose>();
                foreach (var axis in new[] { Vector3.down, -(endPose.rotation * Vector3.forward), Vector3.up, endPose.rotation * Vector3.forward })
                    approaches.Add(new Pose(endPose.position + axis * (transferApproachMillimeters * .001f), endPose.rotation));
                var approachIk = new Dictionary<(int, float), List<ArmTrackState>>();
                // Station/root diversity first; later variants introduce deeper and
                // tool-axis approaches. No endpoint or authored pose is displaced.
                for (int variant = 0; variant < Math.Min(4, roots.Count) * Math.Min(4, approaches.Count) && jobs.Count < 48 && !SearchStop(); variant++)
                foreach (var station in destinations)
                {
                    if (SearchStop() || jobs.Count >= 48) break;
                    int rootIndex = variant % Math.Min(4, roots.Count), approachIndex = variant / Math.Min(4, roots.Count);
                    if (!approachIk.TryGetValue((approachIndex, station.E1), out var arrivals))
                    { diagnosticPhase = "Hybrid approach IK"; arrivals = ValidTransferIk(approaches[approachIndex], station.E1, Evaluate, out _); approachIk[(approachIndex, station.E1)] = arrivals; }
                    var options = new List<ArmTrackState>(arrivals); var root = roots[rootIndex];
                    options.Sort((a,b) => JointDifference(a,root.JointAnglesDegrees).CompareTo(JointDifference(b,root.JointAnglesDegrees)));
                    foreach (var arrival in options)
                    {
                        if (Mathf.Sin((root[4]+rig.ZeroOffsetsDegrees[4])*Mathf.Deg2Rad)*Mathf.Sin((arrival[4]+rig.ZeroOffsetsDegrees[4])*Mathf.Deg2Rad)<=0) continue;
                        jobs.Add(new TransitSearchJob { Id = jobs.Count + 1, Start = root, Arrival = arrival, Approach = approaches[approachIndex] }); break;
                    }
                    yield return null;
                }
                TransferLog($"Hybrid: {jobs.Count} connections; total budget {hybridTotalSeconds:G4} s including quick routes, general search, smoothing and validation. Clearance unchanged.");
                var guides = TransitGuideObstacles(); var direction = CalibratedTrackDirection();
                var quickClock = System.Diagnostics.Stopwatch.StartNew(); HybridStage = "Quick corridors";
                bool QuickStop() => SearchStop() || quickClock.Elapsed.TotalSeconds >= hybridQuickSeconds || best.Transfer != null;
                foreach (var job in jobs) job.Search = SearchTransitJob(job,guides,direction,Evaluate,Solve,finals,QuickStop,best,()=>{},true);
                var frame = System.Diagnostics.Stopwatch.StartNew();int quickVisits=0,quickTurns=0;
                foreach(var job in jobs)job.Visited=false;
                while (!QuickStop())
                {
                    bool active = false;
                    foreach (var scheduled in jobs)
                    {
                        if (QuickStop()) break;
                        var job=scheduled;
                        if (job.Done) continue;
                        if(quickVisits==jobs.Count&&++quickTurns%3!=0)foreach(var option in jobs)if(!option.Done&&option.Progress>job.Progress)job=option;
                        active = true; diagnosticAttempt=job.Id;
                        if (!job.Search.MoveNext()) { job.Done=true; (job.Search as IDisposable)?.Dispose(); }
                        if(!job.Visited){job.Visited=true;quickVisits++;}
                        Status=$"Hybrid · quick corridors · connection {job.Id}/{jobs.Count}";
                        HybridQuickFeedback=$"Searching · {quickClock.Elapsed.TotalSeconds:F1} s";
                        if(frame.Elapsed.TotalMilliseconds>=3){yield return null;frame.Restart();}
                    }
                    if(!active)break;
                }
                foreach(var job in jobs){(job.Search as IDisposable)?.Dispose();job.Search=null;job.Done=false;job.NewChecks=0;}
                HybridQuickFeedback=hybridQuickSeconds==0?"Skipped":best.Transfer!=null?$"Route found · {quickClock.Elapsed.TotalSeconds:F2} s":$"No complete route · {quickClock.Elapsed.TotalSeconds:F2} s";
                TransferLog("Hybrid quick corridors: "+HybridQuickFeedback);
                if(best.Transfer==null&&!SearchStop())
                {
                    HybridStage="Arm + E1 search";var generalClock=System.Diagnostics.Stopwatch.StartNew();
                    foreach(var job in jobs)job.Search=SearchHybridConnection(job,Evaluate,Solve,finals,SearchStop,best);
                    int remaining=jobs.Count;
                    while(remaining>0&&!SearchStop()&&best.Transfer==null)
                    foreach(var scheduled in jobs)
                    {
                        if(SearchStop()||best.Transfer!=null)break;
                        var job=scheduled;
                        if(job.Done)continue;
                        diagnosticAttempt=job.Id;diagnosticPhase="Hybrid arm + E1 RRT-Connect";
                        if(!job.Search.MoveNext()){job.Done=true;remaining--; (job.Search as IDisposable)?.Dispose();}
                        HybridSearchFeedback=$"Searching · {generalClock.Elapsed.TotalSeconds:F1} s";
                        if(frame.Elapsed.TotalMilliseconds>=3){yield return null;frame.Restart();}
                    }
                    HybridSearchFeedback=best.Transfer!=null?$"Route found · {generalClock.Elapsed.TotalSeconds:F2} s":$"No complete route · {generalClock.Elapsed.TotalSeconds:F2} s";
                    TransferLog("Hybrid RRT-Connect: "+HybridSearchFeedback);
                }
                else HybridSearchFeedback=best.Transfer!=null?"Skipped · quick route found":"Not started · search budget reached";
                foreach(var job in jobs){(job.Search as IDisposable)?.Dispose();job.Search=null;}
                if(Cancelled())yield break;
                if(best.Transfer==null)
                { HybridStage="No complete route";Fail("Hybrid found no complete validated arm/track connection within its bounds. No partial motion.\n"+string.Join("\n",stagedTransferLog),session.Result.Checks>=90000?PlanDiagnosticCode.CheckLimit:session.Clock.Elapsed.TotalSeconds>=searchUntil?PlanDiagnosticCode.TimeLimit:PlanDiagnosticCode.NoCompleteRoute);yield break; }
                var before=new List<ArmTrackState>(best.Transfer.States);var poses=new List<Pose>(best.Transfer.Poses);var phases=new List<int>();
                for(int i=0;i<before.Count;i++)phases.Add(i==0?0:(int)TransferPhase(before[i],before[0],best.Job.Arrival.E1Millimeters));
                for(int i=1;i<best.Job.Final.States.Count;i++){before.Add(best.Job.Final.States[i]);poses.Add(best.Job.Final.Poses[i]);phases.Add(4);}
                PreserveAuthoredWaypoints(poses,phases);TransferWaypointsBeforeOptimization=TrackTransferWaypoints.SelectBeforeOptimization(before,poses,phases,transferTargetWaypoints).Length;
                HybridStage="Validated smoothing";diagnosticPhase="Hybrid validated smoothing";
                var smoothing=System.Diagnostics.Stopwatch.StartNew();
                bool RefineStop()=>Cancelled()||Expired();
                var world=TransferSearchWorld(Evaluate,RefineStop,best.Job.Id);world.MaximumChecks=10000;world.CanRefine=()=>!RefineStop()&&smoothing.Elapsed.TotalSeconds<hybridSmoothingSeconds;
                best.Transfer.Checks=0;
                if(best.Anchors==null&&hybridSmoothingSeconds>0)
                {
                    var shortcut=new TrackTransferPlanner(world).RefineBidirectional(best.Transfer);
                    try{while(shortcut.MoveNext())yield return shortcut.Current;}finally{(shortcut as IDisposable)?.Dispose();}
                    var blend=new TrackTransferPlanner(world).Smooth(best.Transfer,pathAccuracyMillimeters);
                    try{while(blend.MoveNext())yield return blend.Current;}finally{(blend as IDisposable)?.Dispose();}
                }
                TransferOptimizationTimeSeconds=(float)smoothing.Elapsed.TotalSeconds;
                HybridSmoothingFeedback=best.Anchors!=null?"Rounded corridor retained":hybridSmoothingSeconds==0?"Skipped":$"Validated · {smoothing.Elapsed.TotalSeconds:F2} s";
                HybridStage="Final validation";
                foreach(var section in new[]{best.Transfer.States,best.Job.Final.States})foreach(var state in section)
                {
                    if(Cancelled())yield break;
                    if(Expired()){Fail("Hybrid total budget reached during final validation. No partial trajectory will execute.",PlanDiagnosticCode.TimeLimit);yield break;}
                    var check=EvaluateCoordinatedState(state,checker);
                    if(!check.Valid||ReferenceEquals(section,best.Transfer.States)&&ReachRejects(state.JointAnglesDegrees,true,out _))
                    {Fail("Hybrid post-refinement validation failed; no partial motion.");yield break;}
                    if(frame.Elapsed.TotalMilliseconds>=3){yield return null;frame.Restart();}
                }
                planningIterations=session.Result.Checks;planningTimeSeconds=(float)session.Clock.Elapsed.TotalSeconds;
                TransferLog($"Hybrid selected {best.Name}; {planningTimeSeconds:F2} s, {planningIterations} new evaluations. Exact Start/End; no wrist flips. "+HybridSmoothingFeedback);
                HybridStage="Route validated";coordinatedChecks.Clear();
                yield return StartCoroutine(PublishStagedTransfer(best.Transfer,best.Job.Final,best.Job.Arrival.E1Millimeters,best.Anchors,best.Name));
            }
            finally
            {
                foreach(var job in jobs)(job.Search as IDisposable)?.Dispose();
                if(coordinatedSearch==session)coordinatedSearch=null;
                coordinatedChecks.Clear();
            }
        }
        private IEnumerator SearchHybridConnection(TransitSearchJob job,Func<ArmTrackState,CoordinatedStateEvaluation> evaluate,
            Func<Pose,float,IReadOnlyList<ArmTrackState>> solve,Dictionary<(ArmTrackStateKey,Vector3,Quaternion),CoordinatedMoveLPlanner.Result> finals,
            Func<bool> stop,TransitBest best)
        {
            var key=(new ArmTrackStateKey(job.Arrival,collisionContextRevision),job.Approach.position,job.Approach.rotation);
            if(!finals.TryGetValue(key,out job.Final))
            {
                var targets=new List<Pose>{job.Approach};foreach(var marker in intermediateWaypoints??Array.Empty<Transform>())if(marker!=null)targets.Add(new Pose(marker.position,marker.rotation));targets.Add(new Pose(endTarget.position,endTarget.rotation));
                var finalWorld=new CoordinatedMoveLPlanner.World{StartStations=new[]{job.Arrival.E1Millimeters},MinimumE1=job.Arrival.E1Millimeters,MaximumE1=job.Arrival.E1Millimeters,
                    PreferredE1=job.Arrival.E1Millimeters,ShouldStop=stop,Evaluate=evaluate,MaximumChecks=15000,BeamWidth=coordinatedBeamWidth,
                    CartesianStepMillimeters=coordinatedCartesianStepMillimeters,TrackStepMillimeters=coordinatedTrackStepMillimeters,
                    JointSweepStepDegrees=Mathf.Min(.25f,collisionResolutionDegrees),WristZeroOffsetDegrees=rig.ZeroOffsetsDegrees[4],
                    PositionToleranceMeters=Mathf.Min(.001f,positionClosureTolerance),RotationToleranceDegrees=Mathf.Min(.25f,rotationClosureToleranceDegrees),
                    Rejected=CaptureSearchRejection,LimitReached=CaptureSearchLimit,Penalty=state=>ReachPenalty(state,false),
                    Solve=(pose,e1)=>pose.position.Equals(job.Approach.position)&&Quaternion.Angle(pose.rotation,job.Approach.rotation)<.001f?new[]{job.Arrival}:solve(pose,e1)};
                job.Final=new CoordinatedMoveLPlanner.Result();var tail=new CoordinatedMoveLPlanner(finalWorld).Search(targets,job.Final);
                try{while(tail.MoveNext())yield return tail.Current;}finally{(tail as IDisposable)?.Dispose();}
                if(!job.Final.Found){TransferLog($"Hybrid connection {job.Id}: final MoveL rejected: {job.Final.Failure}");yield break;}
                finals[key]=job.Final;
            }
            var world=TransferSearchWorld(evaluate,stop,job.Id);world.MaximumIterations=hybridSearchTrials;world.MaximumChecks=30000;
            var planner=new ArmTrackRrtConnectPlanner(world,LinearTrack.MinimumMillimeters,LinearTrack.MaximumMillimeters);
            var transfer=new TrackTransferPlanner.Result();var routine=planner.Search(job.Start,job.Arrival,TransportPostures(job.Start),transfer);
            try{while(routine.MoveNext()){Status=$"Hybrid · arm + E1 search · connection {job.Id} · {planner.StartTreeNodes}/{planner.GoalTreeNodes} nodes";yield return routine.Current;}}
            finally{(routine as IDisposable)?.Dispose();}
            TransferLog($"Hybrid connection {job.Id}: RRT-Connect {planner.StartTreeNodes}/{planner.GoalTreeNodes} nodes, {transfer.Checks} queries; {(transfer.Found?"complete":transfer.Failure)}");
            if(transfer.Found&&!stop()){best.Job=job;best.Transfer=transfer;best.Name="Hybrid RRT-Connect";best.Anchors=null;}
        }
    }
}
