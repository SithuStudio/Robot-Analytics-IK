using System;
using System.Collections;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        private bool endpointReachabilityGateActive;
        private bool trackHeldForReachableEndpoints;
        private string trackReachabilitySummary = string.Empty;
        private Transform reachabilityStart, reachabilityEnd;
        private Pose reachabilityStartPose, reachabilityEndPose;
        public bool IsCheckingEndpointReachability => endpointReachabilityGateActive;
        public bool TrackHeldForReachableEndpoints => trackHeldForReachableEndpoints;
        public string TrackReachabilitySummary => trackReachabilitySummary;
        private bool TrackReachabilityGateApplicable => StagedTransferApplicable || CoordinatedApplicable || AutomaticStationApplicable;

        private void ClearTrackReachabilityDecision()
        {
            endpointReachabilityGateActive = false; trackHeldForReachableEndpoints = false;
            trackReachabilitySummary = string.Empty; reachabilityStart = reachabilityEnd = null;
        }

        // Deliberately geometric: a reachable but colliding/clearance-invalid pose
        // does not authorize track motion. Those checks remain mandatory in the
        // selected arm-only or track-enabled full planner.
        private int CountExactEndpointIk(Transform target)
        {
            var physical = CurrentCombinedState;
            using (new StateEvaluation(this, physical))
            {
                if (!inverseKinematics.SolveAll(target.position, target.rotation, out var solutions)) return 0;
                int count = 0;
                foreach (var q in solutions)
                {
                    if (ValidateCombinedState(new ArmTrackState(q, physical.E1Millimeters)) != null || !rig.SetJointAngles(q)) continue;
                    var tcp = GetCurrentTcp();
                    if (Vector3.Distance(tcp.position, target.position) <= Mathf.Min(.001f, positionClosureTolerance) &&
                        Quaternion.Angle(tcp.rotation, target.rotation) <= Mathf.Min(.25f, rotationClosureToleranceDegrees)) count++;
                }
                return count;
            }
        }

        private bool CheckTrackReachabilityGuard()
        {
            if (!endpointReachabilityGateActive) return true;
            bool same = reachabilityStart != null && reachabilityEnd != null && startPose == reachabilityStart && endTarget == reachabilityEnd &&
                reachabilityStartPose.position.Equals(startPose.position) && reachabilityEndPose.position.Equals(endTarget.position) &&
                Quaternion.Angle(reachabilityStartPose.rotation, startPose.rotation) <= .001f && Quaternion.Angle(reachabilityEndPose.rotation, endTarget.rotation) <= .001f;
            if (same) return true;
            StopMotion(); NotifySceneEdited();
            Status = "Start or End changed after the current-E1 IK decision. Reset & plan again; no stale route will execute.";
            CurrentMotionState = MotionState.Failed; CompleteDiagnosticReport(PlanDiagnosticCode.ConfigurationChanged, Status); return false;
        }

        private IEnumerator ResetWithTrackReachabilityRoutine()
        {
            path.Clear(); motionProfile = null; lastPlanSummary = string.Empty;
            estimatedTravelTimeSeconds = 0; planningIterations = 0; planningTimeSeconds = 0;
            ClearGridGizmoData(); ClearPathVisualization(); InvalidateEndpointChecks();
            if (!ValidateSetup()) yield break;
            inverseKinematics.SolveContinuously = false;
            reachabilityStart = startPose; reachabilityEnd = endTarget;
            reachabilityStartPose = new Pose(startPose.position, startPose.rotation);
            reachabilityEndPose = new Pose(endTarget.position, endTarget.rotation);
            endpointReachabilityGateActive = true;
            exactStationPlan = true;
            float e1 = LinearTrack.CurrentE1Millimeters;
            try
            {
                Status = $"Checking exact Start and End IK at current E1 {e1:F1} mm before considering track movement...";
                diagnosticPhase = "Current-E1 Start IK";
                int startCount = CountExactEndpointIk(startPose);
                diagnosticPhase = "Current-E1 End IK";
                int endCount = CountExactEndpointIk(endTarget);
                trackHeldForReachableEndpoints = startCount > 0 && endCount > 0;
                bool reachRequestsTrack = trackHeldForReachableEndpoints && ReachAllowsTrackSearch &&
                    (BestEndpointStretch(startPose) > ReachPolicy.Limit || BestEndpointStretch(endTarget) > ReachPolicy.Limit);
                if (reachRequestsTrack) trackHeldForReachableEndpoints = false;
                if (trackHeldForReachableEndpoints)
                {
                    trackReachabilitySummary = $"Arm first: Start {startCount} IK, End {endCount} IK at E1 {e1:F1} mm. Track stays fixed; full arm collision and continuity checks still apply.";
                    Status = trackReachabilitySummary;
                    // Intentionally no fallback after an arm-only route failure.
                    yield return StartCoroutine(ResetToStartAndPlanRoutine());
                }
                else
                {
                    trackReachabilitySummary = $"Track search allowed: {(startCount == 0 && endCount == 0 ? "Start and End have" : startCount == 0 ? "Start has" : "End has")} no exact IK at current E1 {e1:F1} mm. Start {startCount} IK, End {endCount} IK.";
                    if (reachRequestsTrack) trackReachabilitySummary = $"Posture exception: exact IK exists at E1 {e1:F1} mm, but endpoint stretch exceeds {maximumStretchPercent:F1}%. E1 adjustment was explicitly enabled; exact Start/End remain required.";
                    Status = trackReachabilitySummary;
                    exactStationPlan = false;
                    yield return StartCoroutine(StagedTransferApplicable ? PlanStagedTransferRoutine() : CoordinatedApplicable ? PlanCoordinatedMoveLRoutine() : SelectStationAndPlanRoutine());
                }
            }
            finally { endpointReachabilityGateActive = false; }
            if (CurrentMotionState == MotionState.Failed)
            {
                path.Clear(); motionProfile = null; ClearPathVisualization();
                Status += trackHeldForReachableEndpoints
                    ? " Track stayed fixed because both exact endpoints have IK; collision or route failure does not enable track fallback."
                    : " " + trackReachabilitySummary;
            }
            else if (CurrentMotionState == MotionState.Complete)
            {
                lastPlanSummary += " " + trackReachabilitySummary;
                Status = "Motion complete. " + lastPlanSummary;
            }
        }
    }
}
