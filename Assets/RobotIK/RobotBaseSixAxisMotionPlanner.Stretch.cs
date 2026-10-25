using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        public enum StretchStrength { Low, Medium, High }
        public enum StretchScope { WholeTrajectory, TransferOnly }
        [Header("Posture and reach (optional)")]
        [SerializeField] private bool avoidOverextension;
        [SerializeField] private bool strictStretchLimit;
        [SerializeField, Range(50, 100)] private float maximumStretchPercent = 84;
        [SerializeField] private StretchStrength stretchStrength = StretchStrength.Medium;
        [SerializeField] private StretchScope stretchScope;
        [SerializeField] private bool allowE1ForStretch = true;
        private ArmStretchGeometry stretchGeometry;
        private ArmStretchPolicy? plannedStretchPolicy;
        public bool AvoidOverextension => avoidOverextension;
        public bool StrictStretchLimit => strictStretchLimit;
        public float MaximumStretchPercent => maximumStretchPercent;
        public StretchStrength ReachPenaltyStrength => stretchStrength;
        public StretchScope ReachScope => stretchScope;
        public bool AllowE1ForStretch => allowE1ForStretch;
        public float PeakPlannedStretchPercent { get; private set; } = float.NaN;
        public string StretchFeedback { get; private set; } = string.Empty;
        private ArmStretchPolicy ReachPolicy => new ArmStretchPolicy(avoidOverextension, strictStretchLimit, allowE1ForStretch,
            stretchScope == StretchScope.TransferOnly, maximumStretchPercent, stretchStrength == StretchStrength.Low ? 2 : stretchStrength == StretchStrength.High ? 30 : 10);
        public bool SetStretchOptions(bool enabled, bool strict, float percent, StretchStrength strength, StretchScope scope, bool allowE1)
        {
            if (IsBusy || (LinearTrack?.IsLocked ?? false) || !LinearTrackAxis.Finite(percent) || percent < 50 || percent > 100 || !Enum.IsDefined(typeof(StretchStrength), strength) || !Enum.IsDefined(typeof(StretchScope), scope)) return false;
            avoidOverextension = enabled; strictStretchLimit = strict; maximumStretchPercent = percent; stretchStrength = strength; stretchScope = scope; allowE1ForStretch = allowE1;
            NotifySceneEdited(); return true;
        }
        private bool PrepareStretchPolicy()
        {
            if (avoidOverextension && (inverseKinematics == null || !inverseKinematics.TryGetArmStretchGeometry(out stretchGeometry))) return Fail("Arm stretch geometry is not calibrated. Check the analytic IK calibration.");
            if (!plannedStretchPolicy.HasValue) plannedStretchPolicy = ReachPolicy;
            return true;
        }
        private bool CheckStretchPolicyGuard()
        {
            if (!IsBusy && path.Count == 0) { plannedStretchPolicy = null; return true; }
            if (!plannedStretchPolicy.HasValue || plannedStretchPolicy.Value.Equals(ReachPolicy)) return true;
            StopMotion(); NotifySceneEdited(); Status = "Posture/reach settings changed. Reset & plan again; stale motion was stopped."; CurrentMotionState = MotionState.Failed;
            CompleteDiagnosticReport(PlanDiagnosticCode.ConfigurationChanged, Status); return false;
        }
        public float MeasureStretchPercent(float[] angles)
        {
            if (!stretchGeometry.Valid && (inverseKinematics == null || !inverseKinematics.TryGetArmStretchGeometry(out stretchGeometry))) return float.NaN;
            return stretchGeometry.Measure(angles) * 100;
        }
        private float ReachPenalty(float[] angles, bool transfer = false) => !ReachPolicy.Applies(transfer) ? 0 : ReachPolicy.Penalty(stretchGeometry.Measure(angles), transfer);
        private float ReachPenalty(ArmTrackState state, bool transfer) => !ReachPolicy.Applies(transfer) ? 0 : ReachPenalty(state.JointAnglesDegrees, transfer);
        private bool ReachRejects(float[] angles, bool transfer, out string reason)
        {
            reason = null; var policy = ReachPolicy;
            if (!policy.Applies(transfer)) return false;
            if (!stretchGeometry.Valid && inverseKinematics != null) inverseKinematics.TryGetArmStretchGeometry(out stretchGeometry);
            float ratio = stretchGeometry.Measure(angles);
            if (float.IsNaN(ratio) || float.IsInfinity(ratio)) { reason = "Arm stretch could not be measured from calibrated geometry."; return true; }
            if (policy.Allows(ratio, transfer)) return false;
            reason = $"Arm stretch {ratio * 100:F1}% exceeds strict limit {maximumStretchPercent:F1}%. Exact pose retained; another permitted station/posture is required."; return true;
        }
        private CoordinatedStateEvaluation EvaluateTransferReach(ArmTrackState state, Func<ArmTrackState, CoordinatedStateEvaluation> evaluate)
        {
            var check = evaluate(state);
            if (!check.Valid || !ReachRejects(state.JointAnglesDegrees, true, out string reason)) return check;
            CaptureSearchRejection(PlanDiagnosticCode.StretchLimit, state, check.Tcp, reason, MeasureStretchPercent(state.JointAnglesDegrees), maximumStretchPercent);
            return new CoordinatedStateEvaluation { Valid = false, Tcp = check.Tcp, Failure = reason };
        }
        private float BestEndpointStretch(Transform target)
        {
            if (target == null || inverseKinematics == null || !inverseKinematics.SolveAll(target.position, target.rotation, out var solutions)) return float.PositiveInfinity;
            float best = float.PositiveInfinity;
            foreach (var q in solutions) best = Mathf.Min(best, stretchGeometry.Measure(q));
            return best;
        }
        private bool ReachAllowsTrackSearch => avoidOverextension && allowE1ForStretch && stretchScope == StretchScope.WholeTrajectory;
        private float ReachStationScore(float e1)
        {
            if (!avoidOverextension || stretchScope != StretchScope.WholeTrajectory) return 0;
            using (new StateEvaluation(this, new ArmTrackState(rig.GetJointAnglesDegrees(), e1)))
            {
                float a = BestEndpointStretch(startPose), b = BestEndpointStretch(endTarget);
                if (float.IsInfinity(a) || float.IsInfinity(b)) return float.PositiveInfinity;
                return Mathf.Max(ReachPolicy.Penalty(a, false), ReachPolicy.Penalty(b, false));
            }
        }
        private float PeakReach(IReadOnlyList<ArmTrackState> states)
        { float peak = 0; foreach (var state in states) peak = Mathf.Max(peak, stretchGeometry.Measure(state.JointAnglesDegrees)); return peak; }
        private void ObserveReach(float[] angles, bool transfer)
        {
            if (!ReachPolicy.Applies(transfer)) return;
            float percent = stretchGeometry.Measure(angles) * 100;
            if (float.IsNaN(PeakPlannedStretchPercent) || percent > PeakPlannedStretchPercent + .01f)
            { PeakPlannedStretchPercent = percent; StretchFeedback = $"Peak arm stretch {percent:F1}% (playback); {(strictStretchLimit ? "strict maximum" : "preferred maximum")} {maximumStretchPercent:F1}%."; }
        }
        private void UpdateStretchFeedback()
        {
            PeakPlannedStretchPercent = float.NaN; StretchFeedback = string.Empty;
            if (!avoidOverextension || path.Count == 0) return;
            float peak = 0; int index = 0, worst = 0, measured = 0;
            if (coordinatedPath.Count > 0)
            {
                foreach (var state in coordinatedPath)
                {
                    bool transfer = stagedPath && stagedPhases[index] != TrackTransferPhase.FinalMoveL;
                    if (ReachPolicy.Applies(transfer)) { measured++; float value = stretchGeometry.Measure(state.JointAnglesDegrees); if (value > peak) { peak = value; worst = index; } }
                    index++;
                }
            }
            else if (stretchScope == StretchScope.WholeTrajectory)
                foreach (var q in path) { measured++; float value = stretchGeometry.Measure(q); if (value > peak) { peak = value; worst = index; } index++; }
            else { StretchFeedback = "Reach scope: transfer only; this plan holds E1 and contains no transfer."; return; }
            if (measured == 0) { StretchFeedback = "Reach scope: transfer only; this plan contains no transfer phase."; return; }
            PeakPlannedStretchPercent = peak * 100;
            StretchFeedback = $"Peak arm stretch {PeakPlannedStretchPercent:F1}% at dense sample {worst + 1}; {(strictStretchLimit ? "strict maximum" : "preferred maximum")} {maximumStretchPercent:F1}%.";
        }
    }
}
