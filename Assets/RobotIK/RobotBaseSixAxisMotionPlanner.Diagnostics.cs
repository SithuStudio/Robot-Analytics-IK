using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class RobotBaseSixAxisMotionPlanner
    {
        [Header("Failure inspection")]
        [SerializeField] private bool highlightFailedPosition = true;
        public bool HighlightFailedPosition { get => highlightFailedPosition; set { highlightFailedPosition = value; RefreshFailureHighlight(); } }
        public PlanDiagnosticReport LastPlanReport { get; private set; }
        public PlanDiagnosticEvent SelectedFailure { get; private set; }
        private string diagnosticPhase = "Planning";
        private int diagnosticAttempt;
        private void CaptureSearchLimit(PlanDiagnosticCode code, string message)
        {
            if (LastPlanReport?.Outcome == PlanReportOutcome.Running)
                LastPlanReport.Add(new PlanDiagnosticEvent { Code = code, Phase = diagnosticPhase, Attempt = diagnosticAttempt,
                    Sample = LastPlanReport.ConfigurationQueries, Message = message, EvidenceKind = "Local search bound; no spatial position is implied." });
        }
        private void CaptureInvalidState(ArmTrackState state, string message)
        {
            if (LastPlanReport?.Outcome != PlanReportOutcome.Running || state == null) return;
            var track = LinearTrack;
            var e = new PlanDiagnosticEvent { Code = PlanDiagnosticCode.InvalidSetup, Phase = diagnosticPhase, Attempt = diagnosticAttempt, Message = message,
                HasJoints = true, Joints = state.JointAnglesDegrees, HasE1 = state.HasTrackAxis, E1 = state.E1Millimeters,
                EvidenceKind = "Configuration not applied. No FK/TCP position captured." };
            if (state.HasTrackAxis && track != null && (state.E1Millimeters < track.MinimumMillimeters || state.E1Millimeters > track.MaximumMillimeters))
            { e.Code = PlanDiagnosticCode.TrackLimit; e.HasMeasurement = e.HasRange = true; e.Measured = state.E1Millimeters; e.Minimum = track.MinimumMillimeters; e.Maximum = track.MaximumMillimeters; e.Units = "mm"; }
            else if (rig != null) for (int i = 0; i < 6; i++)
            {
                var limit = rig.AngleLimitsDegrees[i];
                if (state[i] >= limit.x && state[i] <= limit.y) continue;
                e.Code = PlanDiagnosticCode.JointLimit; e.Joint = i; e.HasMeasurement = e.HasRange = true; e.Measured = state[i]; e.Minimum = limit.x; e.Maximum = limit.y; e.Units = "degrees"; break;
            }
            LastPlanReport.Add(e);
        }
        private void CaptureSearchRejection(PlanDiagnosticCode code, ArmTrackState state, Pose? pose, string message, float? measured, float? required)
        {
            if (state == null) return;
            if (pose.HasValue && (code == PlanDiagnosticCode.JointContinuity || code == PlanDiagnosticCode.WristBranch))
            {
                if (LastPlanReport?.Outcome == PlanReportOutcome.Running)
                    LastPlanReport.Add(new PlanDiagnosticEvent { Code = code, Phase = diagnosticPhase, Attempt = diagnosticAttempt,
                        Sample = LastPlanReport.ConfigurationQueries, Message = message, HasE1 = state.HasTrackAxis, E1 = state.E1Millimeters,
                        HasJoints = true, Joints = state.JointAnglesDegrees, HasTcp = true, TcpPosition = pose.Value.position, TcpRotation = pose.Value.rotation,
                        HasMeasurement = measured.HasValue, Measured = measured ?? 0, HasLimit = required.HasValue, Required = required ?? 0, Units = "degrees",
                        EvidenceKind = "Requested TCP / analytical candidate. Transition rejected before FK/swept validation; base pose not captured." });
                return;
            }
            // Evaluate FK inside a restoring scope; never leave the live robot at a rejected candidate.
            if (ValidateCombinedState(state) == null)
            {
                using (new StateEvaluation(this, state))
                { var tcp = GetCurrentTcp(); CaptureDiagnostic(code, message, state.JointAnglesDegrees, new Pose(tcp.position, tcp.rotation), measured, required, code == PlanDiagnosticCode.StretchLimit ? "%" : "degrees"); }
            }
            else
            {
                var e = new PlanDiagnosticEvent { Code = code, Phase = diagnosticPhase, Attempt = diagnosticAttempt, Message = message,
                    HasE1 = state.HasTrackAxis, E1 = state.E1Millimeters, HasJoints = true, Joints = state.JointAnglesDegrees,
                    HasTcp = pose.HasValue, TcpPosition = pose?.position ?? Vector3.zero, TcpRotation = pose?.rotation ?? Quaternion.identity };
                if (LastPlanReport?.Outcome == PlanReportOutcome.Running) LastPlanReport.Add(e);
            }
        }
        private Transform failureHighlightRoot, failureGhostRoot;
        private TextMesh failureHighlightText;
        private Material failureHighlightMaterial, failureGhostMaterial;
        private static string ObjectPath(Transform t) => t == null ? string.Empty : t.parent == null ? t.name : ObjectPath(t.parent) + "/" + t.name;
        private void BeginDiagnosticReport()
        {
            ClearFailureVisuals(); SelectedFailure = null;
            diagnosticPhase = "Setup"; diagnosticAttempt = 0;
            var limits = rig != null ? rig.AngleLimitsDegrees : null;
            var snapshot = new System.Text.StringBuilder();
            snapshot.Append($"algorithm={plannerAlgorithm}; seed={randomSeed}; clearance_mm={minimumObstacleClearance * 1000:G9}; singularity_margin={CreateAlgorithmWorld().LinearMinimumSingularValueRatio:G9}; ");
            if (startPose != null) snapshot.Append($"Start world meters={startPose.position.ToString("G9")}, quaternion={startPose.rotation.ToString("G9")}; ");
            if (endTarget != null) snapshot.Append($"End world meters={endTarget.position.ToString("G9")}, quaternion={endTarget.rotation.ToString("G9")}; ");
            snapshot.Append($"transfer_spacing_mm={transferStationSpacingMillimeters:G9}; max_stations={transferMaximumStations}; approach_mm={transferApproachMillimeters:G9}; posture_iterations={transferPostureIterations}; budget_s={coordinatedSearchSeconds:G9}; beam={coordinatedBeamWidth}; Cartesian_step_mm={coordinatedCartesianStepMillimeters:G9}; track_step_mm={coordinatedTrackStepMillimeters:G9}; ");
            snapshot.Append($"reach={avoidOverextension}/{strictStretchLimit}/{maximumStretchPercent:G9}/{stretchScope}; corridor={preferLowerTransferCorridor}/{requireLowerTransferCorridor}/{transferCorridorDropMm:G9}; ");
            snapshot.Append($"obstacle_aware_transfer={obstacleAwareTransfer}; ");
            snapshot.Append($"transfer_method={transferPlannerMethod}; hybrid_budget_s={hybridTotalSeconds:R}; hybrid_quick_s={hybridQuickSeconds:R}; hybrid_trials={hybridSearchTrials}; hybrid_smoothing_s={hybridSmoothingSeconds:R}; ");
            if (limits != null) for (int i = 0; i < limits.Length; i++) snapshot.Append($"J{i + 1}=[{limits[i].x:G9},{limits[i].y:G9}]; ");
            var track = LinearTrack;
            if (track != null) snapshot.Append($"E1_range_mm=[{track.MinimumMillimeters:G9},{track.MaximumMillimeters:G9}]; profile={JsonUtility.ToJson(track.Profile)}; ");
            LastPlanReport = new PlanDiagnosticReport { RunId = Guid.NewGuid().ToString("N"), StartedUtc = DateTime.UtcNow.ToString("O"), Planner = plannerAlgorithm.ToString(), Outcome = PlanReportOutcome.Running, SettingsSnapshot = snapshot.ToString() };
            if (inverseKinematics != null) inverseKinematics.DiagnosticRejected = e =>
            {
                if (LastPlanReport?.Outcome != PlanReportOutcome.Running) return;
                e.Phase = diagnosticPhase; e.Attempt = diagnosticAttempt; e.Sample = LastPlanReport.ConfigurationQueries;
                e.HasE1 = LinearTrack != null && LinearTrack.TrackEnabled; e.E1 = LinearTrack != null ? LinearTrack.CurrentE1Millimeters : 0;
                e.HasBase = e.HasJoints && rig != null;
                if (rig != null) { e.BasePosition = rig.transform.position; e.BaseRotation = rig.transform.rotation; e.BaseScale = rig.transform.lossyScale; }
                LastPlanReport.Add(e);
            };
        }
        private PlanDiagnosticEvent CaptureDiagnostic(PlanDiagnosticCode code, string message, float[] angles = null, Pose? pose = null,
            float? measured = null, float? required = null, string units = null, Collider robot = null, Collider obstacle = null, string evidence = null)
        {
            if (LastPlanReport == null || LastPlanReport.Outcome != PlanReportOutcome.Running) return null;
            var tcp = pose ?? (rig != null && rig.IsReady ? new Pose(GetCurrentTcp().position, GetCurrentTcp().rotation) : (Pose?)null);
            var track = LinearTrack;
            var e = new PlanDiagnosticEvent { Code = code, Phase = diagnosticPhase, Attempt = diagnosticAttempt,
                Sample = LastPlanReport.ConfigurationQueries, Message = message, Units = units, EvidenceKind = evidence,
                HasJoints = angles != null, Joints = angles == null ? null : (float[])angles.Clone(), HasTcp = tcp.HasValue,
                TcpPosition = tcp?.position ?? Vector3.zero, TcpRotation = tcp?.rotation ?? Quaternion.identity,
                HasE1 = track != null && track.TrackEnabled, E1 = track != null ? track.CurrentE1Millimeters : 0,
                HasBase = rig != null, BasePosition = rig != null ? rig.transform.position : Vector3.zero, BaseRotation = rig != null ? rig.transform.rotation : Quaternion.identity,
                BaseScale = rig != null ? rig.transform.lossyScale : Vector3.one,
                HasMeasurement = measured.HasValue && LinearTrackAxis.Finite(measured.Value), Measured = measured.HasValue && LinearTrackAxis.Finite(measured.Value) ? measured.Value : 0,
                HasLimit = required.HasValue && LinearTrackAxis.Finite(required.Value), Required = required.HasValue && LinearTrackAxis.Finite(required.Value) ? required.Value : 0,
                RobotObject = ObjectPath(robot != null ? robot.transform : null), RobotId = robot != null ? robot.gameObject.GetInstanceID() : 0,
                ObstacleObject = ObjectPath(obstacle != null ? obstacle.transform : null), ObstacleId = obstacle != null ? obstacle.gameObject.GetInstanceID() : 0 };
            LastPlanReport.Add(e); return e;
        }
        private void CompleteDiagnosticReport(PlanDiagnosticCode code, string message, PlanReportOutcome? outcome = null)
        {
            var report = LastPlanReport; if (report == null) return;
            report.StopCode = code; report.Summary = message; report.StartSummary = StartIkSummary; report.EndSummary = EndIkSummary;
            report.Outcome = outcome ?? (code == PlanDiagnosticCode.CheckLimit || code == PlanDiagnosticCode.TimeLimit || code == PlanDiagnosticCode.AttemptLimit || code == PlanDiagnosticCode.IterationLimit || code == PlanDiagnosticCode.NoCompleteRoute ? PlanReportOutcome.SearchIncomplete : PlanReportOutcome.Failed);
            report.Revision++;
            SelectedFailure = null;
            for (int i = report.Events.Count - 1; i >= 0; i--) if (report.Events[i].HasTcp) { SelectedFailure = report.Events[i]; break; }
            RefreshFailureHighlight();
        }
        public bool SelectFailure(int id)
        {
            var e = LastPlanReport?.Events.Find(x => x.Id == id); if (e == null) return false;
            SelectedFailure = e; ClearFailureGhost(); RefreshFailureHighlight(); return true;
        }
        public string DiagnosticReportText => LastPlanReport?.ToText() ?? "No planning report available.";
        public string DiagnosticReportJson => LastPlanReport == null ? "{}" : JsonUtility.ToJson(LastPlanReport, true);
        public string SaveDiagnosticReport(bool json)
        {
            if (LastPlanReport == null) return null;
            string file = "Robot-plan-" + LastPlanReport.RunId + (json ? ".json" : ".txt");
#if UNITY_EDITOR
            string path = UnityEditor.EditorUtility.SaveFilePanel("Save planner report", "", file, json ? "json" : "txt");
            if (string.IsNullOrEmpty(path)) return null;
#else
            string path = Path.Combine(Application.persistentDataPath, file);
#endif
            File.WriteAllText(path, json ? DiagnosticReportJson : DiagnosticReportText); return path;
        }
        private Material FailureMaterial(ref Material material, Color color)
        {
            if (material != null) return material;
            var shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color"); if (shader == null) return null;
            material = new Material(shader) { hideFlags = HideFlags.DontSave, name = "Planner diagnostic visual" };
            if (material.HasProperty("_UnlitColor")) material.SetColor("_UnlitColor", color);
            if (material.HasProperty("_Color")) material.SetColor("_Color", color);
            if (color.a < 1 && material.HasProperty("_SurfaceType")) material.SetFloat("_SurfaceType", 1);
            if (shader.name == "HDRP/Unlit") UnityEngine.Rendering.HighDefinition.HDMaterial.ValidateMaterial(material);
            return material;
        }
        private void RefreshFailureHighlight()
        {
            bool visible = !IsBusy && highlightFailedPosition && SelectedFailure != null && SelectedFailure.HasTcp && LastPlanReport != null && !LastPlanReport.Stale && LastPlanReport.Outcome != PlanReportOutcome.Running && LastPlanReport.Outcome != PlanReportOutcome.Complete;
            if (!visible) { if (failureHighlightRoot != null) failureHighlightRoot.gameObject.SetActive(false); return; }
            if (!Application.isPlaying) return;
            if (failureHighlightRoot == null)
            {
                failureHighlightRoot = new GameObject("Planner Failed Position").transform; failureHighlightRoot.SetParent(transform, false);
                var marker = GameObject.CreatePrimitive(PrimitiveType.Sphere); marker.name = "Rejected TCP marker"; marker.layer = 2;
                if (marker.TryGetComponent<Collider>(out var collider)) { collider.enabled = false; Destroy(collider); }
                marker.transform.SetParent(failureHighlightRoot, false); marker.transform.localScale = Vector3.one * .045f;
                marker.GetComponent<Renderer>().sharedMaterial = FailureMaterial(ref failureHighlightMaterial, new Color(1, .25f, .1f, 1));
                var text = new GameObject("Rejected TCP label"); text.transform.SetParent(failureHighlightRoot, false); text.transform.localPosition = Vector3.up * .065f;
                failureHighlightText = text.AddComponent<TextMesh>(); failureHighlightText.anchor = TextAnchor.MiddleCenter; failureHighlightText.characterSize = .01f; failureHighlightText.fontSize = 28;
            }
            failureHighlightRoot.gameObject.SetActive(true); failureHighlightRoot.position = SelectedFailure.TcpPosition;
            failureHighlightRoot.rotation = Quaternion.identity; failureHighlightRoot.localScale = Vector3.one;
            failureHighlightText.text = "Rejected TCP · " + SelectedFailure.Code;
            if (Camera.main != null) failureHighlightText.transform.rotation = Quaternion.LookRotation(failureHighlightText.transform.position - Camera.main.transform.position, Camera.main.transform.up);
        }
        public bool LocateSelectedFailure()
        {
            if (SelectedFailure == null || LastPlanReport == null || LastPlanReport.Stale) return false;
            highlightFailedPosition = true; RefreshFailureHighlight();
#if UNITY_EDITOR
            if (SelectedFailure.HasTcp && UnityEditor.SceneView.lastActiveSceneView != null) UnityEditor.SceneView.lastActiveSceneView.Frame(new Bounds(SelectedFailure.TcpPosition, Vector3.one * .6f), false);
#endif
            return SelectedFailure.HasTcp;
        }
        public bool PreviewSelectedFailure()
        {
            var e = SelectedFailure;
            if (e == null || !e.HasBase || !e.HasJoints || LastPlanReport == null || LastPlanReport.Stale || IsBusy || rig == null) return false;
            ClearFailureGhost(); var map = new Dictionary<Transform, Transform>();
            Transform Copy(Transform source, Transform parent)
            {
                if (source != rig.transform && (source.name == "Planned TCP Waypoint Markers" || source.name == "Planned TCP Waypoint Labels" || source.name.StartsWith("Planner Failed") || source.name == "Planned TCP Path" || source.name == "Planner Rejected Pose")) return null;
                var clone = new GameObject(source.name).transform; clone.SetParent(parent, false); clone.localPosition = source.localPosition; clone.localRotation = source.localRotation; clone.localScale = source.localScale; map[source] = clone;
                var filter = source.GetComponent<MeshFilter>();
                if (filter != null && filter.sharedMesh != null)
                {
                    clone.gameObject.AddComponent<MeshFilter>().sharedMesh = filter.sharedMesh;
                    var renderer = clone.gameObject.AddComponent<MeshRenderer>(); renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    bool implicated = false;
                    for (var ancestor = source; ancestor != null; ancestor = ancestor.parent)
                        if (ancestor.gameObject.GetInstanceID() == e.RobotId)
                        { implicated = GetOwningJointIndex(source) == GetOwningJointIndex(ancestor); break; }
                    renderer.sharedMaterial = implicated ? FailureMaterial(ref failureHighlightMaterial, new Color(1, .25f, .1f, 1)) : FailureMaterial(ref failureGhostMaterial, new Color(.2f, .8f, 1, .25f));
                }
                foreach (Transform child in source) Copy(child, clone);
                return clone;
            }
            failureGhostRoot = new GameObject("Planner Rejected Pose").transform; failureGhostRoot.SetParent(transform, false);
            var ghost = Copy(rig.transform, failureGhostRoot); ghost.position = e.BasePosition; ghost.rotation = e.BaseRotation; ghost.localScale = e.BaseScale;
            var current = rig.GetJointAnglesDegrees();
            for (int i = 0; i < 6; i++) if (map.TryGetValue(rig.Joints[i], out var joint))
                joint.localRotation = rig.Joints[i].localRotation * Quaternion.Inverse(Quaternion.AngleAxis(current[i] + rig.ZeroOffsetsDegrees[i], rig.LocalAxes[i].normalized)) * Quaternion.AngleAxis(e.Joints[i] + rig.ZeroOffsetsDegrees[i], rig.LocalAxes[i].normalized);
            return true;
        }
        private void ClearFailureGhost() { if (failureGhostRoot != null) { failureGhostRoot.gameObject.SetActive(false); Destroy(failureGhostRoot.gameObject); failureGhostRoot = null; } }
        public void HideFailurePreview() => ClearFailureGhost();
        private void ClearFailureVisuals() { if (failureHighlightRoot != null) failureHighlightRoot.gameObject.SetActive(false); ClearFailureGhost(); }
        private void InvalidateFailureVisuals() { if (LastPlanReport != null) { LastPlanReport.Stale = true; LastPlanReport.Revision++; } ClearFailureVisuals(); }
        private void DisposeFailureVisuals()
        {
            ClearFailureVisuals();
            if (failureHighlightRoot != null) { failureHighlightRoot.gameObject.SetActive(false); if (Application.isPlaying) Destroy(failureHighlightRoot.gameObject); else DestroyImmediate(failureHighlightRoot.gameObject); failureHighlightRoot = null; }
            if (failureHighlightMaterial != null) { if (Application.isPlaying) Destroy(failureHighlightMaterial); else DestroyImmediate(failureHighlightMaterial); }
            if (failureGhostMaterial != null) { if (Application.isPlaying) Destroy(failureGhostMaterial); else DestroyImmediate(failureGhostMaterial); }
        }
    }
}
