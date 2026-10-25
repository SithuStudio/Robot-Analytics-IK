using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;
using UnityEngine;
using UnityEngine.UIElements;

namespace RobotIK
{
    /// <summary>Runtime Game-view controls. The planner adds this component automatically in Play mode.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(UIDocument))]
    public sealed class RobotMotionControls : MonoBehaviour
    {
        private VisualElement pathSceneOverlay;
        private Label pathStartLabel, pathEndLabel;
        private Toggle vizEndpointLabels, vizWaypointDots;
        private TextField vizStartText, vizEndText;
        private IReadOnlyList<Vector3> annotationPath, annotationWaypoints;
        private Vector3[] annotationPoints = Array.Empty<Vector3>();
        internal static Vector3 ClosestPathPoint(Vector3 point, IReadOnlyList<Vector3> path)
        {
            if (path == null || path.Count == 0) return point;
            Vector3 best = path[0]; float distance = float.PositiveInfinity;
            for (int i = 1; i < path.Count; i++)
            {
                var edge = path[i] - path[i - 1]; float t = edge.sqrMagnitude < 1e-12f ? 0 : Mathf.Clamp01(Vector3.Dot(point - path[i - 1], edge) / edge.sqrMagnitude);
                var candidate = path[i - 1] + edge * t; float delta = (candidate - point).sqrMagnitude;
                if (delta < distance) { best = candidate; distance = delta; }
            }
            return best;
        }
        private void UpdatePathAnnotations()
        {
            var path = planner.DisplayPathPoints; var waypoints = planner.DisplayWaypointPoints;
            if (!ReferenceEquals(path, annotationPath) || !ReferenceEquals(waypoints, annotationWaypoints))
            {
                annotationPath = path; annotationWaypoints = waypoints;
                annotationPoints = path != null && path.Count > 1 && waypoints != null ? new Vector3[waypoints.Count] : Array.Empty<Vector3>();
                for (int i = 0; i < annotationPoints.Length; i++) annotationPoints[i] = i == 0 ? path[0] : i == annotationPoints.Length - 1 ? path[path.Count - 1] : ClosestPathPoint(waypoints[i], path);
            }
            var camera = Camera.main;
            bool visible = planner.PathEndpointLabels && planner.isActiveAndEnabled && planner.GameplayControlsEnabled && annotationPoints.Length >= 2 && camera != null && pathSceneOverlay.panel != null;
            PositionEndpointLabel(pathStartLabel, planner.PathStartText, visible ? annotationPoints[0] : Vector3.zero, visible, camera);
            PositionEndpointLabel(pathEndLabel, planner.PathEndText, visible ? annotationPoints[annotationPoints.Length - 1] : Vector3.zero, visible, camera);
        }
        private void PositionEndpointLabel(Label label, string text, Vector3 position, bool visible, Camera camera)
        {
            label.style.display = DisplayStyle.None;
            if (!visible) return;
            var screen = camera.WorldToScreenPoint(position); if (screen.z < camera.nearClipPlane) return;
            var point = pathSceneOverlay.WorldToLocal(RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, new Vector2(screen.x, Screen.height - screen.y)));
            label.text = text;
            float width = Mathf.Max(55, label.resolvedStyle.width), height = Mathf.Max(24, label.resolvedStyle.height);
            if (float.IsNaN(width)) width = 55; if (float.IsNaN(height)) height = 24;
            if (point.x < 0 || point.y < 0 || point.x > pathSceneOverlay.contentRect.width || point.y > pathSceneOverlay.contentRect.height) return;
            label.style.left = Mathf.Clamp(point.x + 10, 0, Mathf.Max(0, pathSceneOverlay.contentRect.width - width));
            label.style.top = Mathf.Clamp(point.y - height - 8, 0, Mathf.Max(0, pathSceneOverlay.contentRect.height - height));
            label.style.display = DisplayStyle.Flex;
        }
        private void PaintWaypointDots(MeshGenerationContext context)
        {
            var camera = Camera.main;
            if (!planner.PathWaypointDots || camera == null || !planner.isActiveAndEnabled) return;
            var pen = context.painter2D;
            float pixel = (RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, Vector2.right) - RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, Vector2.zero)).magnitude;
            for (int i = 0; i < annotationPoints.Length; i++)
            {
                var p = camera.WorldToScreenPoint(annotationPoints[i]); if (p.z < camera.nearClipPlane) continue;
                var center = pathSceneOverlay.WorldToLocal(RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, new Vector2(p.x, Screen.height - p.y)));
                for (int pass = 0; pass < 2; pass++)
                {
                    float radius = (i == 0 || i == annotationPoints.Length - 1 ? 4 : 3) * pixel + (pass == 0 ? 1.5f * pixel : 0);
                    pen.fillColor = pass == 0 ? new Color(.02f, .04f, .05f) : i == 0 ? new Color(.3f, .9f, .6f) : i == annotationPoints.Length - 1 ? new Color(1, .45f, .4f) : planner.PathDisplayColor;
                    pen.BeginPath();
                    for (int k = 0; k < 16; k++) { float angle = k * Mathf.PI / 8; var vertex = center + new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * radius; if (k == 0) pen.MoveTo(vertex); else pen.LineTo(vertex); }
                    pen.ClosePath(); pen.Fill();
                }
            }
        }
        private Toggle vizOverlay;
        private VisualElement vizSettings, vizPreview, vizHexRow;
        private DropdownField vizColor;
        private TextField vizHex;
        private Button vizSwatch;
        private Label vizPreviewStatus, vizColorError;
        private bool syncingViz, vizCustomOpen;
        private readonly List<Action> synchronizeViz = new List<Action>();
        private static readonly string[] VizColorNames = { "Cyan", "Blue", "Green", "Orange", "Red", "White", "Custom" };
        private static readonly string[] VizColorHex = { "#00FFFF", "#48A5FF", "#55D88A", "#FFB34D", "#FF6973", "#FFFFFF" };
        private void ChangeViz(bool? visible = null, float? width = null, float? outline = null, float? transparency = null, Color? color = null)
        {
            if (syncingViz) return;
            planner.SetPathVisualization(visible ?? planner.PathOverlayVisible, width ?? planner.PathWidthPixels, outline ?? planner.PathOutlinePixels,
                transparency ?? planner.PathTransparencyPercent, color ?? planner.PathDisplayColor);
            if (color.HasValue && vizColorError != null) vizColorError.style.display = DisplayStyle.None;
            SyncVizControls(); pathSceneOverlay?.MarkDirtyRepaint();
        }
        private void AddVizNumber(string title, string name, float low, float high, float step, string unit, Func<float> read, Action<float> write)
        {
            var row = Element("rm-control-row"); vizSettings.Add(row); row.Add(Text(title, "rm-field-label"));
            var slider = new Slider(low, high) { name = name + "-slider", value = read() }; slider.AddToClassList("rm-viz-slider"); row.Add(slider);
            var box = Element("rm-value-box"); row.Add(box);
            var field = new FloatField { name = name, value = read(), isDelayed = true }; field.AddToClassList("rm-number"); box.Add(field); box.Add(Text(unit, "rm-unit"));
            slider.RegisterValueChangedCallback(e => write(Mathf.Round(e.newValue / step) * step));
            field.RegisterValueChangedCallback(e => write(e.newValue));
            synchronizeViz.Add(() => { slider.SetValueWithoutNotify(read()); field.SetValueWithoutNotify(read()); });
        }
        private void BuildVizControls()
        {
            synchronizeViz.Clear();
            vizPage.Add(Text("PATH VISUALIZATION", "rm-planning-hint"));
            vizOverlay = new Toggle("Path scene overlay") { name = "viz-path-overlay", value = planner.PathOverlayVisible };
            vizOverlay.AddToClassList("rm-edit-toggle"); vizPage.Add(vizOverlay);
            vizOverlay.RegisterValueChangedCallback(e => ChangeViz(visible: e.newValue));
            vizSettings = new VisualElement { name = "viz-path-settings" }; vizPage.Add(vizSettings);
            AddVizNumber("Path width", "viz-path-width", 1, 12, .5f, "px", () => planner.PathWidthPixels, v => ChangeViz(width: v));
            vizSettings.Add(Text("Constant screen width · excludes outline", "rm-planning-hint"));
            AddVizNumber("Outline", "viz-path-outline", 0, 6, .5f, "px", () => planner.PathOutlinePixels, v => ChangeViz(outline: v));
            vizSettings.Add(Text("Extra thickness per side · 0 hides outline", "rm-planning-hint"));
            AddVizNumber("Transparency", "viz-path-transparency", 0, 100, 1, "%", () => planner.PathTransparencyPercent, v => ChangeViz(transparency: v));
            vizSettings.Add(Text("0% opaque → 100% invisible · line and outline", "rm-planning-hint"));
            var row = Element("rm-viz-color-row"); vizSettings.Add(row); row.Add(Text("Path color", "rm-field-label"));
            vizColor = new DropdownField(new List<string>(VizColorNames), 1) { name = "viz-path-color" }; vizColor.AddToClassList("rm-edit-dropdown"); row.Add(vizColor);
            vizSwatch = new Button(() => { vizCustomOpen = !vizCustomOpen; SyncVizControls(); }) { name = "viz-color-swatch", tooltip = "Custom color · enter #RRGGBB" }; row.Add(vizSwatch);
            vizColor.RegisterValueChangedCallback(e =>
            {
                if (syncingViz) return;
                int index = Array.IndexOf(VizColorNames, e.newValue);
                vizCustomOpen = index == 6;
                if (index >= 0 && index < VizColorHex.Length && ColorUtility.TryParseHtmlString(VizColorHex[index], out var color)) ChangeViz(color: color);
                else SyncVizControls();
            });
            vizHexRow = Element("rm-viz-color-row"); vizSettings.Add(vizHexRow); vizHexRow.Add(Text("Custom color", "rm-field-label"));
            vizHex = new TextField { name = "viz-color-hex", isDelayed = true, maxLength = 7 }; vizHex.AddToClassList("rm-edit-dropdown"); vizHexRow.Add(vizHex);
            vizColorError = Text("Use a color such as #48A5FF.", "rm-track-error"); vizColorError.style.display = DisplayStyle.None; vizSettings.Add(vizColorError);
            vizHex.RegisterValueChangedCallback(e =>
            {
                string hex = e.newValue.Trim(); if (!hex.StartsWith("#")) hex = "#" + hex;
                bool valid = hex.Length == 7 && ColorUtility.TryParseHtmlString(hex, out _);
                vizColorError.style.display = valid ? DisplayStyle.None : DisplayStyle.Flex;
                if (valid) { ColorUtility.TryParseHtmlString(hex, out var color); ChangeViz(color: color); }
            });
            vizPreviewStatus = Text("Live preview", "rm-planning-hint"); vizPage.Add(vizPreviewStatus);
            vizPreview = Element("rm-viz-preview"); vizPreview.name = "viz-path-preview"; vizPreview.pickingMode = PickingMode.Ignore;
            vizPreview.generateVisualContent += PaintVizPreview; vizPage.Add(vizPreview);
            vizPage.Add(Text("WAYPOINTS", "rm-planning-hint"));
            vizEndpointLabels = new Toggle("Start / End labels") { name = "viz-endpoint-labels", value = planner.PathEndpointLabels };
            vizWaypointDots = new Toggle("Waypoint dots") { name = "viz-waypoint-dots", value = planner.PathWaypointDots };
            vizEndpointLabels.AddToClassList("rm-edit-toggle"); vizWaypointDots.AddToClassList("rm-edit-toggle");
            vizPage.Add(vizEndpointLabels); vizPage.Add(vizWaypointDots);
            vizStartText = AddEndpointText("Start text", "viz-start-text", planner.PathStartText);
            vizEndText = AddEndpointText("End text", "viz-end-text", planner.PathEndText);
            vizEndpointLabels.RegisterValueChangedCallback(_ => ChangeAnnotations()); vizWaypointDots.RegisterValueChangedCallback(_ => ChangeAnnotations());
            vizStartText.RegisterValueChangedCallback(_ => ChangeAnnotations()); vizEndText.RegisterValueChangedCallback(_ => ChangeAnnotations());
            vizPage.Add(Text("Dots mark planned waypoints, not dense validation samples.", "rm-planning-hint"));
            vizPage.Add(Text("Display only · applies immediately · no replan", "rm-planning-hint"));
            SyncVizControls();
        }
        private TextField AddEndpointText(string title, string name, string value)
        {
            var row = Element("rm-viz-color-row"); vizPage.Add(row); row.Add(Text(title, "rm-field-label"));
            var field = new TextField { name = name, value = value, isDelayed = true, maxLength = 40 }; field.AddToClassList("rm-edit-dropdown"); row.Add(field); return field;
        }
        private void ChangeAnnotations()
        {
            if (syncingViz) return;
            planner.SetPathAnnotations(vizEndpointLabels.value, vizWaypointDots.value, vizStartText.value, vizEndText.value);
            SyncVizControls();
        }
        private void SyncVizControls()
        {
            if (vizOverlay == null) return;
            syncingViz = true;
            try
            {
                vizOverlay.SetValueWithoutNotify(planner.PathOverlayVisible); vizOverlay.EnableInClassList("rm-switch--on", planner.PathOverlayVisible);
                if (vizEndpointLabels != null)
                {
                    vizEndpointLabels.SetValueWithoutNotify(planner.PathEndpointLabels); vizEndpointLabels.EnableInClassList("rm-switch--on", planner.PathEndpointLabels);
                    vizWaypointDots.SetValueWithoutNotify(planner.PathWaypointDots); vizWaypointDots.EnableInClassList("rm-switch--on", planner.PathWaypointDots);
                    vizStartText.SetValueWithoutNotify(planner.PathStartText); vizEndText.SetValueWithoutNotify(planner.PathEndText);
                    vizStartText.SetEnabled(planner.PathEndpointLabels); vizEndText.SetEnabled(planner.PathEndpointLabels);
                }
                vizSettings.SetEnabled(planner.PathOverlayVisible);
                foreach (var sync in synchronizeViz) sync();
                string hex = "#" + ColorUtility.ToHtmlStringRGB(planner.PathDisplayColor);
                int index = Array.IndexOf(VizColorHex, hex);
                vizColor.SetValueWithoutNotify(vizCustomOpen || index < 0 ? "Custom" : VizColorNames[index]);
                vizHex.SetValueWithoutNotify(hex); vizHexRow.style.display = vizCustomOpen || index < 0 ? DisplayStyle.Flex : DisplayStyle.None;
                vizSwatch.style.backgroundColor = planner.PathDisplayColor;
                vizPreviewStatus.text = !planner.PathOverlayVisible ? "Live preview · overlay off" : $"Live preview · {100 - planner.PathTransparencyPercent:0}% opacity";
                vizPreview?.MarkDirtyRepaint();
            }
            finally { syncingViz = false; }
        }
        private void PaintVizPreview(MeshGenerationContext context)
        {
            float w = vizPreview.contentRect.width; if (w <= 0) return;
            var pen = context.painter2D; pen.lineJoin = LineJoin.Round; pen.lineCap = LineCap.Round;
            var a = new Vector2(w * .1f, 32); var b = new Vector2(w * .1f, 103); var c = new Vector2(w * .2f, 103);
            var d = new Vector2(w * .8f, 103); var e = new Vector2(w * .9f, 103); var f = new Vector2(w * .9f, 32);
            if (!planner.PathOverlayVisible) return;
            float pixel = vizPreview.panel == null ? 1 : (RuntimePanelUtils.ScreenToPanel(vizPreview.panel, Vector2.right) - RuntimePanelUtils.ScreenToPanel(vizPreview.panel, Vector2.zero)).magnitude;
            for (int pass = 0; pass < 2; pass++)
            {
                if (pass == 0 && planner.PathOutlinePixels <= 0) continue;
                pen.lineWidth = (planner.PathWidthPixels + (pass == 0 ? 2 * planner.PathOutlinePixels : 0)) * pixel;
                pen.strokeColor = pass == 0 ? new Color(.02f, .04f, .05f, planner.PathStrokeColor.a) : planner.PathStrokeColor;
                pen.BeginPath(); pen.MoveTo(a); pen.LineTo(new Vector2(a.x, 82));
                var start = new Vector2(a.x, 82);
                for (int i = 1; i <= 12; i++) { float t = i / 12f; pen.LineTo(Vector2.Lerp(Vector2.Lerp(start, b, t), Vector2.Lerp(b, c, t), t)); }
                pen.LineTo(d); var end = new Vector2(f.x, 82);
                for (int i = 1; i <= 12; i++) { float t = i / 12f; pen.LineTo(Vector2.Lerp(Vector2.Lerp(d, e, t), Vector2.Lerp(e, end, t), t)); }
                pen.LineTo(f); pen.Stroke();
            }
        }
        private readonly List<Vector2> projectedPathStarts = new List<Vector2>();
        private readonly List<Vector2> projectedPathEnds = new List<Vector2>();
        private void BuildPathOverlay()
        {
            pathSceneOverlay?.RemoveFromHierarchy();
            pathSceneOverlay = new VisualElement { name = "planned-path-screen-overlay", pickingMode = PickingMode.Ignore };
            pathSceneOverlay.style.position = Position.Absolute;
            pathSceneOverlay.style.left = pathSceneOverlay.style.right = pathSceneOverlay.style.top = pathSceneOverlay.style.bottom = 0;
            pathSceneOverlay.style.overflow = Overflow.Hidden;
            pathSceneOverlay.generateVisualContent += PaintPlannedPath;
            pathSceneOverlay.generateVisualContent += PaintWaypointDots;
            pathStartLabel = new Label { name = "path-start-screen-label", pickingMode = PickingMode.Ignore, enableRichText = false };
            pathEndLabel = new Label { name = "path-end-screen-label", pickingMode = PickingMode.Ignore, enableRichText = false };
            pathStartLabel.AddToClassList("rm-path-endpoint"); pathEndLabel.AddToClassList("rm-path-endpoint");
            pathStartLabel.style.display = pathEndLabel.style.display = DisplayStyle.None;
            pathSceneOverlay.Add(pathStartLabel); pathSceneOverlay.Add(pathEndLabel);
            root.Insert(0, pathSceneOverlay);
        }
        // Clip crossing segments instead of dropping them at the near plane.
        internal static bool ProjectPathSegment(Camera camera, Vector3 a, Vector3 b, out Vector2 from, out Vector2 to)
        {
            from = to = default;
            var pa = camera.WorldToScreenPoint(a); var pb = camera.WorldToScreenPoint(b);
            float near = camera.nearClipPlane + .0001f;
            if (pa.z < near && pb.z < near) return false;
            if (pa.z < near) pa = camera.WorldToScreenPoint(Vector3.Lerp(a, b, (near - pa.z) / (pb.z - pa.z)));
            else if (pb.z < near) pb = camera.WorldToScreenPoint(Vector3.Lerp(a, b, (near - pa.z) / (pb.z - pa.z)));
            from = new Vector2(pa.x, Screen.height - pa.y); to = new Vector2(pb.x, Screen.height - pb.y);
            return true;
        }
        private void PaintPlannedPath(MeshGenerationContext context)
        {
            var camera = Camera.main; var points = planner?.DisplayPathPoints;
            if (planner == null || !planner.PathOverlayVisible || planner.PathTransparencyPercent >= 100 || !planner.isActiveAndEnabled || pathSceneOverlay.panel == null || camera == null || points == null || points.Count < 2) return;
            var pen = context.painter2D; pen.lineJoin = LineJoin.Round; pen.lineCap = LineCap.Round;
            // Painter widths are panel units; compensate UI scaling to keep physical pixel width constant.
            float pixel = (RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, Vector2.right) - RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, Vector2.zero)).magnitude;
            projectedPathStarts.Clear(); projectedPathEnds.Clear();
            // Project once and reuse for both strokes, including dense track-transfer paths.
            for (int i = 1; i < points.Count; i++)
            {
                if (!ProjectPathSegment(camera, points[i - 1], points[i], out var a, out var b)) continue;
                projectedPathStarts.Add(pathSceneOverlay.WorldToLocal(RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, a)));
                projectedPathEnds.Add(pathSceneOverlay.WorldToLocal(RuntimePanelUtils.ScreenToPanel(pathSceneOverlay.panel, b)));
            }
            for (int pass = 0; pass < 2; pass++)
            {
                if (pass == 0 && planner.PathOutlinePixels <= 0) continue;
                pen.strokeColor = pass == 0 ? new Color(.02f, .04f, .05f, planner.PathStrokeColor.a) : planner.PathStrokeColor;
                pen.lineWidth = (pass == 0 ? planner.PathWidthPixels + 2 * planner.PathOutlinePixels : planner.PathWidthPixels) * pixel;
                pen.BeginPath(); bool connected = false; Vector2 previous = default;
                for (int i = 0; i < projectedPathStarts.Count; i++)
                {
                    var a = projectedPathStarts[i]; var b = projectedPathEnds[i];
                    if (!connected || (previous - a).sqrMagnitude > .001f) pen.MoveTo(a);
                    pen.LineTo(b); previous = b; connected = true;
                }
                pen.Stroke();
            }
        }
        private void LateUpdate()
        {
            bool active = root != null && root.panel != null && pathSceneOverlay != null && planner != null && planner.isActiveAndEnabled &&
                planner.GameplayControlsEnabled && Camera.main != null;
            if (planner != null) planner.SetScreenPathOverlay(this, active);
            if (active) UpdatePathAnnotations();
            else if (pathStartLabel != null) { pathStartLabel.style.display = pathEndLabel.style.display = DisplayStyle.None; }
            pathSceneOverlay?.MarkDirtyRepaint();
        }
        private Foldout diagnosticPanel;
        private Label diagnosticSummary, diagnosticCounts, diagnosticEvidence;
        private DropdownField diagnosticCandidate;
        private Toggle diagnosticHighlight;
        private Button diagnosticLocate, diagnosticPreview;
        private PlanDiagnosticReport displayedReport;
        private int displayedRevision = -1;
        private readonly List<int> diagnosticIds = new List<int>();
        private VisualElement diagnosticSceneOverlay;
        private void PaintFailurePosition(MeshGenerationContext context)
        {
            var report = planner?.LastPlanReport; var e = planner?.SelectedFailure; var camera = Camera.main;
            if (planner == null || !planner.HighlightFailedPosition || planner.IsBusy || e == null || !e.HasTcp || report == null || report.Stale ||
                report.Outcome == PlanReportOutcome.Complete || report.Outcome == PlanReportOutcome.Running || camera == null || diagnosticSceneOverlay.panel == null) return;
            var screen = camera.WorldToScreenPoint(e.TcpPosition); if (screen.z <= 0) return;
            var point = diagnosticSceneOverlay.WorldToLocal(RuntimePanelUtils.ScreenToPanel(diagnosticSceneOverlay.panel, new Vector2(screen.x, Screen.height - screen.y)));
            var pen = context.painter2D; pen.strokeColor = new Color(1f,.25f,.1f); pen.lineWidth = 2;
            // Screen-space reticle stays legible when the saved TCP lies behind the car body.
            pen.BeginPath();
            pen.MoveTo(point + new Vector2(-13,-7)); pen.LineTo(point + new Vector2(-13,-13)); pen.LineTo(point + new Vector2(-7,-13));
            pen.MoveTo(point + new Vector2(7,-13)); pen.LineTo(point + new Vector2(13,-13)); pen.LineTo(point + new Vector2(13,-7));
            pen.MoveTo(point + new Vector2(13,7)); pen.LineTo(point + new Vector2(13,13)); pen.LineTo(point + new Vector2(7,13));
            pen.MoveTo(point + new Vector2(-7,13)); pen.LineTo(point + new Vector2(-13,13)); pen.LineTo(point + new Vector2(-13,7));
            pen.MoveTo(point + new Vector2(-4,0)); pen.LineTo(point + new Vector2(4,0));
            pen.MoveTo(point + new Vector2(0,-4)); pen.LineTo(point + new Vector2(0,4)); pen.Stroke();
        }
        private void BuildDiagnosticControls(VisualElement summary, VisualElement parent)
        {
            diagnosticSceneOverlay?.RemoveFromHierarchy();
            diagnosticSceneOverlay = new VisualElement { name = "failed-position-scene-overlay", pickingMode = PickingMode.Ignore };
            diagnosticSceneOverlay.style.position = Position.Absolute;
            diagnosticSceneOverlay.style.left = diagnosticSceneOverlay.style.right = diagnosticSceneOverlay.style.top = diagnosticSceneOverlay.style.bottom = 0;
            diagnosticSceneOverlay.generateVisualContent += PaintFailurePosition; root.Insert(0, diagnosticSceneOverlay);
            BuildPathOverlay();
            diagnosticHighlight = new Toggle("Highlight failed position") { name = "highlight-failed-position", value = planner.HighlightFailedPosition,
                tooltip = "Show the saved rejected TCP, not a physical contact point. Search limits alone have no spatial position." };
            diagnosticHighlight.AddToClassList("rm-edit-toggle"); summary.Add(diagnosticHighlight);
            diagnosticHighlight.AddToClassList("rm-modern-switch"); diagnosticHighlight.EnableInClassList("rm-switch--on", diagnosticHighlight.value);
            diagnosticHighlight.RegisterValueChangedCallback(e => { planner.HighlightFailedPosition = e.newValue; diagnosticHighlight.EnableInClassList("rm-switch--on", e.newValue); });
            diagnosticPanel = new Foldout { text = "Failure report · observed blockers", value = false, name = "failure-report" };
            diagnosticPanel.AddToClassList("rm-feedback-details"); parent.Add(diagnosticPanel);
            diagnosticSummary = Text(string.Empty, "rm-diagnostic-summary"); diagnosticPanel.Add(diagnosticSummary);
            diagnosticCounts = Text(string.Empty, "rm-track-info"); diagnosticCounts.name = "failure-blocker-counts"; diagnosticPanel.Add(diagnosticCounts);
            diagnosticCandidate = new DropdownField { name = "failure-candidate", tooltip = "Select a saved representative rejection. Counts include observations; they are not unique obstacles." };
            diagnosticPanel.Add(diagnosticCandidate);
            diagnosticCandidate.RegisterValueChangedCallback(e => {
                int index = diagnosticCandidate.choices.IndexOf(e.newValue);
                if (index >= 0 && index < diagnosticIds.Count) { planner.SelectFailure(diagnosticIds[index]); UpdateDiagnosticSelection(); }
            });
            diagnosticEvidence = Text(string.Empty, "rm-diagnostic-evidence"); diagnosticEvidence.name = "failure-evidence"; diagnosticPanel.Add(diagnosticEvidence);
            var inspect = Element("rm-diagnostic-toolbar"); diagnosticPanel.Add(inspect);
            diagnosticLocate = new Button(() => planner.LocateSelectedFailure()) { text = "Locate TCP", name = "locate-failed-position" }; inspect.Add(diagnosticLocate);
            diagnosticPreview = new Button(() => planner.PreviewSelectedFailure()) { text = "Preview pose", name = "preview-failed-pose" }; inspect.Add(diagnosticPreview);
            inspect.Add(new Button(() => planner.HideFailurePreview()) { text = "Hide pose" });
            var export = Element("rm-diagnostic-toolbar"); diagnosticPanel.Add(export);
            export.Add(new Button(() => GUIUtility.systemCopyBuffer = planner.DiagnosticReportText) { text = "Copy report", name = "copy-failure-report" });
            export.Add(new Button(() => SaveReport(false)) { text = "Save TXT" });
            export.Add(new Button(() => SaveReport(true)) { text = "Save JSON" });
        }
        private void SaveReport(bool json)
        {
            try { planner.SaveDiagnosticReport(json); }
            catch (Exception e) { diagnosticEvidence.text = "Report could not be saved: " + e.Message; }
        }
        private void UpdateDiagnosticSelection()
        {
            var report = planner.LastPlanReport; var e = planner.SelectedFailure;
            diagnosticEvidence.text = e != null ? e.Details() : "No rejected TCP was captured. A search budget or setup failure has no spatial location.";
            bool current = report != null && !report.Stale && !planner.IsBusy && report.Outcome != PlanReportOutcome.Complete;
            diagnosticLocate.SetEnabled(current && e != null && e.HasTcp);
            diagnosticPreview.SetEnabled(current && e != null && e.HasJoints && e.HasBase);
            diagnosticHighlight.SetEnabled(current && e != null && e.HasTcp);
            diagnosticHighlight.SetValueWithoutNotify(planner.HighlightFailedPosition);
            diagnosticHighlight.EnableInClassList("rm-switch--on", planner.HighlightFailedPosition);
        }
        private void RefreshDiagnosticControls()
        {
            if (diagnosticPanel == null) return;
            diagnosticSceneOverlay?.MarkDirtyRepaint();
            var report = planner.LastPlanReport;
            diagnosticPanel.style.display = report == null ? DisplayStyle.None : DisplayStyle.Flex;
            diagnosticHighlight.style.display = report == null || report.Outcome == PlanReportOutcome.Complete ? DisplayStyle.None : DisplayStyle.Flex;
            if (report == null) return;
            // Rebuild saved evidence only when a search stops; do not rebuild 128 rows each planning frame.
            foreach (var button in diagnosticPanel.Query<Button>().ToList()) button.SetEnabled(!planner.IsBusy);
            diagnosticCandidate.style.display = report.Outcome == PlanReportOutcome.Running ? DisplayStyle.None : DisplayStyle.Flex;
            if (report.Outcome == PlanReportOutcome.Running)
            {
                diagnosticSummary.text = "Planning · collecting rejection evidence"; diagnosticCounts.text = string.Empty;
                diagnosticEvidence.text = "Saved candidate details become available when the search stops.";
                diagnosticHighlight.SetEnabled(false); return;
            }
            if (displayedReport == report && displayedRevision == report.Revision) { UpdateDiagnosticSelection(); return; }
            displayedReport = report; displayedRevision = report.Revision;
            diagnosticSummary.text = (report.Stale ? "Historical report · scene/settings changed\n" : string.Empty) +
                (report.Outcome == PlanReportOutcome.Complete ? "Motion completed · rejected alternatives below" : PlanDiagnosticReport.StatusTitle(report.StopCode));
            var counts = new List<string>();
            foreach (var c in report.Counters) counts.Add(c.Code + " " + c.Observations.ToString("N0"));
            diagnosticCounts.text = "Observed rejections: " + (counts.Count == 0 ? "none captured" : string.Join(" · ", counts)) +
                $"\nQueries {report.ConfigurationQueries:N0} · cache hits {report.CacheHits:N0} · connections {report.Connections}" +
                (report.DroppedEvents > 0 ? $"\nSaved {report.Events.Count} representatives; {report.DroppedEvents:N0} omitted. Full observation counts retained." : string.Empty);
            var choices = new List<string>(); diagnosticIds.Clear();
            foreach (var e in report.Events) { choices.Add($"#{e.Id} · {e.Code} · {e.Phase}"); diagnosticIds.Add(e.Id); }
            diagnosticCandidate.choices = choices; diagnosticCandidate.SetEnabled(choices.Count > 0);
            int selected = planner.SelectedFailure == null ? -1 : diagnosticIds.IndexOf(planner.SelectedFailure.Id);
            diagnosticCandidate.SetValueWithoutNotify(selected >= 0 ? choices[selected] : string.Empty);
            UpdateDiagnosticSelection();
        }
        private static readonly HashSet<RobotMotionControls> activePanels = new HashSet<RobotMotionControls>();
        public enum ColorTheme { Light, Dark }
        private const string ThemePreferenceKey = "RobotIK.MotionControls.ColorTheme";
        [SerializeField] private ColorTheme colorTheme = ColorTheme.Light;
        public ColorTheme Theme => colorTheme;
        public enum ControlsTab { Motion, SceneEditor, Viz }
        public ControlsTab ActiveTab { get; private set; }
        private VisualElement motionPage, scenePage, vizPage;
        private Button motionTab, sceneTab, vizTab;
        private readonly List<Action<RobotBaseSixAxisMotionPlanner.PlanningSettings>> synchronizePlanning = new List<Action<RobotBaseSixAxisMotionPlanner.PlanningSettings>>();
        [SerializeField] private RobotBaseSixAxisMotionPlanner planner;
        private UIDocument document;
        private ScrollView scrollPanel;
        private PanelSettings ownedPanelSettings;
        private VisualElement root, settingsContainer;
        private VisualElement panelHeader, panelBody, windowPanel, panelTabs, actionBar;
        private Foldout ikDetails, feedbackDetails, feedbackSummary;
        private Foldout trackSettings;
        private VisualElement trackBody;
        private Toggle trackEnabled;
        private FloatField trackPosition;
        private Slider trackPositionSlider;
        private bool synchronizingTrackPosition;
        private Label trackRangeHome, trackRangeEnd, waypointCount, waypointChange;
        private Foldout reachSettings;
        private Toggle reachEnabled, reachAllowE1;
        private VisualElement reachBody;
        private Button reachPrefer, reachStrict;
        private Slider reachPercentSlider;
        private FloatField reachPercent;
        private EnumField reachStrength, reachScope;
        private Label reachLimitLabel, reachFeedback, peakStretchFeedback;
        private bool reachUiStrict;
        private string reachUiError;
        private Label trackInfo, trackError;
        private Toggle automaticStation;
        private FloatField stationSpacing, stationBudget;
        private IntegerField stationCandidates, stationRoutes;
        private Foldout stationLimits;
        private Label stationSummary;
        private Foldout stationDetails;
        private Label stationLog;
        private Toggle coordinatedMode;
        private Foldout coordinatedLimits;
        private FloatField coordinatedCartesianStep, coordinatedTrackStep, coordinatedBudget, coordinatedTrackSpeed;
        private IntegerField coordinatedBeam;
        private Label coordinatedFeedback;
        private Label trackReachabilityFeedback;
        private Toggle stagedTransferMode;
        private Foldout stagedTransferSettings, stagedTransferDetails;
        private FloatField transferSpacing, transferApproach, transferBudget, transferSpeed, transferAcceleration;
        private IntegerField transferStations, transferIterations, transferWaypoints;
        private FloatField transferReductionPosition, transferReductionRotation, transferOptimizationBudget;
        private Toggle transferPreferCorridor, transferRequireCorridor;
        private FloatField transferCorridorDrop;
        private Label transferDetails;
        private readonly List<Toggle> trackChannels = new List<Toggle>();
        private string trackUiError;
        private Button minimizeButton;
        private bool minimized, draggingPanel, hasPanelPosition;
        private int dragPointerId;
        private Vector2 panelPosition, viewportSize, dragStartPointer, dragStartPosition;
        private const float ScreenMargin = 16f;
        public bool IsMinimized => minimized;
        public Vector2 PanelPosition => panelPosition;
        private Label stateLabel, stateDot, message, feedback, iterations, planningTime, travelTime;
        private VisualElement endpointStep, planStep, motionStep;
        private Label endpointStepTitle, endpointStepDetail, planStepTitle, planStepDetail, motionStepTitle, motionStepDetail, feedbackNext;
        private DropdownField algorithm;
        private VisualElement plannerMenuRoot;
        private Button planButton, replayButton, themeButton, headerStopButton;
        private Button startPoseButton, endPoseButton;
        private Toggle checkIkBeforePlan;
        private Label startIkResult, endIkResult, startIkCompact, endIkCompact;
        private string previousStartIkSummary, previousEndIkSummary;
        private ColorTheme appliedTheme;
        private readonly List<Action<MotionProfile.Settings>> synchronize = new List<Action<MotionProfile.Settings>>();
        private readonly RobotBaseSixAxisMotionPlanner.PlannerAlgorithm[] algorithms =
        {
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.CartesianLinear,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.CartesianArc,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.GridAStar,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.RRTConnectOptimized,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.RRTConnectFast,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.JointSpacePrm,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.InformedRrtStar
        };
        private MotionProfile.Settings draft;
        private bool pendingSettings, syncing, algorithmChanged;
        private float applyAt, nextSync;
        private string previousStatus;
        private RobotBaseSixAxisMotionPlanner.MotionState previousState;
        private RobotRuntimeSceneEditor sceneEditor;

        public void Bind(RobotBaseSixAxisMotionPlanner target)
        {
            if (isActiveAndEnabled) activePanels.Add(this);
            if (planner == target && root != null) return;
            if (planner != null) planner.SetScreenPathOverlay(this, false);
            planner = target;
            if (isActiveAndEnabled) Build();
        }

        private void OnEnable()
        {
            activePanels.Add(this);
            int savedTheme = PlayerPrefs.GetInt(ThemePreferenceKey, (int)colorTheme);
            if (Enum.IsDefined(typeof(ColorTheme), savedTheme)) colorTheme = (ColorTheme)savedTheme;
            if (planner == null) planner = GetComponent<RobotBaseSixAxisMotionPlanner>();
            if (planner != null) Build();
        }

        private void OnDisable()
        {
            if (planner != null) planner.SetScreenPathOverlay(this, false);
            plannerMenuRoot?.RemoveFromHierarchy();
            activePanels.Remove(this);
            EndPanelDrag();
            pendingSettings = false;
            if (root != null) root.style.display = DisplayStyle.None;
        }

        private void OnDestroy()
        {
            if (planner != null) planner.SetScreenPathOverlay(this, false);
            activePanels.Remove(this);
            if (ownedPanelSettings == null) return;
            if (document != null && document.panelSettings == ownedPanelSettings) document.panelSettings = null;
            if (Application.isPlaying) Destroy(ownedPanelSettings);
            else DestroyImmediate(ownedPanelSettings);
        }

        /// <summary>Screen coordinates use Unity's bottom-left origin.</summary>
        public static bool IsPointerOverControls(Vector2 screenPosition)
        {
            foreach (RobotMotionControls controls in activePanels)
            {
                if (controls == null || controls.root == null || controls.planner == null ||
                    !controls.planner.isActiveAndEnabled || !controls.planner.GameplayControlsEnabled || controls.root.panel == null) continue;
                if (controls.draggingPanel) return true;
                Vector2 point = RuntimePanelUtils.ScreenToPanel(controls.root.panel,
                    new Vector2(screenPosition.x, Screen.height - screenPosition.y));
                VisualElement picked = controls.root.panel.Pick(point);
                if (picked != null && controls.plannerMenuRoot != null && controls.plannerMenuRoot.Contains(picked)) return true;
                if (picked != null && picked != controls.root && controls.root.Contains(picked)) return true;
            }
            return false;
        }

        public static bool IsEditingControls()
        {
            foreach (RobotMotionControls controls in activePanels)
            {
                var focused = controls != null ? controls.root?.focusController?.focusedElement as VisualElement : null;
                if (IsNumericField(focused)) return true;
            }
            return false;
        }

        public static void ReleaseControlFocus()
        {
            foreach (RobotMotionControls controls in activePanels)
                (controls != null ? controls.root?.focusController?.focusedElement : null)?.Blur();
        }

        private void Build()
        {
            EndPanelDrag();
            document = GetComponent<UIDocument>();
            if (document.panelSettings == null)
            {
                ownedPanelSettings = ScriptableObject.CreateInstance<PanelSettings>();
                ownedPanelSettings.name = "Robot motion runtime panel";
                ownedPanelSettings.scaleMode = PanelScaleMode.ScaleWithScreenSize;
                ownedPanelSettings.referenceResolution = new Vector2Int(1600, 900);
                ownedPanelSettings.screenMatchMode = PanelScreenMatchMode.MatchWidthOrHeight;
                ownedPanelSettings.match = 0.5f;
                ownedPanelSettings.sortingOrder = 100;
                ownedPanelSettings.themeStyleSheet = Resources.Load<ThemeStyleSheet>("RobotMotionTheme");
                document.panelSettings = ownedPanelSettings;
            }
            root = document.rootVisualElement;
            root.Clear();
            root.pickingMode = PickingMode.Ignore;
            root.AddToClassList("robot-motion-overlay");
            StyleSheet sheet = Resources.Load<StyleSheet>("RobotMotionControls");
            if (sheet == null)
            {
                Debug.LogError("Robot motion UI stylesheet is missing from Resources.", this);
                return;
            }
            if (!root.styleSheets.Contains(sheet)) root.styleSheets.Add(sheet);
            synchronize.Clear();
            synchronizePlanning.Clear();
            draft = planner.GetMotionSettings();
            pendingSettings = false;

            var panel = Element("rm-panel-content");
            windowPanel = panel;
            panel.AddToClassList("rm-panel");
            root.Add(panel);
            panel.RegisterCallback<GeometryChangedEvent>(_ => { UpdateScrollHeight(); ConstrainPanelToViewport(); });
            root.UnregisterCallback<GeometryChangedEvent>(ResizePanel);
            root.RegisterCallback<GeometryChangedEvent>(ResizePanel);

            var header = Element("rm-header");
            panelHeader = header;
            header.tooltip = "Drag to move the panel";
            header.RegisterCallback<PointerDownEvent>(BeginPanelDrag);
            header.RegisterCallback<PointerMoveEvent>(MovePanelDrag);
            header.RegisterCallback<PointerUpEvent>(evt =>
            {
                if (!draggingPanel || evt.pointerId != dragPointerId) return;
                EndPanelDrag();
                evt.StopPropagation();
            });
            header.RegisterCallback<PointerCancelEvent>(_ => EndPanelDrag());
            header.RegisterCallback<PointerCaptureOutEvent>(_ => draggingPanel = false);
            var heading = Element("rm-heading");
            heading.Add(Text("Robot motion", "rm-title"));
            header.Add(heading);
            var state = Element("rm-state");
            stateDot = Text("", "rm-state-dot");
            stateLabel = Text("Ready", "rm-state-label");
            state.Add(stateDot); state.Add(stateLabel);
            var headerTools = Element("rm-header-tools");
            headerTools.Add(state);
            themeButton = new Button(() => SetTheme(colorTheme == ColorTheme.Light ? ColorTheme.Dark : ColorTheme.Light));
            themeButton.AddToClassList("rm-theme-button");
            var windowActions = Element("rm-window-actions");
            headerStopButton = new Button(StopImmediately) { text = "Stop", name = "header-stop-motion" };
            headerStopButton.AddToClassList("rm-header-stop"); windowActions.Add(headerStopButton);
            headerStopButton.style.display = DisplayStyle.None;
            windowActions.Add(themeButton);
            minimizeButton = new Button(() => SetMinimized(!minimized));
            minimizeButton.AddToClassList("rm-minimize-button");
            windowActions.Add(minimizeButton);
            headerTools.Add(windowActions); header.Add(headerTools);
            panel.Add(header);
            ApplyTheme();
            panelBody = Element("rm-panel-body");
            panel.Add(panelBody);
            var tabs = Element("rm-tabs"); panelBody.Add(tabs);
            panelTabs = tabs;
            motionTab = new Button(() => SetActiveTab(ControlsTab.Motion)) { text = "Motion", name = "motion-tab" };
            sceneTab = new Button(() => SetActiveTab(ControlsTab.SceneEditor)) { text = "Scene editor", name = "scene-editor-tab" };
            vizTab = new Button(() => SetActiveTab(ControlsTab.Viz)) { text = "Viz", name = "viz-tab" }; vizTab.AddToClassList("rm-tab");
            motionTab.AddToClassList("rm-tab"); sceneTab.AddToClassList("rm-tab");
            tabs.Add(motionTab); tabs.Add(sceneTab); tabs.Add(vizTab);
            scrollPanel = new ScrollView(ScrollViewMode.Vertical);
            scrollPanel.AddToClassList("rm-body-scroll");
            scrollPanel.horizontalScrollerVisibility = ScrollerVisibility.Hidden;
            scrollPanel.verticalScrollerVisibility = ScrollerVisibility.Auto;
            panelBody.Add(scrollPanel);
            motionPage = Element("rm-motion-page"); motionPage.name = "motion-page"; scrollPanel.Add(motionPage);
            scenePage = Element("rm-scene-page"); scenePage.name = "scene-editor-page"; scrollPanel.Add(scenePage);
            vizPage = Element("rm-viz-page"); vizPage.name = "viz-page"; scrollPanel.Add(vizPage); BuildVizControls();

            settingsContainer = Element("rm-settings"); motionPage.Add(settingsContainer);
            var plannerRow = Element("rm-planner-row"); settingsContainer.Add(plannerRow);
            plannerRow.Add(Text("Planner", "rm-field-label"));
            algorithm = new DropdownField(new List<string>
            {
                "Cartesian Linear (MoveL)", "Cartesian Arc", "3D Grid A*", "Optimized RRT-Connect", "Fast RRT-Connect", "Joint-Space PRM", "Informed RRT*"
            }, Array.IndexOf(algorithms, planner.SelectedPlanner));
            algorithm.name = "planner-algorithm";
            algorithm.AddToClassList("rm-algorithm");
            algorithm.RegisterCallback<PointerDownEvent>(evt =>
            {
                if (evt.button != 0) return;
                evt.StopImmediatePropagation();
                ShowPlannerMenu();
            }, TrickleDown.TrickleDown);
            algorithm.RegisterCallback<PointerMoveEvent>(evt =>
            {
                if ((evt.pressedButtons & 1) != 0) evt.StopImmediatePropagation();
            }, TrickleDown.TrickleDown);
            algorithm.RegisterCallback<NavigationSubmitEvent>(evt =>
            {
                evt.StopImmediatePropagation();
                ShowPlannerMenu();
            }, TrickleDown.TrickleDown);
            algorithm.RegisterValueChangedCallback(evt =>
            {
                if (syncing || planner.IsBusy) return;
                if (algorithm.index >= 0 && algorithm.index < algorithms.Length && planner.SetPlannerAlgorithm(algorithms[algorithm.index]))
                {
                    algorithmChanged = true;
                    message.text = "Planner changed. Reset & plan to create a new route.";
                }
            });
            plannerRow.Add(algorithm);
            var endpointChecks = Element("rm-endpoint-checks"); settingsContainer.Add(endpointChecks);
            var poseHeading = Element("rm-pose-heading"); endpointChecks.Add(poseHeading);
            poseHeading.Add(Text("POSE CHECK", "rm-section-label"));
            checkIkBeforePlan = new Toggle("Check before planning") { name = "check-endpoint-ik-before-plan", value = planner.CheckEndpointIkBeforePlanning,
                tooltip = "Check Start and End IK before planning. Planning always retains mandatory collision and clearance validation." };
            checkIkBeforePlan.AddToClassList("rm-edit-toggle");
            checkIkBeforePlan.RegisterValueChangedCallback(evt => planner.CheckEndpointIkBeforePlanning = evt.newValue);
            poseHeading.Add(checkIkBeforePlan);
            var poseButtons = Element("rm-pose-buttons"); endpointChecks.Add(poseButtons);
            startPoseButton = new Button(() =>
            {
                if (!ApplyPendingSettings()) return;
                planner.ShowEndpointIkPose(true);
                RefreshStatus();
            }) { name = "show-start-pose", tooltip = "Show Start IK pose, highlighting colliding parts in red." };
            endPoseButton = new Button(() =>
            {
                if (!ApplyPendingSettings()) return;
                planner.ShowEndpointIkPose(false);
                RefreshStatus();
            }) { name = "show-end-pose", tooltip = "Show End IK pose, highlighting colliding parts in red." };
            startPoseButton.Add(Text("Start Pose", "rm-pose-caption"));
            startIkCompact = Text("Not checked", "rm-pose-state"); startPoseButton.Add(startIkCompact);
            endPoseButton.Add(Text("End Pose", "rm-pose-caption"));
            endIkCompact = Text("Not checked", "rm-pose-state"); endPoseButton.Add(endIkCompact);
            startPoseButton.AddToClassList("rm-secondary"); poseButtons.Add(startPoseButton);
            endPoseButton.AddToClassList("rm-secondary"); poseButtons.Add(endPoseButton);
            ikDetails = new Foldout { text = "IK details & help", value = false, name = "endpoint-ik-details" };
            ikDetails.AddToClassList("rm-ik-details"); endpointChecks.Add(ikDetails);
            startIkResult = Text(planner.StartIkSummary, "rm-ik-result"); ikDetails.Add(startIkResult);
            endIkResult = Text(planner.EndIkSummary, "rm-ik-result"); ikDetails.Add(endIkResult);
            ikDetails.Add(Text("Pose buttons show the robot at the selected IK solution. Red parts indicate actual obstacle contact. Unreachable poses leave the robot unchanged. Results describe the last check.", "rm-planning-hint"));
            ikDetails.Add(Text("Accuracy rounds corners within the selected distance; unsafe curves shrink or stay sharp. Start and End stay exact. Clearance is the minimum robot-to-obstacle gap enforced during planning.", "rm-planning-hint"));
            previousStartIkSummary = previousEndIkSummary = null;

            AddNumberControl(settingsContainer, "Speed", "mm/s", 1000f,
                s => s.LinearSpeed, (s, v) => s.LinearSpeed = v, "Slower", "Faster");
            AddNumberControl(settingsContainer, "Accuracy", "mm", 500f,
                s => s.PathAccuracyMillimeters, (s, v) => s.PathAccuracyMillimeters = v, "Closer to waypoints", "Wider curves");
            AddPlanningNumber(settingsContainer, "Clearance", "mm", "minimum-clearance", 150f,
                s => s.MinimumObstacleClearanceMillimeters, (s, v) => s.MinimumObstacleClearanceMillimeters = v);
            AddReachControls(settingsContainer);
            AddTrackControls(settingsContainer);
            var adaptive = new Foldout { text = "Adaptive goal", value = false, name = "adaptive-goal-settings" };
            adaptive.AddToClassList("rm-adaptive-goal"); settingsContainer.Add(adaptive);
            var adaptiveEnabled = new Toggle("Allow goal adjustment") { name = "adaptive-goal-enabled" };
            adaptiveEnabled.AddToClassList("rm-edit-toggle"); adaptive.Add(adaptiveEnabled);
            adaptive.Add(Text("If the exact goal is unavailable, find a reachable pose within these limits. Clearance still applies.", "rm-planning-hint"));
            var tolerances = Element("rm-goal-tolerances"); adaptive.Add(tolerances);
            adaptiveEnabled.RegisterValueChangedCallback(evt => ChangePlanningSettings(s => s.AdaptiveGoalEnabled = evt.newValue));
            synchronizePlanning.Add(s => { adaptiveEnabled.SetValueWithoutNotify(s.AdaptiveGoalEnabled); tolerances.SetEnabled(s.AdaptiveGoalEnabled); });
            AddPlanningNumber(tolerances, "Position tolerance", "mm", "goal-position-tolerance", 0f,
                s => s.PositionToleranceMillimeters, (s, v) => s.PositionToleranceMillimeters = v);
            AddPlanningNumber(tolerances, "Rotation tolerance", "°", "goal-rotation-tolerance", 0f,
                s => s.RotationToleranceDegrees, (s, v) => s.RotationToleranceDegrees = v, 180f);
            var samplesRow = Element("rm-control-row"); samplesRow.AddToClassList("rm-control"); tolerances.Add(samplesRow);
            samplesRow.Add(Text("Search samples", "rm-field-label"));
            var samples = new IntegerField { name = "goal-maximum-samples", isDelayed = true, tooltip = "Maximum nearby goal poses to test (1–512)." };
            samples.AddToClassList("rm-number"); samples.AddToClassList("rm-samples"); samplesRow.Add(samples);
            samples.RegisterValueChangedCallback(evt => ChangePlanningSettings(s => s.MaximumGoalSamples = Mathf.Clamp(evt.newValue, 1, 512)));
            synchronizePlanning.Add(s => samples.SetValueWithoutNotify(s.MaximumGoalSamples));
            var advanced = new Foldout { text = "Motion settings", value = false };
            advanced.AddToClassList("rm-advanced"); settingsContainer.Add(advanced);
            var posePositionRetry = new Toggle("Intermediate position retry (20 mm)") { name = "intermediate-position-retry", value = planner.IntermediatePositionRetry };
            var poseRotationRetry = new Toggle("Intermediate rotation retry (5°)") { name = "intermediate-rotation-retry", value = planner.IntermediateRotationRetry };
            posePositionRetry.AddToClassList("rm-edit-toggle"); advanced.Add(posePositionRetry);
            poseRotationRetry.AddToClassList("rm-edit-toggle"); advanced.Add(poseRotationRetry);
            posePositionRetry.RegisterValueChangedCallback(evt => planner.IntermediatePositionRetry = evt.newValue);
            poseRotationRetry.RegisterValueChangedCallback(evt => planner.IntermediateRotationRetry = evt.newValue);
            synchronizePlanning.Add(s => {
                posePositionRetry.SetValueWithoutNotify(planner.IntermediatePositionRetry);
                poseRotationRetry.SetValueWithoutNotify(planner.IntermediateRotationRetry);
            });
            advanced.Add(Text("MoveL fallback after Start posture attempts fail. Start/End stay exact; adaptive goal is suppressed while enabled. The 5° joint-step, singularity and collision checks still apply.", "rm-planning-hint"));
            AddNumberControl(advanced, "Rotation", "°/s", 0f, s => s.RotationSpeedDegree, (s, v) => s.RotationSpeedDegree = v);
            AddNumberControl(advanced, "Acceleration", "mm/s²", 0f, s => s.TcpAccelerationMillimetersPerSecondSquared, (s, v) => s.TcpAccelerationMillimetersPerSecondSquared = v);
            AddNumberControl(advanced, "Deceleration", "mm/s²", 0f, s => s.TcpDecelerationMillimetersPerSecondSquared, (s, v) => s.TcpDecelerationMillimetersPerSecondSquared = v);
            AddNumberControl(advanced, "Angular acceleration", "°/s²", 0f, s => s.TcpRotationAccelerationDegreesPerSecondSquared, (s, v) => s.TcpRotationAccelerationDegreesPerSecondSquared = v);
            AddNumberControl(advanced, "Angular deceleration", "°/s²", 0f, s => s.TcpRotationDecelerationDegreesPerSecondSquared, (s, v) => s.TcpRotationDecelerationDegreesPerSecondSquared = v);
            sceneEditor = planner.GetComponent<RobotRuntimeSceneEditor>();
            if (sceneEditor == null) sceneEditor = planner.gameObject.AddComponent<RobotRuntimeSceneEditor>();
            sceneEditor.Bind(planner);
            sceneEditor.AttachOverlay(root);
            scenePage.Add(sceneEditor.CreateControls(false, ShowThemedMenu));
            ConfigureModernSwitches();
            scrollPanel.verticalScroller.lowButton.style.display = DisplayStyle.None;
            scrollPanel.verticalScroller.highButton.style.display = DisplayStyle.None;

            var status = Element("rm-status-info"); motionPage.Add(status);
            message = Text("Ready to plan a route.", "rm-status-message");
            var metrics = Element("rm-metrics"); status.Add(metrics);
            iterations = AddMetric(metrics, "Iterations");
            planningTime = AddMetric(metrics, "Plan");
            travelTime = AddMetric(metrics, "Travel");
            waypointCount = AddMetric(metrics, "Waypoints"); waypointCount.name = "planned-waypoint-count";
            waypointChange = Text(string.Empty, "rm-waypoint-change"); waypointChange.name = "waypoint-optimization-change"; waypointCount.parent.Add(waypointChange);
            waypointCount.tooltip = "Actual planned waypoints. Dense collision and playback samples are retained separately for track transfers.";
            var stepLog = Element("rm-step-log"); status.Add(stepLog);
            stepLog.Add(Text("Planner feedback", "rm-step-heading"));
            feedbackSummary = new Foldout { text = "Ready to plan", value = false, name = "planner-feedback-summary" };
            feedbackSummary.AddToClassList("rm-feedback-summary"); stepLog.Add(feedbackSummary);
            endpointStep = AddStep(feedbackSummary, "endpoint-step", out endpointStepTitle, out endpointStepDetail);
            planStep = AddStep(feedbackSummary, "plan-step", out planStepTitle, out planStepDetail);
            motionStep = AddStep(feedbackSummary, "motion-step", out motionStepTitle, out motionStepDetail);
            peakStretchFeedback = Text(string.Empty, "rm-track-info"); peakStretchFeedback.name = "peak-stretch-feedback"; feedbackSummary.Add(peakStretchFeedback);
            feedbackNext = Text(string.Empty, "rm-step-next"); feedbackNext.name = "feedback-next"; feedbackSummary.Add(feedbackNext);
            feedbackDetails = new Foldout { text = "Technical details", value = false, name = "planner-feedback" };
            feedbackDetails.AddToClassList("rm-feedback-details"); status.Add(feedbackDetails);
            feedback = Text(planner.Status, "rm-feedback-message"); feedbackDetails.Add(feedback);
            BuildDiagnosticControls(feedbackSummary, status);
            actionBar = Element("rm-action-bar"); panelBody.Add(actionBar);

            planButton = new Button(() =>
            {
                if (!ApplyPendingSettings()) return;
                algorithmChanged = false;
                planner.ResetToStartAndPlan();
                RefreshStatus();
            }) { text = "Reset & plan" };
            planButton.AddToClassList("rm-primary"); actionBar.Add(planButton);
            replayButton = new Button(() =>
            {
                if (planner.IsBusy || planner.IsRrtSearchInProgress) { StopImmediately(); return; }
                if (!ApplyPendingSettings()) return;
                planner.ReplayPlannedMotion();
                RefreshStatus();
            }) { text = "Replay", name = "replay-or-stop", tooltip = "Replay the stored motion" };
            replayButton.AddToClassList("rm-secondary"); actionBar.Add(replayButton);
            panelBody.Add(message);
            message.RegisterCallback<GeometryChangedEvent>(_ => UpdateScrollHeight());
            SyncPlanningSettings();
            SetActiveTab(ActiveTab);
            SetMinimized(minimized);
            previousStatus = null;
            RefreshStatus();
        }

        /// <summary>Switches pages without rebuilding controls or discarding edits or selection.</summary>
        public void SetActiveTab(ControlsTab tab)
        {
            if (!Enum.IsDefined(typeof(ControlsTab), tab)) return;
            ReleaseControlFocus();
            ActiveTab = tab;
            if (motionPage == null || scenePage == null) return;
            motionPage.style.display = tab == ControlsTab.Motion ? DisplayStyle.Flex : DisplayStyle.None;
            scenePage.style.display = tab == ControlsTab.SceneEditor ? DisplayStyle.Flex : DisplayStyle.None;
            vizPage.style.display = tab == ControlsTab.Viz ? DisplayStyle.Flex : DisplayStyle.None;
            if (actionBar != null) actionBar.style.display = tab == ControlsTab.Motion ? DisplayStyle.Flex : DisplayStyle.None;
            if (message != null) message.style.display = tab == ControlsTab.Motion ? DisplayStyle.Flex : DisplayStyle.None;
            motionTab.EnableInClassList("rm-tab--active", tab == ControlsTab.Motion);
            sceneTab.EnableInClassList("rm-tab--active", tab == ControlsTab.SceneEditor);
            vizTab.EnableInClassList("rm-tab--active", tab == ControlsTab.Viz);
            scrollPanel.scrollOffset = Vector2.zero;
            if (planButton != null) RefreshStatus();
            windowPanel.schedule.Execute(() => { UpdateScrollHeight(); ConstrainPanelToViewport(); });
        }

        private void AddPlanningNumber(VisualElement parent, string title, string unit, string controlName, float range,
            Func<RobotBaseSixAxisMotionPlanner.PlanningSettings, float> read,
            Action<RobotBaseSixAxisMotionPlanner.PlanningSettings, float> write, float maximum = float.MaxValue)
        {
            var group = Element("rm-control"); parent.Add(group);
            var row = Element("rm-control-row"); group.Add(row);
            var label = Text(title, "rm-field-label"); row.Add(label);
            var box = Element("rm-value-box"); row.Add(box);
            var number = new FloatField { name = controlName, isDelayed = true, tooltip = title + " (" + unit + ")" };
            number.AddToClassList("rm-number"); box.Add(number); box.Add(Text(unit, "rm-unit"));
            label.RegisterCallback<ClickEvent>(_ => number.Focus());
            Slider slider = null;
            if (range > 0f)
            {
                slider = new Slider(0f, range) { name = controlName + "-slider" };
                slider.AddToClassList("rm-slider"); row.Insert(1, slider);
                slider.tooltip = "Minimum obstacle clearance: closer to obstacles / more space (mm).";
                slider.RegisterValueChangedCallback(evt => ChangePlanningSettings(s => write(s, evt.newValue)));
            }
            number.RegisterValueChangedCallback(evt =>
            {
                if (float.IsNaN(evt.newValue) || float.IsInfinity(evt.newValue)) { SyncPlanningSettings(); return; }
                ChangePlanningSettings(s => write(s, Mathf.Clamp(evt.newValue, 0f, maximum)));
            });
            synchronizePlanning.Add(s =>
            {
                float value = read(s); number.SetValueWithoutNotify(value);
                if (slider != null) { slider.highValue = Mathf.Max(range, value); slider.SetValueWithoutNotify(value); }
            });
        }

        private void ChangePlanningSettings(Action<RobotBaseSixAxisMotionPlanner.PlanningSettings> change)
        {
            if (syncing || planner == null) return;
            if (!planner.IsBusy)
            {
                var settings = planner.GetPlanningSettings(); change(settings);
                planner.ApplyPlanningSettings(settings);
            }
            SyncPlanningSettings();
            RefreshStatus();
        }

        private void SyncPlanningSettings()
        {
            var settings = planner.GetPlanningSettings();
            foreach (var sync in synchronizePlanning) sync(settings);
        }

        private static bool IsNumericField(VisualElement focused) => focused is FloatField || focused is IntegerField ||
            focused?.GetFirstAncestorOfType<FloatField>() != null || focused?.GetFirstAncestorOfType<IntegerField>() != null;

        private void AddNumberControl(VisualElement parent, string title, string unit, float range,
            Func<MotionProfile.Settings, float> read, Action<MotionProfile.Settings, float> write,
            string lowLabel = null, string highLabel = null)
        {
            var group = Element("rm-control"); parent.Add(group);
            var row = Element("rm-control-row"); group.Add(row);
            var label = Text(title, "rm-field-label"); row.Add(label);
            var valueBox = Element("rm-value-box"); row.Add(valueBox);
            var number = new FloatField { value = Mathf.Max(1f, read(draft)), isDelayed = true, tooltip = title + " (" + unit + ")" };
            number.AddToClassList("rm-number"); valueBox.Add(number); valueBox.Add(Text(unit, "rm-unit"));
            label.RegisterCallback<ClickEvent>(_ => number.Focus());
            Slider slider = null;
            if (range > 0)
            {
                slider = new Slider(1f, Mathf.Max(range, read(draft))) { value = Mathf.Max(1f, read(draft)) };
                slider.AddToClassList("rm-slider"); row.Insert(1, slider);
                slider.tooltip = title + ": " + lowLabel + " / " + highLabel;
            }
            Action<float> change = value =>
            {
                if (syncing || planner.IsBusy) return;
                if (float.IsNaN(value) || float.IsInfinity(value)) value = read(draft);
                value = Mathf.Max(1f, value);
                number.SetValueWithoutNotify(value);
                if (slider != null) { slider.highValue = Mathf.Max(range, value); slider.SetValueWithoutNotify(value); }
                write(draft, value);
                pendingSettings = true;
                applyAt = Time.unscaledTime + 0.15f;
            };
            number.RegisterValueChangedCallback(evt => change(evt.newValue));
            if (slider != null) slider.RegisterValueChangedCallback(evt => change(evt.newValue));
            synchronize.Add(settings =>
            {
                float value = Mathf.Max(1f, read(settings));
                number.SetValueWithoutNotify(value);
                if (slider != null) { slider.highValue = Mathf.Max(range, value); slider.SetValueWithoutNotify(value); }
            });
        }

        private bool ApplyPendingSettings()
        {
            if (!pendingSettings) return !planner.IsBusy;
            if (!planner.ApplyMotionSettings(draft)) return false;
            pendingSettings = false;
            return true;
        }

        private void Update()
        {
            if (root == null || planner == null) return;
            if (appliedTheme != colorTheme) ApplyTheme();
            bool visible = planner.isActiveAndEnabled && planner.GameplayControlsEnabled;
            root.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
            if (!visible || settingsContainer == null) return;
            if (pendingSettings && !planner.IsBusy && Time.unscaledTime >= applyAt) ApplyPendingSettings();
            if (Time.unscaledTime < nextSync) return;
            nextSync = Time.unscaledTime + 0.1f;
            VisualElement focused = root.focusController?.focusedElement as VisualElement;
            bool editingNumber = IsNumericField(focused);
            if (!pendingSettings && !editingNumber)
            {
                syncing = true;
                draft = planner.GetMotionSettings();
                foreach (var sync in synchronize) sync(draft);
                int selectedIndex = Array.IndexOf(algorithms, planner.SelectedPlanner);
                if (selectedIndex >= 0 && selectedIndex < algorithm.choices.Count)
                    algorithm.SetValueWithoutNotify(algorithm.choices[selectedIndex]);
                SyncPlanningSettings();
                syncing = false;
            }
            RefreshStatus();
        }

        /// <summary>Changes panel appearance without rebuilding controls or modifying the robot.</summary>
        public void SetTheme(ColorTheme theme)
        {
            if (!Enum.IsDefined(typeof(ColorTheme), theme)) return;
            colorTheme = theme;
            PlayerPrefs.SetInt(ThemePreferenceKey, (int)theme);
            PlayerPrefs.Save();
            ApplyTheme();
        }

        private void ApplyTheme()
        {
            appliedTheme = colorTheme;
            if (root == null) return;
            root.EnableInClassList("rm-dark", colorTheme == ColorTheme.Dark);
            if (themeButton == null) return;
            themeButton.text = colorTheme == ColorTheme.Dark ? "Light" : "Dark";
            themeButton.tooltip = "Switch to " + (colorTheme == ColorTheme.Dark ? "light" : "dark") + " appearance";
        }

        private void RefreshStatus()
        {
            if (ActiveTab == ControlsTab.Viz && root.focusController?.focusedElement == null) SyncVizControls();
            bool busy = planner.IsBusy || planner.IsRrtSearchInProgress;
            settingsContainer.SetEnabled(!busy);
            scenePage.SetEnabled(!busy);
            planButton.SetEnabled(!busy);
            replayButton.SetEnabled(busy || (!algorithmChanged && planner.PlannedPath.Count > 1));
            bool planning = planner.CurrentMotionState == RobotBaseSixAxisMotionPlanner.MotionState.Planning || planner.IsRrtSearchInProgress;
            replayButton.text = busy ? (planning ? "Cancel planning" : "Stop motion") : "Replay";
            replayButton.tooltip = busy ? "Stop immediately without confirmation" : "Replay the stored motion";
            replayButton.EnableInClassList("rm-stop", busy);
            bool useHeaderStop = busy && (minimized || ActiveTab != ControlsTab.Motion);
            headerStopButton.style.display = useHeaderStop ? DisplayStyle.Flex : DisplayStyle.None;
            headerStopButton.text = planning ? "Cancel" : "Stop";
            headerStopButton.tooltip = replayButton.tooltip;
            stateLabel.style.display = useHeaderStop ? DisplayStyle.None : DisplayStyle.Flex;
            startPoseButton.SetEnabled(!busy);
            endPoseButton.SetEnabled(!busy);
            checkIkBeforePlan.SetValueWithoutNotify(planner.CheckEndpointIkBeforePlanning);
            SyncTrackControls();
            SyncReachControls();
            foreach (var toggle in root.Query<Toggle>(className: "rm-modern-switch").ToList())
                toggle.EnableInClassList("rm-switch--on", toggle.value);
            void ShowIk(Label label, Label compact, string text, RobotBaseSixAxisMotionPlanner.EndpointIkState result)
            {
                label.text = text;
                compact.text = CompactIkSummary(text, result);
                compact.tooltip = text;
                label.EnableInClassList("rm-ik-result--valid", result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Valid);
                label.EnableInClassList("rm-ik-result--invalid", result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Invalid || result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Collision);
                compact.EnableInClassList("rm-ik-result--valid", result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Valid);
                compact.EnableInClassList("rm-ik-result--invalid", result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Invalid || result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Collision);
            }
            ShowIk(startIkResult, startIkCompact, planner.StartIkSummary, planner.StartIkState);
            ShowIk(endIkResult, endIkCompact, planner.EndIkSummary, planner.EndIkState);
            bool IkFailed(RobotBaseSixAxisMotionPlanner.EndpointIkState result) =>
                result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Invalid || result == RobotBaseSixAxisMotionPlanner.EndpointIkState.Collision;
            if ((planner.StartIkSummary != previousStartIkSummary && IkFailed(planner.StartIkState)) ||
                (planner.EndIkSummary != previousEndIkSummary && IkFailed(planner.EndIkState))) ikDetails.value = true;
            previousStartIkSummary = planner.StartIkSummary; previousEndIkSummary = planner.EndIkSummary;
            var state = planner.CurrentMotionState;
            stateLabel.text = state == RobotBaseSixAxisMotionPlanner.MotionState.Complete ? "Complete" : state.ToString();
            stateDot.EnableInClassList("rm-state-dot--busy", busy);
            stateDot.EnableInClassList("rm-state-dot--failed", state == RobotBaseSixAxisMotionPlanner.MotionState.Failed);
            int count = planner.IsRrtSearchInProgress ? planner.CurrentRrtSearchIterations : planner.PlanningIterations;
            iterations.text = count > 0 || busy ? count.ToString("N0", CultureInfo.InvariantCulture) : "—";
            iterations.tooltip = count.ToString("N0", CultureInfo.InvariantCulture);
            planningTime.text = FormatSeconds(planner.LivePlanningTimeSeconds);
            travelTime.text = FormatSeconds(planner.EstimatedTravelTimeSeconds);
            waypointCount.text = planner.PlannedWaypointCount > 0 ? planner.PlannedWaypointCount.ToString("N0", CultureInfo.InvariantCulture) : "—";
            waypointCount.tooltip = string.IsNullOrEmpty(planner.TransferOptimizationSummary) ? "Actual planned waypoint count." : planner.TransferOptimizationSummary;
            waypointChange.text = planner.TransferWaypointsBeforeOptimization > 0 ? $"{planner.TransferWaypointsBeforeOptimization:N0} → {planner.PlannedWaypointCount:N0}" : string.Empty;
            waypointChange.style.display = planner.TransferWaypointsBeforeOptimization > 0 ? DisplayStyle.Flex : DisplayStyle.None;
            waypointChange.tooltip = waypointCount.tooltip;
            feedback.text = planner.Status;
            RefreshStepLog(state);
            RefreshDiagnosticControls();
            if (planner.Status == previousStatus && state == previousState) return;
            previousStatus = planner.Status;
            previousState = state;
            switch (state)
            {
                case RobotBaseSixAxisMotionPlanner.MotionState.Planning: message.text = "Searching for a collision-free route…"; break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Moving: message.text = "Following the planned route."; break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Complete: message.text = "Motion complete. Replay is available."; break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Failed: message.text = "Motion failed. See planner feedback."; break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Stopped: message.text = planner.LastStopWasPlanning ? "Planning cancelled. No partial route will run." : "Motion stopped. Current pose held."; break;
                default: message.text = "Ready to plan a route."; break;
            }
        }

        private void StopImmediately()
        {
            // No pending numeric edits or validation may delay a stop click.
            planner.StopMotion();
            RefreshStatus();
        }

        private void AddReachControls(VisualElement parent)
        {
            reachSettings = new Foldout { text = "Posture & reach", name = "reach-settings", value = false };
            reachSettings.AddToClassList("rm-adaptive-goal"); parent.Add(reachSettings);
            reachEnabled = new Toggle("Avoid overextension") { name = "reach-enabled" }; reachEnabled.AddToClassList("rm-edit-toggle"); reachSettings.Add(reachEnabled);
            reachBody = new VisualElement(); reachSettings.Add(reachBody);
            var modes = Element("rm-reach-modes"); reachBody.Add(modes);
            reachPrefer = new Button(() => { reachUiStrict = false; ApplyReachOptions(); }) { text = "Prefer compact", name = "reach-prefer" };
            reachStrict = new Button(() => { reachUiStrict = true; ApplyReachOptions(); }) { text = "Strict limit", name = "reach-strict" }; modes.Add(reachPrefer); modes.Add(reachStrict);
            reachLimitLabel = Text("Preferred max stretch", "rm-track-info"); reachBody.Add(reachLimitLabel);
            var row = Element("rm-reach-limit"); reachBody.Add(row);
            reachPercentSlider = new Slider(50, 100) { name = "reach-percent-slider" }; reachPercentSlider.AddToClassList("rm-reach-slider"); row.Add(reachPercentSlider);
            reachPercent = new FloatField { name = "reach-percent", isDelayed = true }; reachPercent.AddToClassList("rm-edit-dropdown"); row.Add(reachPercent); row.Add(Text("%", "rm-track-info"));
            reachPercentSlider.RegisterValueChangedCallback(evt => { reachPercent.SetValueWithoutNotify(evt.newValue); ApplyReachOptions(); });
            reachPercent.RegisterValueChangedCallback(_ => ApplyReachOptions());
            var pair = Element("rm-transfer-grid"); reachBody.Add(pair);
            reachStrength = new EnumField("Penalty strength", RobotBaseSixAxisMotionPlanner.StretchStrength.Medium) { name = "reach-strength" };
            reachScope = new EnumField("Apply to", RobotBaseSixAxisMotionPlanner.StretchScope.WholeTrajectory) { name = "reach-scope" };
            foreach (var field in new[] { reachStrength, reachScope }) { var cell = Element("rm-transfer-cell"); pair.Add(cell); field.AddToClassList("rm-edit-dropdown"); field.AddToClassList("rm-transfer-field"); cell.Add(field); }
            reachAllowE1 = new Toggle("Allow E1 to reduce stretch") { name = "reach-allow-e1", tooltip = "Selectable exception to arm-first: an active track-planning mode may search another E1 when reachable endpoints exceed the stretch setting." };
            reachAllowE1.AddToClassList("rm-edit-toggle"); reachBody.Add(reachAllowE1);
            var help = new Foldout { text = "Reach definition & endpoint policy", value = false }; help.AddToClassList("rm-ik-details"); reachBody.Add(help);
            help.Add(Text("Calibrated shoulder-to-wrist reach / upper-arm + forearm length. Tool length is excluded. Exact Start/End and wrist continuity remain protected. Strict limits can reject a reachable pose; no automatic exemptions.", "rm-planning-hint"));
            reachEnabled.RegisterValueChangedCallback(_ => ApplyReachOptions()); reachAllowE1.RegisterValueChangedCallback(_ => ApplyReachOptions());
            reachStrength.RegisterValueChangedCallback(_ => ApplyReachOptions()); reachScope.RegisterValueChangedCallback(_ => ApplyReachOptions());
            reachFeedback = Text(string.Empty, "rm-track-info"); reachFeedback.name = "reach-feedback"; reachSettings.Add(reachFeedback);
        }
        private void ApplyReachOptions()
        {
            bool accepted = planner.SetStretchOptions(reachEnabled.value, reachUiStrict, reachPercent.value,
                (RobotBaseSixAxisMotionPlanner.StretchStrength)reachStrength.value, (RobotBaseSixAxisMotionPlanner.StretchScope)reachScope.value, reachAllowE1.value);
            reachUiError = accepted ? null : "Stop before editing. Stretch must be a finite percentage between 50 and 100.";
            RefreshStatus();
        }
        private void SyncReachControls()
        {
            bool editable = !planner.IsBusy && !(planner.LinearTrack?.IsLocked ?? false);
            reachEnabled.SetValueWithoutNotify(planner.AvoidOverextension); reachEnabled.SetEnabled(editable); reachBody.SetEnabled(editable && planner.AvoidOverextension);
            reachUiStrict = planner.StrictStretchLimit; reachPrefer.EnableInClassList("rm-reach-selected", !reachUiStrict); reachStrict.EnableInClassList("rm-reach-selected", reachUiStrict);
            reachPercent.SetValueWithoutNotify(planner.MaximumStretchPercent); reachPercentSlider.SetValueWithoutNotify(planner.MaximumStretchPercent);
            reachStrength.SetValueWithoutNotify(planner.ReachPenaltyStrength); reachScope.SetValueWithoutNotify(planner.ReachScope); reachStrength.parent.style.display = reachUiStrict ? DisplayStyle.None : DisplayStyle.Flex;
            reachAllowE1.SetValueWithoutNotify(planner.AllowE1ForStretch); reachLimitLabel.text = reachUiStrict ? "Maximum allowed stretch" : "Preferred max stretch";
            reachSettings.text = planner.AvoidOverextension ? $"Posture & reach · {(reachUiStrict ? "strict" : "prefer")} ≤ {planner.MaximumStretchPercent:0.#}%" : "Posture & reach";
            reachFeedback.text = planner.AvoidOverextension ? string.IsNullOrEmpty(planner.StretchFeedback) ? "Peak stretch: — · Reset & plan to check." : planner.StretchFeedback : string.Empty;
            if (reachUiError != null) reachFeedback.text = reachUiError;
            reachFeedback.style.display = planner.AvoidOverextension || reachUiError != null ? DisplayStyle.Flex : DisplayStyle.None;
            if (peakStretchFeedback != null) { peakStretchFeedback.text = reachFeedback.text; peakStretchFeedback.style.display = planner.AvoidOverextension ? DisplayStyle.Flex : DisplayStyle.None; }
        }
        private void AddTrackControls(VisualElement parent)
        {
            trackChannels.Clear();
            trackSettings = new Foldout { text = "External axis E1 · locked position", value = false, name = "track-settings" };
            trackSettings.AddToClassList("rm-adaptive-goal"); parent.Add(trackSettings);
            trackEnabled = new Toggle("Enable calibrated track") { name = "track-enabled" };
            trackEnabled.AddToClassList("rm-edit-toggle"); trackSettings.Add(trackEnabled);
            trackEnabled.RegisterValueChangedCallback(evt =>
            {
                var track = planner.LinearTrack;
                if (track == null) return;
                TrackUiResult(track.TrySetEnabled(evt.newValue, out string error), error);
            });
            trackBody = new VisualElement(); trackSettings.Add(trackBody);
            stagedTransferMode = new Toggle("Track transfer → MoveL") { name = "track-staged-transfer",
                tooltip = "Arm first when both exact endpoints have IK here. Otherwise plan a safe arm/track transfer to a reachable station, stop E1, then execute final MoveL. Entire sequence is validated first." };
            stagedTransferMode.AddToClassList("rm-edit-toggle"); trackBody.Add(stagedTransferMode);
            stagedTransferSettings = new Foldout { text = "Track transfer settings", value = false, name = "track-transfer-settings" }; trackBody.Add(stagedTransferSettings);
            transferSpacing = new FloatField("Station step · mm") { name = "track-transfer-spacing", isDelayed = true };
            transferWaypoints = new IntegerField("Preferred max points") { name = "track-transfer-waypoints", isDelayed = true,
                tooltip = "Preferred maximum after optimization. Required poses and tolerance limits may need more points. No points are added just to reach this value." };
            transferStations = new IntegerField("Max stations") { name = "track-transfer-stations", isDelayed = true };
            transferApproach = new FloatField("Approach · mm") { name = "track-transfer-approach", isDelayed = true };
            transferIterations = new IntegerField("Posture trials") { name = "track-transfer-iterations", isDelayed = true };
            transferBudget = new FloatField("Budget · s") { name = "track-transfer-budget", isDelayed = true };
            transferSpeed = new FloatField("E1 speed · mm/s") { name = "track-transfer-speed", isDelayed = true };
            transferAcceleration = new FloatField("E1 accel · mm/s²") { name = "track-transfer-acceleration", isDelayed = true,
                tooltip = "Track acceleration and deceleration. Continuous E1 timing spans moving transfer samples; necessary stationary arm-reposition phases still hold E1." };
            var transferGrid = Element("rm-transfer-grid"); transferGrid.name = "track-transfer-grid"; stagedTransferSettings.Add(transferGrid);
            foreach (var field in new VisualElement[] { transferWaypoints, transferBudget, transferSpacing, transferStations, transferApproach, transferIterations, transferSpeed, transferAcceleration })
            {
                var cell = Element("rm-transfer-cell"); transferGrid.Add(cell);
                field.AddToClassList("rm-edit-dropdown"); field.AddToClassList("rm-transfer-field"); cell.Add(field);
            }
            stagedTransferSettings.Add(Text("Preferred maximum on next plan. Required poses and tolerance limits may retain more points.", "rm-track-info"));
            var corridorOptions = new Foldout { text = "Lower transfer corridor", value = false, name = "track-lower-corridor" }; stagedTransferSettings.Add(corridorOptions);
            transferPreferCorridor = new Toggle("Prefer lower corridor") { name = "track-prefer-corridor", tooltip = "Try downward retract, level travel below Start/approach, then upward approach before free-posture transfer. Path accuracy rounds the shoulders." };
            transferRequireCorridor = new Toggle("Require corridor") { name = "track-require-corridor", tooltip = "Fail if no validated lower corridor exists; do not fall back to a free-posture route." };
            transferCorridorDrop = new FloatField("Corridor drop · mm") { name = "track-corridor-drop", isDelayed = true };
            transferPreferCorridor.AddToClassList("rm-edit-toggle"); transferRequireCorridor.AddToClassList("rm-edit-toggle"); transferCorridorDrop.AddToClassList("rm-edit-dropdown");
            corridorOptions.Add(transferPreferCorridor); corridorOptions.Add(transferRequireCorridor); corridorOptions.Add(transferCorridorDrop);
            void CorridorOptionsChanged()
            {
                trackUiError = planner.SetLowerTransferCorridorOptions(transferPreferCorridor.value, transferRequireCorridor.value, transferCorridorDrop.value) ? null : "Stop before editing. Corridor drop must be 10–2000 mm.";
                RefreshStatus();
            }
            transferPreferCorridor.RegisterValueChangedCallback(_ => CorridorOptionsChanged()); transferRequireCorridor.RegisterValueChangedCallback(_ => CorridorOptionsChanged()); transferCorridorDrop.RegisterValueChangedCallback(_ => CorridorOptionsChanged());
            var optimizationOptions = new Foldout { text = "Waypoint optimization", name = "track-waypoint-optimization", value = false }; stagedTransferSettings.Add(optimizationOptions);
            var optimizationGrid = Element("rm-transfer-grid"); optimizationOptions.Add(optimizationGrid);
            transferReductionPosition = new FloatField("Transfer deviation · mm") { name = "track-reduction-position", isDelayed = true };
            transferReductionRotation = new FloatField("Tool deviation · °") { name = "track-reduction-rotation", isDelayed = true };
            transferOptimizationBudget = new FloatField("Shortcut budget · s") { name = "track-optimization-budget", isDelayed = true };
            foreach (var field in new[] { transferReductionPosition, transferReductionRotation, transferOptimizationBudget })
            {
                var cell = Element("rm-transfer-cell"); optimizationGrid.Add(cell); field.AddToClassList("rm-edit-dropdown"); field.AddToClassList("rm-transfer-field"); cell.Add(field);
            }
            void OptimizationOptionsChanged()
            {
                trackUiError = planner.SetTransferOptimizationOptions(transferReductionPosition.value, transferReductionRotation.value, transferOptimizationBudget.value) ? null :
                    "Stop before editing. Deviation ≥ 0.01 mm / 0.001°; shortcut budget 0.1–30 s; finite values required.";
                RefreshStatus();
            }
            transferReductionPosition.RegisterValueChangedCallback(_ => OptimizationOptionsChanged());
            transferReductionRotation.RegisterValueChangedCallback(_ => OptimizationOptionsChanged());
            transferOptimizationBudget.RegisterValueChangedCallback(_ => OptimizationOptionsChanged());
            transferWaypoints.RegisterValueChangedCallback(evt =>
            {
                trackUiError = planner.SetTransferTargetWaypoints(evt.newValue) ? null : "Stop before editing. Target waypoints must be between 2 and 10,000.";
                RefreshStatus();
            });
            void TransferOptionsChanged()
            {
                bool accepted = planner.SetStagedTransferOptions(stagedTransferMode.value, transferSpacing.value, transferStations.value,
                    transferApproach.value, transferIterations.value, transferBudget.value, transferSpeed.value);
                trackUiError = accepted ? null : "Stop before editing. Transfer spacing/speed ≥ 1, approach ≥ 10 mm, budget ≥ 0.1 s; values must be finite.";
                RefreshStatus();
            }
            stagedTransferMode.RegisterValueChangedCallback(_ => TransferOptionsChanged());
            transferSpacing.RegisterValueChangedCallback(_ => TransferOptionsChanged()); transferStations.RegisterValueChangedCallback(_ => TransferOptionsChanged());
            transferApproach.RegisterValueChangedCallback(_ => TransferOptionsChanged()); transferIterations.RegisterValueChangedCallback(_ => TransferOptionsChanged());
            transferBudget.RegisterValueChangedCallback(_ => TransferOptionsChanged()); transferSpeed.RegisterValueChangedCallback(_ => TransferOptionsChanged());
            transferAcceleration.RegisterValueChangedCallback(evt =>
            {
                trackUiError = planner.SetTransferTrackAcceleration(evt.newValue) ? null : "Stop before editing. E1 acceleration must be finite and ≥ 1 mm/s².";
                RefreshStatus();
            });
            coordinatedMode = new Toggle("Coordinated MoveL") { name = "track-coordinated-movel",
                tooltip = "Arm first: keep E1 fixed when both exact endpoints have IK here. Only an unreachable endpoint enables coordinated track search. Collision/route failure alone never enables track fallback." };
            coordinatedMode.AddToClassList("rm-edit-toggle"); trackBody.Add(coordinatedMode);
            coordinatedLimits = new Foldout { text = "Coordinated search settings", value = false, name = "track-coordinated-settings" }; trackBody.Add(coordinatedLimits);
            coordinatedCartesianStep = new FloatField("TCP step · mm") { name = "track-coordinated-tcp-step", isDelayed = true };
            coordinatedTrackStep = new FloatField("E1 step · mm") { name = "track-coordinated-e1-step", isDelayed = true };
            coordinatedBeam = new IntegerField("Candidates") { name = "track-coordinated-beam", isDelayed = true };
            coordinatedBudget = new FloatField("Budget · s") { name = "track-coordinated-budget", isDelayed = true };
            coordinatedTrackSpeed = new FloatField("E1 · mm/s") { name = "track-coordinated-speed", isDelayed = true };
            foreach (var field in new VisualElement[] { coordinatedCartesianStep, coordinatedTrackStep, coordinatedBeam, coordinatedBudget, coordinatedTrackSpeed })
            { field.AddToClassList("rm-edit-dropdown"); coordinatedLimits.Add(field); }
            void CoordinatedOptionsChanged()
            {
                bool accepted = planner.SetCoordinatedMoveLOptions(coordinatedMode.value, coordinatedCartesianStep.value, coordinatedTrackStep.value,
                    coordinatedBeam.value, coordinatedBudget.value, coordinatedTrackSpeed.value);
                trackUiError = accepted ? null : "Stop before editing. Coordinated step/speed must be finite and ≥ 1; budget ≥ 0.1 s.";
                RefreshStatus();
            }
            coordinatedMode.RegisterValueChangedCallback(_ => CoordinatedOptionsChanged());
            coordinatedCartesianStep.RegisterValueChangedCallback(_ => CoordinatedOptionsChanged()); coordinatedTrackStep.RegisterValueChangedCallback(_ => CoordinatedOptionsChanged());
            coordinatedBeam.RegisterValueChangedCallback(_ => CoordinatedOptionsChanged()); coordinatedBudget.RegisterValueChangedCallback(_ => CoordinatedOptionsChanged());
            coordinatedTrackSpeed.RegisterValueChangedCallback(_ => CoordinatedOptionsChanged());
            automaticStation = new Toggle("Auto station · MoveL") { name = "track-auto-station",
                tooltip = "Search another fixed E1 only when Start or End has no exact IK at the current station. If both have IK, plan arm-only here. Station trials are setup previews." };
            automaticStation.AddToClassList("rm-edit-toggle"); trackBody.Add(automaticStation);
            stationLimits = new Foldout { text = "Station search limits", value = false, name = "track-station-limits" }; trackBody.Add(stationLimits);
            stationSpacing = new FloatField("Spacing · mm") { name = "track-station-spacing", isDelayed = true };
            stationCandidates = new IntegerField("Max stations") { name = "track-station-candidates", isDelayed = true };
            stationRoutes = new IntegerField("Max routes") { name = "track-station-routes", isDelayed = true };
            stationBudget = new FloatField("Budget · s") { name = "track-station-budget", isDelayed = true };
            foreach (var field in new VisualElement[] { stationSpacing, stationCandidates, stationRoutes, stationBudget })
            { field.AddToClassList("rm-edit-dropdown"); stationLimits.Add(field); }
            void StationOptionsChanged()
            {
                bool accepted = planner.SetAutomaticStationOptions(automaticStation.value, stationSpacing.value, stationCandidates.value, stationRoutes.value, stationBudget.value);
                trackUiError = accepted ? null : "Station search needs finite spacing ≥ 1 mm and budget ≥ 0.1 s; stop before editing.";
                RefreshStatus();
            }
            automaticStation.RegisterValueChangedCallback(_ => StationOptionsChanged());
            stationSpacing.RegisterValueChangedCallback(_ => StationOptionsChanged()); stationCandidates.RegisterValueChangedCallback(_ => StationOptionsChanged());
            stationRoutes.RegisterValueChangedCallback(_ => StationOptionsChanged()); stationBudget.RegisterValueChangedCallback(_ => StationOptionsChanged());
            var positionGroup = Element("rm-track-position"); trackBody.Add(positionGroup);
            trackPosition = new FloatField("E1 position · mm") { name = "track-position", isDelayed = true,
                tooltip = "Setup preview only. Track stays locked during arm planning and playback; changing E1 discards saved motion." };
            trackPosition.AddToClassList("rm-edit-dropdown"); positionGroup.Add(trackPosition);
            trackPosition.RegisterValueChangedCallback(evt =>
            {
                var track = planner.LinearTrack;
                if (track == null) return;
                TrackUiResult(track.TrySetPosition(evt.newValue, out string error), error);
            });
            trackPositionSlider = new Slider(0, 1) { name = "track-position-slider", tooltip = trackPosition.tooltip };
            trackPositionSlider.AddToClassList("rm-slider"); positionGroup.Add(trackPositionSlider);
            trackPositionSlider.RegisterValueChangedCallback(evt =>
            {
                if (synchronizingTrackPosition) return;
                var track = planner.LinearTrack; if (track == null) return;
                TrackUiResult(track.TrySetPosition(evt.newValue, out string error), error);
            });
            var range = Element("rm-track-range"); positionGroup.Add(range);
            trackRangeHome = Text(string.Empty, "rm-track-range-label"); trackRangeEnd = Text(string.Empty, "rm-track-range-label"); range.Add(trackRangeHome); range.Add(trackRangeEnd);
            var stations = Element("rm-track-stations"); trackBody.Add(stations);
            void SelectStation(bool home)
            {
                var track = planner.LinearTrack;
                if (track == null) return;
                TrackUiResult(track.TrySetPosition(home ? track.MinimumMillimeters : track.MaximumMillimeters, out string error), error);
            }
            stations.Add(new Button(() => SelectStation(true)) { text = "Preview Home", name = "track-home", tooltip = "Select Home during setup; no carriage travel is planned." });
            stations.Add(new Button(() => SelectStation(false)) { text = "Preview End", name = "track-end", tooltip = "Select End during setup; this is separate from the TCP End target." });
            trackBody.Add(Text("Calibration correction channels", "rm-track-info"));
            var channels = Element("rm-track-channels"); trackBody.Add(channels);
            var names = new[] { "X", "Y", "Z", "Rx", "Ry", "Rz" };
            for (int i = 0; i < names.Length; i++)
            {
                int index = i;
                var toggle = new Toggle(names[i]) { name = "track-correction-" + names[i].ToLowerInvariant() };
                toggle.AddToClassList("rm-edit-toggle"); toggle.AddToClassList("rm-track-channel");
                channels.Add(toggle); trackChannels.Add(toggle);
                toggle.RegisterValueChangedCallback(evt =>
                {
                    var track = planner.LinearTrack; var profile = track?.Profile;
                    if (profile == null) return;
                    var c = profile.Corrections;
                    switch (index) { case 0: c.X = evt.newValue; break; case 1: c.Y = evt.newValue; break; case 2: c.Z = evt.newValue; break;
                        case 3: c.Rx = evt.newValue; break; case 4: c.Ry = evt.newValue; break; case 5: c.Rz = evt.newValue; break; }
                    profile.Corrections = c;
                    TrackUiResult(track.TrySetProfile(profile, out string error), error);
                });
            }
            trackInfo = Text(string.Empty, "rm-track-info"); trackSettings.Add(trackInfo);
            stationSummary = Text(string.Empty, "rm-track-info"); stationSummary.name = "track-station-summary"; trackSettings.Add(stationSummary);
            coordinatedFeedback = Text(string.Empty, "rm-track-info"); coordinatedFeedback.name = "track-coordinated-summary"; trackSettings.Add(coordinatedFeedback);
            trackReachabilityFeedback = Text(string.Empty, "rm-track-info"); trackReachabilityFeedback.name = "track-reachability-decision"; trackSettings.Add(trackReachabilityFeedback);
            stagedTransferDetails = new Foldout { text = "Last transfer diagnostics", value = false, name = "track-transfer-details" }; trackSettings.Add(stagedTransferDetails);
            transferDetails = Text(string.Empty, "rm-track-info"); stagedTransferDetails.Add(transferDetails);
            stationDetails = new Foldout { text = "Station search details", value = false, name = "track-station-details" }; trackSettings.Add(stationDetails);
            stationLog = Text(string.Empty, "rm-track-info"); stationDetails.Add(stationLog);
            trackError = Text(string.Empty, "rm-track-error"); trackError.name = "track-error"; trackSettings.Add(trackError);
        }

        private void TrackUiResult(bool success, string error)
        {
            trackUiError = success ? null : error;
            if (success) planner.NotifySceneEdited();
            RefreshStatus();
        }

        private void SyncTrackControls()
        {
            var track = planner.LinearTrack;
            trackSettings.style.display = track != null ? DisplayStyle.Flex : DisplayStyle.None;
            if (track == null) return;
            trackEnabled.SetValueWithoutNotify(track.TrackEnabled);
            trackEnabled.SetEnabled(!planner.IsBusy && !track.IsLocked);
            trackBody.SetEnabled(track.TrackEnabled && !planner.IsBusy && !track.IsLocked);
            trackPosition.SetValueWithoutNotify(track.SelectedE1Millimeters);
            synchronizingTrackPosition = true;
            try
            {
                trackPositionSlider.lowValue = track.MinimumMillimeters; trackPositionSlider.highValue = track.MaximumMillimeters;
                trackPositionSlider.SetValueWithoutNotify(track.SelectedE1Millimeters);
            }
            finally { synchronizingTrackPosition = false; }
            trackRangeHome.text = $"Home · {track.MinimumMillimeters:0.##} mm"; trackRangeEnd.text = $"End · {track.MaximumMillimeters:0.##} mm";
            automaticStation.SetValueWithoutNotify(planner.AutomaticTrackStation);
            bool moveL = planner.SelectedPlanner == RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.CartesianLinear;
            stagedTransferMode.SetValueWithoutNotify(planner.StagedTrackTransferEnabled); stagedTransferMode.SetEnabled(moveL);
            stagedTransferSettings.SetEnabled(moveL && planner.StagedTrackTransferEnabled);
            transferSpacing.SetValueWithoutNotify(planner.TransferStationSpacingMillimeters); transferStations.SetValueWithoutNotify(planner.TransferMaximumStations);
            transferApproach.SetValueWithoutNotify(planner.TransferApproachMillimeters); transferIterations.SetValueWithoutNotify(planner.TransferPostureIterations);
            transferBudget.SetValueWithoutNotify(planner.CoordinatedSearchSeconds); transferSpeed.SetValueWithoutNotify(planner.CoordinatedTrackSpeedMillimetersPerSecond);
            transferAcceleration.SetValueWithoutNotify(planner.TransferTrackAcceleration);
            transferWaypoints.SetValueWithoutNotify(planner.TransferTargetWaypoints);
            transferPreferCorridor.SetValueWithoutNotify(planner.PreferLowerTransferCorridor); transferRequireCorridor.SetValueWithoutNotify(planner.RequireLowerTransferCorridor); transferCorridorDrop.SetValueWithoutNotify(planner.TransferCorridorDropMm);
            transferReductionPosition.SetValueWithoutNotify(planner.TransferReductionPositionMm);
            transferReductionRotation.SetValueWithoutNotify(planner.TransferReductionRotationDegrees);
            transferOptimizationBudget.SetValueWithoutNotify(planner.TransferOptimizationBudgetSeconds);
            coordinatedMode.SetValueWithoutNotify(planner.CoordinatedMoveLEnabled); coordinatedMode.SetEnabled(moveL && !planner.StagedTrackTransferEnabled);
            coordinatedLimits.SetEnabled(moveL && !planner.StagedTrackTransferEnabled && planner.CoordinatedMoveLEnabled);
            coordinatedCartesianStep.SetValueWithoutNotify(planner.CoordinatedCartesianStepMillimeters); coordinatedTrackStep.SetValueWithoutNotify(planner.CoordinatedTrackStepMillimeters);
            coordinatedBeam.SetValueWithoutNotify(planner.CoordinatedBeamWidth); coordinatedBudget.SetValueWithoutNotify(planner.CoordinatedSearchSeconds);
            coordinatedTrackSpeed.SetValueWithoutNotify(planner.CoordinatedTrackSpeedMillimetersPerSecond);
            automaticStation.SetEnabled(moveL && !planner.StagedTrackTransferEnabled && !planner.CoordinatedMoveLEnabled);
            stationLimits.SetEnabled(moveL && !planner.StagedTrackTransferEnabled && !planner.CoordinatedMoveLEnabled && planner.AutomaticTrackStation);
            stationSpacing.SetValueWithoutNotify(planner.StationSpacingMillimeters); stationCandidates.SetValueWithoutNotify(planner.MaximumStationCandidates);
            stationRoutes.SetValueWithoutNotify(planner.MaximumStationRouteAttempts); stationBudget.SetValueWithoutNotify(planner.StationSearchSeconds);
            stationSummary.text = planner.StationSearchSummary;
            stationSummary.style.display = string.IsNullOrEmpty(stationSummary.text) ? DisplayStyle.None : DisplayStyle.Flex;
            coordinatedFeedback.text = planner.CoordinatedMoveLSummary;
            coordinatedFeedback.style.display = string.IsNullOrEmpty(coordinatedFeedback.text) ? DisplayStyle.None : DisplayStyle.Flex;
            trackReachabilityFeedback.text = planner.TrackReachabilitySummary;
            trackReachabilityFeedback.style.display = string.IsNullOrEmpty(trackReachabilityFeedback.text) ? DisplayStyle.None : DisplayStyle.Flex;
            transferDetails.text = string.Join("\n", planner.TrackTransferLog);
            stagedTransferDetails.style.display = planner.TrackTransferLog.Count == 0 ? DisplayStyle.None : DisplayStyle.Flex;
            stationLog.text = string.Join("\n", planner.StationSearchLog);
            stationDetails.style.display = planner.StationSearchLog.Count == 0 ? DisplayStyle.None : DisplayStyle.Flex;
            var profile = track.Profile;
            if (profile != null)
            {
                var c = profile.Corrections;
                var enabled = new[] { c.X, c.Y, c.Z, c.Rx, c.Ry, c.Rz };
                for (int i = 0; i < enabled.Length; i++) trackChannels[i].SetValueWithoutNotify(enabled[i]);
            }
            bool useCoordinated = moveL && planner.CoordinatedMoveLEnabled && !planner.TrackHeldForReachableEndpoints;
            bool useTransfer = moveL && planner.StagedTrackTransferEnabled && !planner.TrackHeldForReachableEndpoints;
            trackSettings.text = useTransfer ? "External axis E1 · transfer → MoveL" : useCoordinated ? "External axis E1 · coordinated MoveL" : "External axis E1 · locked position";
            trackInfo.text = $"{(track.IsStopHeld ? "E1 held" : useTransfer ? "Track transfer" : useCoordinated ? "Coordinated E1" : track.IsLocked ? "E1 locked" : "Setup preview")} · {track.MinimumMillimeters:F1}–{track.MaximumMillimeters:F1} mm\n" +
                $"Home → {profile?.Intermediate?.Length ?? 0} calibration station(s) → End. Configure poses, references and collision geometry in the track component Inspector.";
            trackError.text = trackUiError ?? track.LastError ?? string.Empty;
            trackError.style.display = string.IsNullOrEmpty(trackError.text) ? DisplayStyle.None : DisplayStyle.Flex;
        }

        private void ShowPlannerMenu()
        {
            if (algorithm == null || !algorithm.enabledInHierarchy || planner.IsBusy || planner.IsRrtSearchInProgress) return;
            plannerMenuRoot?.RemoveFromHierarchy();
            var menu = new GenericDropdownMenu();
            for (int i = 0; i < algorithms.Length; i++)
            {
                if (i == 0 || i == 3)
                {
                    if (i == 3) menu.AddSeparator(string.Empty);
                    menu.AddDisabledItem(i == 0 ? "Cartesian & Grid planners" : "Joint-space planners", false);
                    menu.contentContainer.ElementAt(menu.contentContainer.childCount - 1).AddToClassList("rm-planner-group");
                }
                int choice = i;
                menu.AddItem(algorithm.choices[i], planner.SelectedPlanner == algorithms[i], () =>
                {
                    if (!planner.IsBusy && !planner.IsRrtSearchInProgress && algorithm.enabledInHierarchy) algorithm.index = choice;
                });
                var item = menu.contentContainer.ElementAt(menu.contentContainer.childCount - 1);
                item.AddToClassList("rm-planner-choice");
                item.userData = algorithms[i];
            }
            ShowThemedMenu(menu, algorithm);
        }

        private void ShowThemedMenu(GenericDropdownMenu menu, VisualElement field)
        {
            if (!field.enabledInHierarchy || planner.IsBusy || planner.IsRrtSearchInProgress) return;
            plannerMenuRoot?.RemoveFromHierarchy();
            plannerMenuRoot = menu.contentContainer;
            while (plannerMenuRoot.parent != null) plannerMenuRoot = plannerMenuRoot.parent;
            plannerMenuRoot.name = field.name + "-popup";
            plannerMenuRoot.AddToClassList("rm-planner-popup");
            plannerMenuRoot.EnableInClassList("rm-dark", colorTheme == ColorTheme.Dark);
            var sheet = Resources.Load<StyleSheet>("RobotMotionControls");
            if (sheet != null) plannerMenuRoot.styleSheets.Add(sheet);
            var input = field.Q(className: "unity-base-field__input");
            menu.DropDown(input != null ? input.worldBound : field.worldBound, field, true);
        }

        private void RefreshStepLog(RobotBaseSixAxisMotionPlanner.MotionState state)
        {
            bool endpointsValid = planner.StartIkState == RobotBaseSixAxisMotionPlanner.EndpointIkState.Valid &&
                planner.EndIkState == RobotBaseSixAxisMotionPlanner.EndpointIkState.Valid;
            bool endpointFailed = planner.StartIkState == RobotBaseSixAxisMotionPlanner.EndpointIkState.Collision ||
                planner.StartIkState == RobotBaseSixAxisMotionPlanner.EndpointIkState.Invalid ||
                planner.EndIkState == RobotBaseSixAxisMotionPlanner.EndpointIkState.Collision ||
                planner.EndIkState == RobotBaseSixAxisMotionPlanner.EndpointIkState.Invalid;
            string checks = $"Start: {CompactIkSummary(planner.StartIkSummary, planner.StartIkState)} · End: {CompactIkSummary(planner.EndIkSummary, planner.EndIkState)}";
            SetStep(endpointStep, endpointStepTitle, endpointStepDetail,
                !planner.CheckEndpointIkBeforePlanning ? "Endpoint precheck disabled" : endpointsValid ? "Endpoint checks passed" : endpointFailed ? "Endpoint check failed" : "Endpoint IK check",
                (endpointsValid || endpointFailed ? "Last check · " : string.Empty) + checks, endpointFailed ? "failed" : endpointsValid ? "passed" : "neutral");
            string raw = planner.Status ?? string.Empty;
            string route = algorithm.value + $" · {planner.PlannedWaypointCount:N0} waypoints";
            var posture = Regex.Match(raw, @"Selected Start IK posture ([\d]+/[\d]+)");
            if (posture.Success) route += " · Start IK " + posture.Groups[1].Value;
            motionStep.style.display = DisplayStyle.None;
            switch (state)
            {
                case RobotBaseSixAxisMotionPlanner.MotionState.Planning:
                    string stage = raw.IndexOf("scan", StringComparison.OrdinalIgnoreCase) >= 0 ? "Scanning occupancy" :
                        raw.IndexOf("intermediate pose", StringComparison.OrdinalIgnoreCase) >= 0 ? "Trying intermediate pose adjustments" : "Searching for a collision-free route";
                    var segment = Regex.Match(raw, @"(?:Segment \[([^\]]+)\]|^(.+?): searching with)");
                    if (segment.Success) stage += " · " + (segment.Groups[1].Success ? segment.Groups[1].Value : segment.Groups[2].Value);
                    SetStep(planStep, planStepTitle, planStepDetail, "Planning route", stage, "current");
                    feedbackNext.text = "Cancel planning stops the search and discards the unfinished route.";
                    break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Moving:
                case RobotBaseSixAxisMotionPlanner.MotionState.Complete:
                    SetStep(planStep, planStepTitle, planStepDetail, "Route validated", route, "passed");
                    motionStep.style.display = DisplayStyle.Flex;
                    bool complete = state == RobotBaseSixAxisMotionPlanner.MotionState.Complete;
                    SetStep(motionStep, motionStepTitle, motionStepDetail, complete ? "Motion complete" : "Executing motion",
                        complete ? "The robot reached the planned End pose." : "Following the validated trajectory.", complete ? "passed" : "current");
                    feedbackNext.text = complete ? "Replay the route or reset and plan again." : "Stop motion holds the current simulated robot pose.";
                    break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Stopped:
                    SetStep(planStep, planStepTitle, planStepDetail, planner.LastStopWasPlanning ? "Planning cancelled" : "Route retained",
                        planner.LastStopWasPlanning ? "Search stopped · partial route discarded" : route, "neutral");
                    if (!planner.LastStopWasPlanning)
                    {
                        motionStep.style.display = DisplayStyle.Flex;
                        SetStep(motionStep, motionStepTitle, motionStepDetail, "Motion stopped", "Robot held at its current pose.", "neutral");
                    }
                    feedbackNext.text = planner.LastStopWasPlanning ? "Adjust settings or start a new plan." : "Replay the retained route or plan again.";
                    break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Failed:
                    string title = "Planning failed", detail = "See Technical details for the full diagnostic.";
                    if (raw.IndexOf("continuous IK branch", StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        title = "No continuous IK route";
                        var joint = Regex.Match(raw, @"\(J\d+: [^)]+\)");
                        var jump = Regex.Match(raw, @"nearest joint jump [\d.]+ degrees exceeds [\d.]+ degrees");
                        detail = joint.Success ? joint.Value.Trim('(', ')') + " · maximum joint step 5°" : jump.Success ? jump.Value : "Joint continuity limit exceeded.";
                    }
                    else if (raw.IndexOf("No free grid route", StringComparison.OrdinalIgnoreCase) >= 0 || raw.IndexOf("No traversable route", StringComparison.OrdinalIgnoreCase) >= 0)
                    { title = "No traversable scanned route"; detail = "The scanned free cells could not connect this segment."; }
                    else if (raw.IndexOf("unreachable", StringComparison.OrdinalIgnoreCase) >= 0)
                    { title = "Target pose unreachable"; detail = "IK could not find a solution for the requested pose."; }
                    else if (Regex.IsMatch(raw, @"\b(?:singular|singularity)\b", RegexOptions.IgnoreCase))
                    { title = "Singularity check failed"; detail = "No route satisfying the singularity margin was found."; }
                    else if (raw.IndexOf("Path ended short", StringComparison.OrdinalIgnoreCase) >= 0)
                    { title = "End target was not reached"; detail = "Playback ended outside the target pose tolerance."; }
                    else if (raw.IndexOf("wall-time limit", StringComparison.OrdinalIgnoreCase) >= 0 || raw.IndexOf("route attempt limit", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        raw.IndexOf("node limit", StringComparison.OrdinalIgnoreCase) >= 0 || raw.IndexOf("deadline", StringComparison.OrdinalIgnoreCase) >= 0 || raw.IndexOf("budget", StringComparison.OrdinalIgnoreCase) >= 0)
                    { title = "Search limit reached"; detail = "No complete route was found within the search budget."; }
                    else if (raw.IndexOf("geometry is missing", StringComparison.OrdinalIgnoreCase) >= 0)
                    { title = "Collision geometry missing"; detail = "Assign the robot and obstacle colliders before planning."; }
                    else if (raw.IndexOf("collision", StringComparison.OrdinalIgnoreCase) >= 0 || raw.IndexOf("clearance", StringComparison.OrdinalIgnoreCase) >= 0)
                    { title = "No collision-free route"; detail = "A pose or route did not satisfy collision and clearance checks."; }
                    if (planner.LastPlanReport != null)
                    { title = PlanDiagnosticReport.StatusTitle(planner.LastPlanReport.StopCode); detail = "Observed candidate blockers are listed below; the stop reason does not prove all routes impossible."; }
                    SetStep(planStep, planStepTitle, planStepDetail, title, detail, "failed");
                    feedbackNext.text = "Inspect Technical details and adjust the route or intermediate pose options.";
                    break;
                default:
                    SetStep(planStep, planStepTitle, planStepDetail, planner.PlannedPath.Count > 1 ? "Route available" : "Ready to plan",
                        planner.PlannedPath.Count > 1 ? route : "Start → End", "neutral");
                    feedbackNext.text = string.Empty;
                    break;
            }
            // Successful rows keep only the differentiating information; the full
            // descriptions remain available in tooltips and the existing details.
            endpointStepTitle.tooltip = endpointStepTitle.text;
            endpointStepDetail.tooltip = endpointStepDetail.text;
            if (endpointsValid)
            {
                endpointStepTitle.text = "Endpoint IK passed";
                endpointStepDetail.text = "Start " + CompactIkSummary(planner.StartIkSummary, planner.StartIkState).Replace("Valid · ", string.Empty) +
                    " · End " + CompactIkSummary(planner.EndIkSummary, planner.EndIkState).Replace("Valid · ", string.Empty);
            }
            planStepDetail.tooltip = planStepDetail.text;
            motionStepDetail.tooltip = motionStepDetail.text;
            if (state == RobotBaseSixAxisMotionPlanner.MotionState.Moving || state == RobotBaseSixAxisMotionPlanner.MotionState.Complete ||
                (state == RobotBaseSixAxisMotionPlanner.MotionState.Stopped && !planner.LastStopWasPlanning))
                planStepDetail.text = $"{planner.PlannedWaypointCount:N0} pts" + (posture.Success ? " · IK " + posture.Groups[1].Value : string.Empty) +
                    (planner.UsedAdaptiveGoal ? " · adjusted End" : string.Empty);
            if (state == RobotBaseSixAxisMotionPlanner.MotionState.Complete) motionStepDetail.text = "End reached";
            else if (state == RobotBaseSixAxisMotionPlanner.MotionState.Moving) motionStepDetail.text = "Stop available";
            else if (state == RobotBaseSixAxisMotionPlanner.MotionState.Stopped) motionStepDetail.text = "Current pose held";
            feedbackNext.style.display = state == RobotBaseSixAxisMotionPlanner.MotionState.Failed || state == RobotBaseSixAxisMotionPlanner.MotionState.Stopped
                ? DisplayStyle.Flex : DisplayStyle.None;
            string summary = state == RobotBaseSixAxisMotionPlanner.MotionState.Complete ?
                (endpointsValid && planner.CheckEndpointIkBeforePlanning ? "Motion complete · 3 checks passed" : "Motion complete · route validated") :
                state == RobotBaseSixAxisMotionPlanner.MotionState.Moving ? "Executing validated motion" :
                state == RobotBaseSixAxisMotionPlanner.MotionState.Planning ? "Planning route · view progress" :
                state == RobotBaseSixAxisMotionPlanner.MotionState.Failed ? planStepTitle.text :
                state == RobotBaseSixAxisMotionPlanner.MotionState.Stopped ? planner.LastStopWasPlanning ? "Planning cancelled" : "Motion stopped" : "Ready to plan";
            feedbackSummary.text = summary;
            feedbackSummary.tooltip = feedbackNext.text;
            feedbackSummary.EnableInClassList("rm-feedback-summary--failed", state == RobotBaseSixAxisMotionPlanner.MotionState.Failed);
            // Expand only on a new failure; ordinary refreshes preserve user choice.
            if (state == RobotBaseSixAxisMotionPlanner.MotionState.Failed && (previousState != state || previousStatus != planner.Status))
                feedbackSummary.value = true;
        }

        private void ConfigureModernSwitches()
        {
            // Keep native Toggle input, keyboard navigation and value callbacks.
            // Never restyle Foldout toggles as switches.
            foreach (var toggle in root.Query<Toggle>(className: "rm-edit-toggle").ToList())
            {
                toggle.AddToClassList("rm-modern-switch");
                toggle.EnableInClassList("rm-switch--on", toggle.value);
                toggle.RegisterValueChangedCallback(evt => toggle.EnableInClassList("rm-switch--on", evt.newValue));
            }
        }

        private static VisualElement AddStep(VisualElement parent, string name, out Label title, out Label detail)
        {
            var row = Element("rm-step"); row.name = name; parent.Add(row);
            row.Add(Element("rm-step-dot"));
            var content = Element("rm-step-content"); row.Add(content);
            title = Text(string.Empty, "rm-step-title"); content.Add(title);
            detail = Text(string.Empty, "rm-step-detail"); content.Add(detail);
            return row;
        }

        private static void SetStep(VisualElement row, Label title, Label detail, string heading, string body, string state)
        {
            title.text = heading; detail.text = body;
            row.EnableInClassList("rm-step--passed", state == "passed");
            row.EnableInClassList("rm-step--current", state == "current");
            row.EnableInClassList("rm-step--failed", state == "failed");
        }

        private void ResizePanel(GeometryChangedEvent evt)
        {
            viewportSize = evt.newRect.size;
            float width = Mathf.Min(360f, Mathf.Max(0f, viewportSize.x - ScreenMargin * 2f));
            windowPanel.style.width = width;
            windowPanel.style.maxHeight = Mathf.Max(0f, viewportSize.y - ScreenMargin * 2f);
            if (!hasPanelPosition)
            {
                panelPosition = new Vector2(viewportSize.x - width - ScreenMargin, ScreenMargin);
                hasPanelPosition = true;
            }
            UpdateScrollHeight(); ConstrainPanelToViewport();
        }

        private static string CompactIkSummary(string text, RobotBaseSixAxisMotionPlanner.EndpointIkState state)
        {
            if (state == RobotBaseSixAxisMotionPlanner.EndpointIkState.NotChecked) return "Not checked";
            if (state == RobotBaseSixAxisMotionPlanner.EndpointIkState.Collision) return "Collision";
            if (state == RobotBaseSixAxisMotionPlanner.EndpointIkState.Invalid) return "Invalid";
            int separator = text.IndexOf("— ", StringComparison.Ordinal);
            if (separator >= 0)
            {
                string remainder = text.Substring(separator + 2);
                int space = remainder.IndexOf(' ');
                if (space > 0 && int.TryParse(remainder.Substring(0, space), out int count)) return $"Valid · {count} IK";
            }
            return "Valid";
        }

        private void UpdateScrollHeight()
        {
            if (scrollPanel == null || windowPanel == null || viewportSize.y <= 0f) return;
            float Height(VisualElement element)
            {
                if (element == null || element.resolvedStyle.display == DisplayStyle.None) return 0f;
                var style = element.resolvedStyle;
                return float.IsNaN(style.height) ? 0f : style.height + style.marginTop + style.marginBottom;
            }
            var windowStyle = windowPanel.resolvedStyle;
            float chrome = Height(panelHeader) + Height(panelTabs) + Height(actionBar) + Height(message) +
                windowStyle.paddingTop + windowStyle.paddingBottom + windowStyle.borderTopWidth + windowStyle.borderBottomWidth;
            scrollPanel.style.maxHeight = Mathf.Max(0f, viewportSize.y - ScreenMargin * 2f - chrome);
        }

        /// <summary>Folds the panel to its draggable header without interrupting robot motion.</summary>
        public void SetMinimized(bool value)
        {
            minimized = value;
            if (panelBody == null) return;
            panelBody.style.display = minimized ? DisplayStyle.None : DisplayStyle.Flex;
            windowPanel.EnableInClassList("rm-panel--minimized", minimized);
            minimizeButton.text = minimized ? "□" : "−";
            minimizeButton.tooltip = minimized ? "Restore panel" : "Minimize panel";
            scrollPanel.verticalScrollerVisibility = minimized ? ScrollerVisibility.Hidden : ScrollerVisibility.Auto;
            RefreshStatus();
            windowPanel.schedule.Execute(() => { UpdateScrollHeight(); ConstrainPanelToViewport(); });
        }

        /// <summary>Moves the panel in UI coordinates and clamps it to the available viewport.</summary>
        public void SetPanelPosition(Vector2 position)
        {
            if (float.IsNaN(position.x) || float.IsNaN(position.y) || float.IsInfinity(position.x) || float.IsInfinity(position.y)) return;
            panelPosition = position;
            hasPanelPosition = true;
            ConstrainPanelToViewport();
        }

        private void ConstrainPanelToViewport()
        {
            if (windowPanel == null || viewportSize.x <= 0f || viewportSize.y <= 0f) return;
            float width = windowPanel.layout.width;
            float height = windowPanel.layout.height;
            if (float.IsNaN(width) || width <= 0f) width = Mathf.Min(360f, Mathf.Max(0f, viewportSize.x - ScreenMargin * 2f));
            if (float.IsNaN(height) || height <= 0f) height = Mathf.Min(minimized ? 110f : 700f, viewportSize.y - ScreenMargin * 2f);
            float maxX = Mathf.Max(ScreenMargin, viewportSize.x - width - ScreenMargin);
            float maxY = Mathf.Max(ScreenMargin, viewportSize.y - height - ScreenMargin);
            panelPosition.x = Mathf.Clamp(panelPosition.x, ScreenMargin, maxX);
            panelPosition.y = Mathf.Clamp(panelPosition.y, ScreenMargin, maxY);
            windowPanel.style.left = panelPosition.x;
            windowPanel.style.top = panelPosition.y;
        }

        private void BeginPanelDrag(PointerDownEvent evt)
        {
            if (evt.button != 0 || !evt.isPrimary) return;
            var target = evt.target as VisualElement;
            if (target is Button || target?.GetFirstAncestorOfType<Button>() != null) return;
            draggingPanel = true;
            dragPointerId = evt.pointerId;
            dragStartPointer = root.WorldToLocal(evt.position);
            dragStartPosition = panelPosition;
            panelHeader.CapturePointer(dragPointerId);
            evt.StopPropagation();
        }

        private void MovePanelDrag(PointerMoveEvent evt)
        {
            if (!draggingPanel || evt.pointerId != dragPointerId) return;
            Vector2 pointer = root.WorldToLocal(evt.position);
            SetPanelPosition(dragStartPosition + pointer - dragStartPointer);
            evt.StopPropagation();
        }

        private void EndPanelDrag()
        {
            if (!draggingPanel) return;
            draggingPanel = false;
            if (panelHeader != null && panelHeader.HasPointerCapture(dragPointerId)) panelHeader.ReleasePointer(dragPointerId);
        }

        private static string FormatSeconds(float seconds) => seconds > 0f ? seconds.ToString("0.00", CultureInfo.InvariantCulture) + " s" : "—";
        private static VisualElement Element(string className) { var element = new VisualElement(); element.AddToClassList(className); return element; }
        private static Label Text(string text, string className) { var label = new Label(text); label.AddToClassList(className); return label; }
        private static Label AddMetric(VisualElement row, string title)
        {
            var metric = Element("rm-metric"); row.Add(metric);
            metric.Add(Text(title, "rm-metric-title"));
            var value = Text("—", "rm-metric-value"); metric.Add(value); return value;
        }
    }
}

