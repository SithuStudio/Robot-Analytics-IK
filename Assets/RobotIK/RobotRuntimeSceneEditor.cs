using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.UIElements;
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
using UnityEngine.InputSystem;
#endif

namespace RobotIK
{
    /// <summary>Play-mode selection and transform tools for planner targets and obstacles.</summary>
    [DisallowMultipleComponent]
    public sealed class RobotRuntimeSceneEditor : MonoBehaviour
    {
        public enum TransformTool { Move, Rotate }
        public enum MoveAxisSpace { World, Local }
        [SerializeField] private MoveAxisSpace moveAxisSpace;
        private EnumField moveAxesField;
        public MoveAxisSpace MoveAxes
        {
            get => moveAxisSpace;
            set
            {
                if (!Enum.IsDefined(typeof(MoveAxisSpace), value)) return;
                EndDrag();
                moveAxisSpace = value;
                moveAxesField?.SetValueWithoutNotify(value);
                UpdateHint();
            }
        }
        [SerializeField, Range(80f, 240f), Tooltip("Handle radius in screen pixels. World size adapts to camera distance and zoom.")]
        private float handleSizePixels = 130f;
        private VisualElement handleOverlay;
        private readonly Label[] directionLabels = new Label[3];
        private float directionLength;
        private Camera handleCamera;
        public float HandleSizePixels
        {
            get => handleSizePixels;
            set => handleSizePixels = Mathf.Clamp(float.IsNaN(value) ? 130f : value, 80f, 240f);
        }
        private static readonly HashSet<RobotRuntimeSceneEditor> editors = new HashSet<RobotRuntimeSceneEditor>();
        private RobotBaseSixAxisMotionPlanner planner;
        private readonly List<Transform> targets = new List<Transform>();
        private readonly List<string> names = new List<string>();
        private readonly LineRenderer[] axes = new LineRenderer[3];
        private readonly Color[] colors = { new Color(0.95f, 0.25f, 0.22f), new Color(0.25f, 0.85f, 0.35f), new Color(0.2f, 0.5f, 1f) };
        private GameObject gizmoRoot;
        private LineRenderer outline;
        private Toggle editToggle;
        private DropdownField objects;
        private Action<GenericDropdownMenu, VisualElement> objectMenuPresenter;
        private VisualElement tool;
        private Button moveTool, rotateTool;
        private VisualElement waypointControls;
        private Button addWaypoint, removeWaypoint, waypointEarlier, waypointLater;
        private Label hint, waypointCount;
        private readonly FloatField[] positionFields = new FloatField[3], rotationFields = new FloatField[3];
        private bool editing, syncingFields, dragging;
        private bool translatingDrag;
        private int dragAxis = -1;
        private Vector3 startPosition, startDirection, startHit, dragWorldAxis;
        private Quaternion startRotation;
        private Plane dragPlane;
        public Transform SelectedObject { get; private set; }
        private TransformTool selectedTool;
        public TransformTool Tool
        {
            get => selectedTool;
            set
            {
                if (!Enum.IsDefined(typeof(TransformTool), value)) return;
                EndDrag(); selectedTool = value;
                moveTool?.EnableInClassList("rm-tool--active", value == TransformTool.Move);
                rotateTool?.EnableInClassList("rm-tool--active", value == TransformTool.Rotate);
                moveAxesField?.SetEnabled(editing && value == TransformTool.Move);
                UpdateHint();
            }
        }
        public bool EditingEnabled => editing;
        public bool IsManipulating => dragging;
        public IReadOnlyList<Transform> EditableObjects => targets;
        public static bool IsManipulatingAny
        {
            get { foreach (var editor in editors) if (editor != null && editor.dragging) return true; return false; }
        }

        public void Bind(RobotBaseSixAxisMotionPlanner target)
        {
            planner = target;
            if (isActiveAndEnabled) editors.Add(this);
            RefreshObjects();
        }

        private void OnEnable() { editors.Add(this); }
        private void OnDisable() { editors.Remove(this); EndDrag(); if (gizmoRoot != null) gizmoRoot.SetActive(false); UpdateDirectionLabels(false); handleOverlay?.MarkDirtyRepaint(); }
        private void OnApplicationFocus(bool focus) { if (!focus) EndDrag(); }
        private void OnDestroy()
        {
            editors.Remove(this);
            handleOverlay?.RemoveFromHierarchy();
            foreach (LineRenderer line in axes) if (line != null) DestroyOwned(line.sharedMaterial);
            if (outline != null) DestroyOwned(outline.sharedMaterial);
            if (gizmoRoot != null) DestroyOwned(gizmoRoot);
        }
        private static void DestroyOwned(UnityEngine.Object item)
        {
            if (item == null) return;
            if (Application.isPlaying) Destroy(item); else DestroyImmediate(item);
        }

        public VisualElement CreateControls(bool foldable = true, Action<GenericDropdownMenu, VisualElement> menuPresenter = null)
        {
            objectMenuPresenter = menuPresenter;
            VisualElement foldout = foldable ? new Foldout { text = "Scene editor", value = false } : new VisualElement();
            foldout.AddToClassList("rm-scene-editor");
            foldout.EnableInClassList("rm-scene-editor--page", !foldable);
            editToggle = new Toggle("Edit targets & obstacles");
            editToggle.AddToClassList("rm-edit-toggle");
            editToggle.SetValueWithoutNotify(editing);
            editToggle.RegisterValueChangedCallback(evt => SetEditingEnabled(evt.newValue));
            foldout.Add(editToggle);
            objects = new DropdownField("Object", new List<string>(names), SelectedObject != null ? targets.IndexOf(SelectedObject) : -1);
            objects.name = "scene-object-dropdown";
            objects.AddToClassList("rm-edit-dropdown");
            objects.RegisterValueChangedCallback(_ => { if (!syncingFields && objects.index >= 0) SelectObject(targets[objects.index]); });
            if (objectMenuPresenter != null) BindDropdownMenu(objects, ShowObjectMenu);
            foldout.Add(objects);
            waypointControls = new VisualElement();
            waypointControls.AddToClassList("rm-route-controls");
            var waypointHeading = new VisualElement(); waypointHeading.AddToClassList("rm-route-heading");
            waypointHeading.Add(new Label("MoveL route waypoints"));
            waypointCount = new Label(); waypointCount.AddToClassList("rm-route-count"); waypointHeading.Add(waypointCount);
            waypointControls.Add(waypointHeading);
            var waypointHelp = new Label("Insert after Start or the selected waypoint. Move markers above/around obstacles, then reset & plan. Orange segments are a route preview, not a validated path.");
            waypointHelp.style.whiteSpace = WhiteSpace.Normal;
            var waypointActions = new VisualElement { name = "scene-waypoint-actions" }; waypointActions.AddToClassList("rm-waypoint-actions");
            addWaypoint = new Button(() =>
            {
                Transform marker = planner.AddIntermediateWaypoint(SelectedObject);
                if (marker == null) return;
                RefreshObjects(); SelectObject(marker);
            }) { text = "+ Add", name = "scene-add-waypoint", tooltip = "Add waypoint after the selected route target" };
            removeWaypoint = new Button(() =>
            {
                if (planner.RemoveIntermediateWaypoint(SelectedObject)) RefreshObjects();
            }) { text = "− Remove", name = "scene-remove-waypoint", tooltip = "Remove the selected intermediate waypoint from the route" };
            waypointEarlier = new Button(() => MoveWaypoint(-1)) { text = "↑ Earlier", name = "scene-waypoint-earlier", tooltip = "Move the selected waypoint earlier in the route" };
            waypointLater = new Button(() => MoveWaypoint(1)) { text = "↓ Later", name = "scene-waypoint-later", tooltip = "Move the selected waypoint later in the route" };
            foreach (var button in new[] { addWaypoint, removeWaypoint, waypointEarlier, waypointLater })
            { button.AddToClassList("rm-waypoint-action"); waypointActions.Add(button); }
            addWaypoint.AddToClassList("rm-waypoint-add"); removeWaypoint.AddToClassList("rm-waypoint-remove");
            waypointControls.Add(waypointActions);
            var routeHelp = new Foldout { text = "Route help", value = false, name = "scene-route-help" };
            routeHelp.AddToClassList("rm-scene-help"); routeHelp.Add(waypointHelp); waypointControls.Add(routeHelp);
            foldout.Add(waypointControls);
            var toolRow = new VisualElement(); toolRow.AddToClassList("rm-scene-tool-row");
            var toolLabel = new Label("Tool"); toolLabel.AddToClassList("rm-scene-tool-label"); toolRow.Add(toolLabel);
            tool = new VisualElement(); tool.AddToClassList("rm-tool-options");
            moveTool = new Button(() => Tool = TransformTool.Move) { text = "↔ Move", name = "scene-move-tool", tooltip = "Move the selected object along world or local axes" };
            rotateTool = new Button(() => Tool = TransformTool.Rotate) { text = "↻ Rotate", name = "scene-rotate-tool", tooltip = "Drag rings to rotate, or local X/Y/Z arrows to move" };
            moveTool.AddToClassList("rm-tool-button"); rotateTool.AddToClassList("rm-tool-button");
            tool.Add(moveTool); tool.Add(rotateTool); Tool = selectedTool;
            toolRow.Add(tool); foldout.Add(toolRow);
            moveAxesField = new EnumField("Axes", MoveAxes) { name = "scene-move-axes" };
            moveAxesField.AddToClassList("rm-edit-dropdown");
            moveAxesField.tooltip = "Move mode: choose World or Local axes. Rotate mode: rings use world axes; arrows always move along local X/Y/Z. Position fields always show world coordinates.";
            moveAxesField.RegisterValueChangedCallback(evt => MoveAxes = (MoveAxisSpace)evt.newValue);
            if (objectMenuPresenter != null) BindDropdownMenu(moveAxesField, ShowAxesMenu);
            foldout.Add(moveAxesField);
            var handleSize = new Slider("Handle size", 80f, 240f) { value = HandleSizePixels, showInputField = true };
            handleSize.AddToClassList("rm-edit-handle-size");
            handleSize.tooltip = "Larger arrows and rings stay the same screen size as you zoom. Drawn above scene objects.";
            handleSize.RegisterValueChangedCallback(evt => HandleSizePixels = evt.newValue);
            foldout.Add(new Label("World position · mm") { name = "edit-position-label" });
            foldout.Add(CreateVectorFields(positionFields, false));
            foldout.Add(new Label("World rotation · degrees") { name = "edit-rotation-label" });
            foldout.Add(CreateVectorFields(rotationFields, true));
            foldout.Add(handleSize);
            var editingHelp = new Foldout { text = "Editing help", value = false, name = "scene-editing-help" };
            editingHelp.AddToClassList("rm-scene-help");
            hint = new Label(); hint.AddToClassList("rm-edit-hint"); editingHelp.Add(hint); foldout.Add(editingHelp);
            var refresh = new Button(RefreshObjects) { text = "↻ Refresh objects", name = "scene-refresh-objects", tooltip = "Refresh the list of editable targets, waypoints and obstacles" };
            refresh.AddToClassList("rm-edit-refresh"); foldout.Add(refresh);
            RefreshObjects();
            SyncFields(); UpdateHint();
            return foldout;
        }

        /// <summary>Draw projected handles above the 3D scene and below the floating controls, in any render pipeline.</summary>
        public void AttachOverlay(VisualElement root)
        {
            handleOverlay?.RemoveFromHierarchy();
            handleOverlay = new VisualElement { name = "runtime-transform-overlay", pickingMode = PickingMode.Ignore };
            handleOverlay.style.position = Position.Absolute;
            handleOverlay.style.left = handleOverlay.style.right = handleOverlay.style.top = handleOverlay.style.bottom = 0;
            handleOverlay.generateVisualContent += PaintHandles;
            for (int i = 0; i < directionLabels.Length; i++)
            {
                var label = new Label("XYZ"[i].ToString()) { pickingMode = PickingMode.Ignore };
                label.style.position = Position.Absolute;
                label.style.width = label.style.height = 24;
                label.style.fontSize = 15;
                label.style.unityFontStyleAndWeight = FontStyle.Bold;
                label.style.unityTextAlign = TextAnchor.MiddleCenter;
                label.style.color = colors[i];
                label.style.backgroundColor = new Color(0.04f, 0.05f, 0.07f, 0.9f);
                label.style.borderTopLeftRadius = label.style.borderTopRightRadius =
                    label.style.borderBottomLeftRadius = label.style.borderBottomRightRadius = 12;
                label.style.display = DisplayStyle.None;
                directionLabels[i] = label;
                handleOverlay.Add(label);
            }
            root.Insert(0, handleOverlay);
        }

        private void PaintHandles(MeshGenerationContext context)
        {
            if (!isActiveAndEnabled || !editing || planner == null || planner.IsBusy || !planner.isActiveAndEnabled ||
                SelectedObject == null || handleCamera == null || gizmoRoot == null || !gizmoRoot.activeSelf) return;
            // UI overlay bypasses scene depth, including HDRP depth, without a pipeline-specific material.
            PaintLine(context, outline, new Color(1f, 0.8f, 0.15f, 0.65f), 1.5f);
            for (int i = 0; i < axes.Length; i++)
            {
                Color color = axes[i].startColor;
                if (Tool == TransformTool.Rotate) color.a = dragging && !translatingDrag && dragAxis == i ? 0.7f : 0.35f;
                PaintLine(context, axes[i], color, 3f);
            }
            if (Tool == TransformTool.Rotate)
                for (int i = 0; i < 3; i++) PaintDirectionArrow(context, i);
        }

        private bool ProjectDirection(int axis, out Vector2 origin, out Vector2 tip)
        {
            origin = tip = default;
            if (handleOverlay?.panel == null || handleCamera == null || SelectedObject == null) return false;
            Vector3 a = handleCamera.WorldToScreenPoint(SelectedObject.position);
            Vector3 direction = Tool == TransformTool.Move ? MoveAxis(axis) : SelectedObject.rotation * Axis(axis);
            Vector3 b = handleCamera.WorldToScreenPoint(SelectedObject.position + direction * directionLength);
            if (a.z < handleCamera.nearClipPlane || b.z < handleCamera.nearClipPlane) return false;
            origin = handleOverlay.WorldToLocal(RuntimePanelUtils.ScreenToPanel(handleOverlay.panel, new Vector2(a.x, Screen.height - a.y)));
            tip = handleOverlay.WorldToLocal(RuntimePanelUtils.ScreenToPanel(handleOverlay.panel, new Vector2(b.x, Screen.height - b.y)));
            return true;
        }

        private void UpdateDirectionLabels(bool visible)
        {
            for (int i = 0; i < directionLabels.Length; i++)
            {
                Label label = directionLabels[i];
                if (label == null) continue;
                if (!visible || !ProjectDirection(i, out Vector2 origin, out Vector2 tip))
                {
                    label.style.display = DisplayStyle.None;
                    continue;
                }
                Vector2 delta = tip - origin;
                // Keep the label readable when an axis points directly toward the camera.
                Vector2 offset = delta.sqrMagnitude > 1f ? delta.normalized : Vector2.up;
                Vector2 position = tip + offset * 18f;
                label.style.left = position.x - 12f;
                label.style.top = position.y - 12f;
                label.style.display = DisplayStyle.Flex;
            }
        }

        private void PaintDirectionArrow(MeshGenerationContext context, int axis)
        {
            if (!ProjectDirection(axis, out Vector2 origin, out Vector2 tip)) return;
            Vector2 delta = tip - origin;
            Vector2 direction = delta.normalized;
            Vector2 side = new Vector2(-direction.y, direction.x);
            float head = Mathf.Min(10f, delta.magnitude * 0.3f);
            var painter = context.painter2D;
            for (int pass = 0; pass < 2; pass++)
            {
                painter.strokeColor = pass == 0 ? new Color(0.04f, 0.05f, 0.07f, 0.9f) :
                    dragging && translatingDrag && dragAxis == axis ? new Color(1f, 0.85f, 0.2f) : colors[axis];
                painter.lineWidth = pass == 0 ? 6f : 3f;
                painter.BeginPath();
                painter.MoveTo(origin); painter.LineTo(tip);
                painter.MoveTo(tip - direction * head + side * head * 0.5f);
                painter.LineTo(tip);
                painter.LineTo(tip - direction * head - side * head * 0.5f);
                painter.Stroke();
            }
        }

        private void PaintLine(MeshGenerationContext context, LineRenderer line, Color color, float width)
        {
            if (line == null || handleOverlay.panel == null) return;
            var painter = context.painter2D;
            // A dark outline keeps the axes legible against bright and dark objects alike.
            for (int pass = 0; pass < 2; pass++)
            {
                painter.strokeColor = pass == 0 ? new Color(0.04f, 0.05f, 0.07f, 0.8f * color.a) : color;
                painter.lineWidth = pass == 0 ? width + 3f : width;
                painter.BeginPath();
                int count = line.positionCount;
                for (int i = 1; i <= count - (line.loop ? 0 : 1); i++)
                {
                    Vector3 a = handleCamera.WorldToScreenPoint(line.GetPosition(i - 1));
                    Vector3 b = handleCamera.WorldToScreenPoint(line.GetPosition(i % count));
                    if (a.z < handleCamera.nearClipPlane || b.z < handleCamera.nearClipPlane) continue;
                    Vector2 p = RuntimePanelUtils.ScreenToPanel(handleOverlay.panel, new Vector2(a.x, Screen.height - a.y));
                    Vector2 q = RuntimePanelUtils.ScreenToPanel(handleOverlay.panel, new Vector2(b.x, Screen.height - b.y));
                    painter.MoveTo(handleOverlay.WorldToLocal(p));
                    painter.LineTo(handleOverlay.WorldToLocal(q));
                }
                painter.Stroke();
            }
        }

        private VisualElement CreateVectorFields(FloatField[] fields, bool rotation)
        {
            var row = new VisualElement(); row.AddToClassList("rm-vector-row");
            for (int i = 0; i < 3; i++)
            {
                var field = new FloatField(new[] { "X", "Y", "Z" }[i]) { isDelayed = true };
                field.AddToClassList("rm-vector-field"); fields[i] = field; row.Add(field);
                field.RegisterValueChangedCallback(_ =>
                {
                    if (syncingFields || SelectedObject == null) return;
                    Vector3 value = new Vector3(fields[0].value, fields[1].value, fields[2].value);
                    if (rotation) SetSelectedPose(SelectedObject.position, Quaternion.Euler(value));
                    else SetSelectedPose(value * 0.001f, SelectedObject.rotation);
                    SyncFields();
                });
            }
            return row;
        }

        private static void BindDropdownMenu(VisualElement field, Action showMenu)
        {
            field.RegisterCallback<PointerDownEvent>(evt =>
            {
                if (evt.button != 0) return;
                evt.StopImmediatePropagation(); showMenu();
            }, TrickleDown.TrickleDown);
            field.RegisterCallback<PointerMoveEvent>(evt =>
            {
                if ((evt.pressedButtons & 1) != 0) evt.StopImmediatePropagation();
            }, TrickleDown.TrickleDown);
            field.RegisterCallback<NavigationSubmitEvent>(evt =>
            {
                evt.StopImmediatePropagation(); showMenu();
            }, TrickleDown.TrickleDown);
        }

        private void ShowAxesMenu()
        {
            if (objectMenuPresenter == null || moveAxesField == null || !moveAxesField.enabledInHierarchy ||
                planner == null || planner.IsBusy || planner.IsRrtSearchInProgress) return;
            var menu = new GenericDropdownMenu();
            foreach (MoveAxisSpace space in new[] { MoveAxisSpace.World, MoveAxisSpace.Local })
            {
                MoveAxisSpace choice = space;
                menu.AddItem(space.ToString(), MoveAxes == space, () =>
                {
                    if (moveAxesField.enabledInHierarchy && !planner.IsBusy && !planner.IsRrtSearchInProgress)
                        moveAxesField.value = choice;
                });
                var row = menu.contentContainer.ElementAt(menu.contentContainer.childCount - 1);
                row.AddToClassList("rm-planner-choice");
                row.userData = space;
            }
            objectMenuPresenter(menu, moveAxesField);
        }

        private void ShowObjectMenu()
        {
            if (objectMenuPresenter == null || objects == null || !objects.enabledInHierarchy ||
                planner == null || planner.IsBusy || planner.IsRrtSearchInProgress) return;
            var menu = new GenericDropdownMenu();
            for (int i = 0; i < targets.Count; i++)
            {
                Transform target = targets[i];
                if (target == null) continue;
                menu.AddItem(names[i], target == SelectedObject, () =>
                {
                    if (target != null && objects.enabledInHierarchy && !planner.IsBusy && !planner.IsRrtSearchInProgress)
                        SelectObject(target);
                });
                var row = menu.contentContainer.ElementAt(menu.contentContainer.childCount - 1);
                row.AddToClassList("rm-planner-choice");
                row.userData = target;
            }
            if (menu.contentContainer.childCount == 0) menu.AddDisabledItem("No editable objects", false);
            objectMenuPresenter(menu, objects);
        }

        public void RefreshObjects()
        {
            targets.Clear(); names.Clear();
            if (planner == null) return;
            AddObject(planner.StartPose, "Start"); AddObject(planner.EndTarget, "End target");
            if (planner.IntermediateWaypoints != null)
                for (int i = 0; i < planner.IntermediateWaypoints.Count; i++) AddObject(planner.IntermediateWaypoints[i], "Waypoint " + (i + 1));
            AddObject(planner.ObstaclesRoot, "Obstacles group");
            if (planner.ObstaclesRoot != null)
                foreach (Transform child in planner.ObstaclesRoot) AddObject(child, "Obstacle " + (child.GetSiblingIndex() + 1));
            if (SelectedObject == null || !targets.Contains(SelectedObject)) SelectedObject = targets.Count > 0 ? targets[0] : null;
            if (objects != null)
            {
                syncingFields = true;
                objects.choices = new List<string>(names);
                objects.SetValueWithoutNotify(SelectedObject != null ? names[targets.IndexOf(SelectedObject)] : string.Empty);
                syncingFields = false;
            }
            SyncFields(); UpdateHint();
        }

        private void AddObject(Transform target, string role)
        {
            if (target == null || targets.Contains(target) || target == planner.transform || target.IsChildOf(planner.transform)) return;
            targets.Add(target); names.Add(role + " · " + target.name);
        }

        public void SetEditingEnabled(bool enabled)
        {
            if (enabled && (planner == null || planner.IsBusy)) return;
            editing = enabled;
            EndDrag();
            if (editToggle != null) editToggle.SetValueWithoutNotify(editing);
            if (enabled) RefreshObjects();
            if (gizmoRoot != null) gizmoRoot.SetActive(editing && SelectedObject != null);
            UpdateDirectionLabels(false);
            handleOverlay?.MarkDirtyRepaint();
            SyncFields(); UpdateHint();
        }

        public bool SelectObject(Transform target)
        {
            if (planner == null || !targets.Contains(target) || !planner.CanEditSceneObject(target)) return false;
            EndDrag(); SelectedObject = target;
            if (objects != null) objects.SetValueWithoutNotify(names[targets.IndexOf(target)]);
            SyncFields(); UpdateHint(); return true;
        }

        public bool SetSelectedPose(Vector3 worldPosition, Quaternion worldRotation)
        {
            if (!editing || planner == null || !planner.CanEditSceneObject(SelectedObject) || !Finite(worldPosition) ||
                !Finite(new Vector3(worldRotation.x, worldRotation.y, worldRotation.z)) || float.IsNaN(worldRotation.w) || float.IsInfinity(worldRotation.w)) return false;
            float magnitude = worldRotation.x * worldRotation.x + worldRotation.y * worldRotation.y + worldRotation.z * worldRotation.z + worldRotation.w * worldRotation.w;
            if (magnitude < 0.000001f) return false;
            SelectedObject.SetPositionAndRotation(worldPosition, worldRotation.normalized);
            planner.NotifySceneEdited();
            SyncFields(); return true;
        }
        private static bool Finite(Vector3 v) => !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) || float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));

        private void UpdateHint()
        {
            if (hint != null) hint.text = editing
                ? "Select an object or click it in the scene. Drag a colored " + (Tool == TransformTool.Move
                    ? "arrow to move along " + (MoveAxes == MoveAxisSpace.Local ? "the object's local" : "world") + " X, Y, Z axes. Position fields show world coordinates."
                    : "ring to rotate about a world axis, or a labeled arrow to move along local +X (red), +Y (green), +Z (blue). Arrow drags keep orientation unchanged.") + " Edits clear saved motion; replan to continue. Play-mode changes are temporary."
                : "Enable editing to reposition targets and obstacles.";
        }

        private void MoveWaypoint(int offset)
        {
            Transform marker = SelectedObject;
            if (!planner.ReorderIntermediateWaypoint(marker, offset)) return;
            RefreshObjects(); SelectObject(marker);
        }

        private void SyncWaypointControls()
        {
            if (waypointControls == null || planner == null) return;
            int count = planner.IntermediateWaypoints?.Count ?? 0;
            waypointCount.text = count + (count == 1 ? " waypoint" : " waypoints");
            waypointControls.style.display = planner.SelectedPlanner == RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.CartesianLinear
                ? DisplayStyle.Flex : DisplayStyle.None;
            bool available = editing && !planner.IsBusy;
            int index = -1;
            var points = planner.IntermediateWaypoints;
            if (points != null)
                for (int i = 0; i < points.Count; i++) if (points[i] == SelectedObject) { index = i; break; }
            addWaypoint.SetEnabled(available);
            removeWaypoint.SetEnabled(available && index >= 0);
            waypointEarlier.SetEnabled(available && index > 0);
            waypointLater.SetEnabled(available && index >= 0 && index + 1 < points.Count);
        }
        private void SyncFields()
        {
            SyncWaypointControls();
            if (positionFields[0] == null) return;
            syncingFields = true;
            Vector3 position = SelectedObject != null ? SelectedObject.position * 1000f : Vector3.zero;
            Vector3 rotation = SelectedObject != null ? SelectedObject.eulerAngles : Vector3.zero;
            for (int i = 0; i < 3; i++)
            {
                positionFields[i].SetValueWithoutNotify(position[i]); rotationFields[i].SetValueWithoutNotify(rotation[i]);
                positionFields[i].SetEnabled(editing && SelectedObject != null && !planner.IsBusy);
                rotationFields[i].SetEnabled(editing && SelectedObject != null && !planner.IsBusy);
            }
            if (objects != null) objects.SetEnabled(editing && targets.Count > 0);
            if (tool != null) tool.SetEnabled(editing);
            moveAxesField?.SetEnabled(editing && Tool == TransformTool.Move && !planner.IsBusy);
            syncingFields = false;
        }

        private void LateUpdate()
        {
            SyncWaypointControls();
            bool available = editing && planner != null && !planner.IsBusy && SelectedObject != null && planner.isActiveAndEnabled;
            if (gizmoRoot != null) gizmoRoot.SetActive(available);
            handleOverlay?.MarkDirtyRepaint();
            if (!available) { EndDrag(); UpdateDirectionLabels(false); return; }
            Camera camera = Camera.main;
            if (camera == null) { handleCamera = null; UpdateDirectionLabels(false); return; }
            DrawGizmos(camera);
            UpdateDirectionLabels(true);
            if (!ReadPointer(out Vector2 pointer, out bool down, out bool held, out bool up)) { EndDrag(); return; }
            if (up || !held) EndDrag();
            if (dragging) { Drag(camera.ScreenPointToRay(pointer)); UpdateDirectionLabels(true); return; }
            if (!down || !camera.pixelRect.Contains(pointer) || RobotMotionControls.IsPointerOverControls(pointer)) return;
            RobotMotionControls.ReleaseControlFocus();
            if (Tool == TransformTool.Rotate)
            {
                int localAxis = PickLocalTranslationAxis(camera, pointer);
                if (localAxis >= 0)
                {
                    // An arrow click must never fall through to a ring rotation,
                    // even if its translation plane cannot be resolved in this view.
                    BeginDrag(localAxis, camera.ScreenPointToRay(pointer), camera, localTranslation: true);
                    return;
                }
            }
            int axis = PickAxis(camera, pointer);
            if (axis >= 0 && BeginDrag(axis, camera.ScreenPointToRay(pointer), camera)) return;
            if (Physics.Raycast(camera.ScreenPointToRay(pointer), out RaycastHit hit, camera.farClipPlane, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Collide))
                for (int i = targets.Count - 1; i >= 0; i--)
                    if (hit.transform == targets[i] || hit.transform.IsChildOf(targets[i])) { SelectObject(targets[i]); return; }
            foreach (Transform target in targets)
            {
                Vector3 projected = camera.WorldToScreenPoint(target.position);
                if (projected.z > 0f && Vector2.Distance(pointer, projected) < 14f) { SelectObject(target); return; }
            }
        }

        private bool BeginDrag(int axis, Ray ray, Camera camera, bool localTranslation = false)
        {
            bool translate = Tool == TransformTool.Move || localTranslation;
            Vector3 direction = localTranslation ? SelectedObject.rotation * Axis(axis) : Tool == TransformTool.Move ? MoveAxis(axis) : Axis(axis);
            Vector3 normal = translate ? camera.transform.forward - direction * Vector3.Dot(camera.transform.forward, direction) : direction;
            if (normal.sqrMagnitude < 0.0001f) return false;
            dragPlane = new Plane(normal.normalized, SelectedObject.position);
            if (!dragPlane.Raycast(ray, out float enter)) return false;
            startHit = ray.GetPoint(enter); startPosition = SelectedObject.position; startRotation = SelectedObject.rotation;
            startDirection = (startHit - startPosition).normalized;
            if (!translate && (startHit - startPosition).sqrMagnitude < 0.000001f) return false;
            // Freeze the chosen world direction for the entire drag, including local-axis moves.
            dragWorldAxis = direction;
            translatingDrag = translate;
            dragAxis = axis; dragging = true; return true;
        }

        private void Drag(Ray ray)
        {
            if (!dragPlane.Raycast(ray, out float enter)) return;
            Vector3 hit = ray.GetPoint(enter), axis = dragWorldAxis;
            if (translatingDrag) SetSelectedPose(startPosition + axis * Vector3.Dot(hit - startHit, axis), startRotation);
            else SetSelectedPose(startPosition, Quaternion.AngleAxis(Vector3.SignedAngle(startDirection, hit - startPosition, axis), axis) * startRotation);
        }
        private void EndDrag() { dragging = false; translatingDrag = false; dragAxis = -1; }
        private static Vector3 Axis(int index) => index == 0 ? Vector3.right : index == 1 ? Vector3.up : Vector3.forward;
        private Vector3 MoveAxis(int index) => MoveAxes == MoveAxisSpace.Local && SelectedObject != null
            ? SelectedObject.rotation * Axis(index) : Axis(index);

        private void DrawGizmos(Camera camera)
        {
            EnsureGizmos(); gizmoRoot.SetActive(true);
            handleCamera = camera;
            bool overlayActive = handleOverlay != null && handleOverlay.panel != null;
            outline.forceRenderingOff = overlayActive;
            Vector3 center = SelectedObject.position;
            float scale = camera.orthographic ? camera.orthographicSize * 2f :
                Mathf.Max(0.1f, Vector3.Dot(center - camera.transform.position, camera.transform.forward)) * 2f * Mathf.Tan(camera.fieldOfView * Mathf.Deg2Rad * 0.5f);
            float length = scale * HandleSizePixels / Mathf.Max(1, camera.pixelHeight);
            directionLength = Tool == TransformTool.Move ? length : length * 0.65f;
            for (int i = 0; i < 3; i++)
            {
                Color color = dragging && (Tool == TransformTool.Move || !translatingDrag) && dragAxis == i ? new Color(1f, 0.85f, 0.2f) : colors[i];
                LineRenderer line = axes[i]; line.startColor = line.endColor = color;
                line.forceRenderingOff = overlayActive;
                if (line.sharedMaterial.HasProperty("_UnlitColor")) line.sharedMaterial.SetColor("_UnlitColor", color);
                if (line.sharedMaterial.HasProperty("_Color")) line.sharedMaterial.SetColor("_Color", color);
                line.startWidth = line.endWidth = length * 0.014f;
                Vector3 axis = Tool == TransformTool.Move ? MoveAxis(i) : Axis(i);
                if (Tool == TransformTool.Move)
                {
                    Vector3 side = Vector3.Cross(axis, camera.transform.forward).normalized;
                    if (side.sqrMagnitude < 0.0001f) side = Vector3.Cross(axis, camera.transform.up).normalized;
                    Vector3 tip = center + axis * length;
                    line.loop = false; line.positionCount = 5;
                    line.SetPositions(new[] { center, tip, tip - axis * length * 0.15f + side * length * 0.065f, tip, tip - axis * length * 0.15f - side * length * 0.065f });
                }
                else
                {
                    Vector3 u = Axis((i + 1) % 3), v = Vector3.Cross(axis, u);
                    line.loop = true; line.positionCount = 64;
                    for (int p = 0; p < 64; p++) { float angle = p * Mathf.PI * 2f / 64; line.SetPosition(p, center + (u * Mathf.Cos(angle) + v * Mathf.Sin(angle)) * length); }
                }
            }
            Bounds bounds = new Bounds(center, Vector3.one * length * 0.12f);
            bool found = false;
            foreach (Renderer renderer in SelectedObject.GetComponentsInChildren<Renderer>())
            {
                if (renderer is LineRenderer || renderer is TrailRenderer || !renderer.enabled) continue;
                if (!found) { bounds = renderer.bounds; found = true; } else bounds.Encapsulate(renderer.bounds);
            }
            bounds.Expand(length * 0.04f);
            Vector3 min = bounds.min, max = bounds.max;
            Vector3[] corners = { new Vector3(min.x,min.y,min.z),new Vector3(max.x,min.y,min.z),new Vector3(max.x,max.y,min.z),new Vector3(min.x,max.y,min.z),new Vector3(min.x,min.y,max.z),new Vector3(max.x,min.y,max.z),new Vector3(max.x,max.y,max.z),new Vector3(min.x,max.y,max.z) };
            int[] order = { 0,1,2,3,0,4,5,1,5,6,2,6,7,3,7,4 };
            outline.positionCount = order.Length; outline.startWidth = outline.endWidth = length * 0.006f;
            for (int i = 0; i < order.Length; i++) outline.SetPosition(i, corners[order[i]]);
        }

        private int PickLocalTranslationAxis(Camera camera, Vector2 point)
        {
            if (SelectedObject == null || Tool != TransformTool.Rotate) return -1;
            Vector3 origin = camera.WorldToScreenPoint(SelectedObject.position);
            if (origin.z <= camera.nearClipPlane) return -1;
            if (Vector2.Distance(point, (Vector2)origin) < 8f) return -1;
            int selected = -1;
            float best = Mathf.Clamp(HandleSizePixels * 0.06f, 8f, 12f);
            for (int axis = 0; axis < 3; axis++)
            {
                Vector3 tip = camera.WorldToScreenPoint(SelectedObject.position + SelectedObject.rotation * Axis(axis) * directionLength);
                if (tip.z <= camera.nearClipPlane) continue;
                Vector2 delta = (Vector2)(tip - origin);
                // Avoid selecting an arbitrary axis at the shared center, and
                // skip nearly camera-aligned arrows that cannot be dragged reliably.
                if (delta.sqrMagnitude < 64f) continue;
                Vector2 start = (Vector2)origin + delta.normalized * Mathf.Min(10f, delta.magnitude * 0.25f);
                Vector2 segment = (Vector2)tip - start;
                float t = Mathf.Clamp01(Vector2.Dot(point - start, segment) / segment.sqrMagnitude);
                float distance = Vector2.Distance(point, start + segment * t);
                var label = directionLabels[axis];
                if (label != null && label.resolvedStyle.display != DisplayStyle.None && handleOverlay?.panel != null &&
                    label.worldBound.Contains(RuntimePanelUtils.ScreenToPanel(handleOverlay.panel, new Vector2(point.x, Screen.height - point.y))))
                    distance = 0f;
                if (distance < best) { best = distance; selected = axis; }
            }
            return selected;
        }

        private int PickAxis(Camera camera, Vector2 point)
        {
            int selected = -1; float best = Mathf.Clamp(HandleSizePixels * 0.1f, 12f, 20f);
            for (int a = 0; a < 3; a++)
            {
                LineRenderer line = axes[a]; int count = line.positionCount;
                for (int i = 1; i <= count - (line.loop ? 0 : 1); i++)
                {
                    Vector3 p = camera.WorldToScreenPoint(line.GetPosition(i - 1)), q = camera.WorldToScreenPoint(line.GetPosition(i % count));
                    if (p.z <= 0f || q.z <= 0f) continue;
                    Vector2 segment = (Vector2)(q - p);
                    if (segment.sqrMagnitude < 1f) continue;
                    float t = Mathf.Clamp01(Vector2.Dot(point - (Vector2)p, segment) / segment.sqrMagnitude);
                    float distance = Vector2.Distance(point, (Vector2)p + segment * t);
                    if (distance < best) { best = distance; selected = a; }
                }
            }
            return selected;
        }

        private void EnsureGizmos()
        {
            if (gizmoRoot != null) return;
            gizmoRoot = new GameObject("Runtime transform handles") { hideFlags = HideFlags.HideAndDontSave };
            for (int i = 0; i < 3; i++) axes[i] = CreateLine("XYZ"[i] + " axis", colors[i]);
            outline = CreateLine("Selection", new Color(1f, 0.8f, 0.15f));
        }
        private LineRenderer CreateLine(string title, Color color)
        {
            var child = new GameObject(title); child.transform.SetParent(gizmoRoot.transform, false);
            var line = child.AddComponent<LineRenderer>();
            Shader shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
            line.sharedMaterial = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
            if (line.sharedMaterial.HasProperty("_UnlitColor")) line.sharedMaterial.SetColor("_UnlitColor", color);
            if (line.sharedMaterial.HasProperty("_Color")) line.sharedMaterial.SetColor("_Color", color);
            line.startColor = line.endColor = color; line.useWorldSpace = true;
            line.shadowCastingMode = ShadowCastingMode.Off; line.receiveShadows = false;
            return line;
        }

        private static bool ReadPointer(out Vector2 position, out bool down, out bool held, out bool up)
        {
            position = default; down = held = up = false;
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
            Mouse mouse = Mouse.current; if (mouse == null) return false;
            position = mouse.position.ReadValue(); down = mouse.leftButton.wasPressedThisFrame; held = mouse.leftButton.isPressed; up = mouse.leftButton.wasReleasedThisFrame;
#else
            if (!Input.mousePresent) return false;
            position = Input.mousePosition; down = Input.GetMouseButtonDown(0); held = Input.GetMouseButton(0); up = Input.GetMouseButtonUp(0);
#endif
            return true;
        }
    }
}


