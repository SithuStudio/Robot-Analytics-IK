using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.UIElements;

namespace RobotIK
{
    /// <summary>Runtime Game-view controls. The planner adds this component automatically in Play mode.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(UIDocument))]
    public sealed class RobotMotionControls : MonoBehaviour
    {
        private static readonly HashSet<RobotMotionControls> activePanels = new HashSet<RobotMotionControls>();
        public enum ColorTheme { Light, Dark }
        private const string ThemePreferenceKey = "RobotIK.MotionControls.ColorTheme";
        [SerializeField] private ColorTheme colorTheme = ColorTheme.Light;
        public ColorTheme Theme => colorTheme;
        public enum ControlsTab { Motion, SceneEditor }
        public ControlsTab ActiveTab { get; private set; }
        private VisualElement motionPage, scenePage;
        private Button motionTab, sceneTab;
        private readonly List<Action<RobotBaseSixAxisMotionPlanner.PlanningSettings>> synchronizePlanning = new List<Action<RobotBaseSixAxisMotionPlanner.PlanningSettings>>();
        [SerializeField] private RobotBaseSixAxisMotionPlanner planner;
        private UIDocument document;
        private ScrollView scrollPanel;
        private PanelSettings ownedPanelSettings;
        private VisualElement root, settingsContainer;
        private VisualElement panelHeader, panelBody;
        private Button minimizeButton;
        private bool minimized, draggingPanel, hasPanelPosition;
        private int dragPointerId;
        private Vector2 panelPosition, viewportSize, dragStartPointer, dragStartPosition;
        private const float ScreenMargin = 16f;
        public bool IsMinimized => minimized;
        public Vector2 PanelPosition => panelPosition;
        private Label stateLabel, stateDot, message, feedback, iterations, planningTime, travelTime;
        private DropdownField algorithm;
        private Button planButton, replayButton, themeButton;
        private ColorTheme appliedTheme;
        private readonly List<Action<MotionProfile.Settings>> synchronize = new List<Action<MotionProfile.Settings>>();
        private readonly RobotBaseSixAxisMotionPlanner.PlannerAlgorithm[] algorithms =
        {
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.RRTConnectOptimized,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.RRTConnectFast,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.JointSpacePrm,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.InformedRrtStar,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.CartesianArc,
            RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.GridAStar
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
            activePanels.Remove(this);
            EndPanelDrag();
            pendingSettings = false;
            if (root != null) root.style.display = DisplayStyle.None;
        }

        private void OnDestroy()
        {
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

            var panel = new ScrollView(ScrollViewMode.Vertical);
            scrollPanel = panel;
            panel.AddToClassList("rm-panel");
            panel.contentContainer.AddToClassList("rm-panel-content");
            panel.horizontalScrollerVisibility = ScrollerVisibility.Hidden;
            panel.verticalScrollerVisibility = ScrollerVisibility.Auto;
            root.Add(panel);
            panel.RegisterCallback<GeometryChangedEvent>(_ => ConstrainPanelToViewport());
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
            heading.Add(Text("Six-axis planner", "rm-subtitle"));
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
            motionTab = new Button(() => SetActiveTab(ControlsTab.Motion)) { text = "Motion", name = "motion-tab" };
            sceneTab = new Button(() => SetActiveTab(ControlsTab.SceneEditor)) { text = "Scene editor", name = "scene-editor-tab" };
            motionTab.AddToClassList("rm-tab"); sceneTab.AddToClassList("rm-tab");
            tabs.Add(motionTab); tabs.Add(sceneTab);
            motionPage = Element("rm-motion-page"); motionPage.name = "motion-page"; panelBody.Add(motionPage);
            scenePage = Element("rm-scene-page"); scenePage.name = "scene-editor-page"; panelBody.Add(scenePage);

            settingsContainer = Element("rm-settings"); motionPage.Add(settingsContainer);
            settingsContainer.Add(Text("Planner algorithm", "rm-field-label"));
            algorithm = new DropdownField(new List<string>
            {
                "Optimized RRT-Connect", "Fast RRT-Connect", "Joint-Space PRM", "Informed RRT*", "Cartesian Arc", "3D Grid A*", "Cartesian Linear (MoveL)"
            }, Array.IndexOf(algorithms, planner.SelectedPlanner));
            algorithm.AddToClassList("rm-algorithm");
            algorithm.RegisterValueChangedCallback(evt =>
            {
                if (syncing || planner.IsBusy) return;
                if (algorithm.index >= 0 && algorithm.index < algorithms.Length && planner.SetPlannerAlgorithm(algorithms[algorithm.index]))
                {
                    algorithmChanged = true;
                    message.text = "Planner changed. Reset & plan to create a new route.";
                }
            });
            settingsContainer.Add(algorithm);

            AddNumberControl(settingsContainer, "Motion speed", "mm/s", 1000f,
                s => s.LinearSpeed, (s, v) => s.LinearSpeed = v, "Slower", "Faster");
            AddNumberControl(settingsContainer, "Path accuracy", "mm", 500f,
                s => s.PathAccuracyMillimeters, (s, v) => s.PathAccuracyMillimeters = v, "Closer to waypoints", "Wider curves");
            AddPlanningNumber(settingsContainer, "Minimum obstacle clearance", "mm", "minimum-clearance", 150f,
                s => s.MinimumObstacleClearanceMillimeters, (s, v) => s.MinimumObstacleClearanceMillimeters = v);
            settingsContainer.Add(Text("Minimum gap between the robot and obstacles.", "rm-planning-hint"));
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
            var advanced = new Foldout { text = "More motion settings", value = false };
            advanced.AddToClassList("rm-advanced"); settingsContainer.Add(advanced);
            AddNumberControl(advanced, "Rotation", "°/s", 0f, s => s.RotationSpeedDegree, (s, v) => s.RotationSpeedDegree = v);
            AddNumberControl(advanced, "Acceleration", "mm/s²", 0f, s => s.TcpAccelerationMillimetersPerSecondSquared, (s, v) => s.TcpAccelerationMillimetersPerSecondSquared = v);
            AddNumberControl(advanced, "Deceleration", "mm/s²", 0f, s => s.TcpDecelerationMillimetersPerSecondSquared, (s, v) => s.TcpDecelerationMillimetersPerSecondSquared = v);
            AddNumberControl(advanced, "Angular acceleration", "°/s²", 0f, s => s.TcpRotationAccelerationDegreesPerSecondSquared, (s, v) => s.TcpRotationAccelerationDegreesPerSecondSquared = v);
            AddNumberControl(advanced, "Angular deceleration", "°/s²", 0f, s => s.TcpRotationDecelerationDegreesPerSecondSquared, (s, v) => s.TcpRotationDecelerationDegreesPerSecondSquared = v);
            sceneEditor = planner.GetComponent<RobotRuntimeSceneEditor>();
            if (sceneEditor == null) sceneEditor = planner.gameObject.AddComponent<RobotRuntimeSceneEditor>();
            sceneEditor.Bind(planner);
            sceneEditor.AttachOverlay(root);
            scenePage.Add(sceneEditor.CreateControls(false));

            var status = Element("rm-status-info"); motionPage.Add(status);
            status.Add(Text("Status", "rm-field-label"));
            message = Text("Ready to plan a route.", "rm-status-message"); status.Add(message);
            var metrics = Element("rm-metrics"); status.Add(metrics);
            iterations = AddMetric(metrics, "Iterations");
            planningTime = AddMetric(metrics, "Planning time");
            travelTime = AddMetric(metrics, "Travel time");
            var detail = Element("rm-feedback"); status.Add(detail);
            detail.Add(Text("Planner feedback", "rm-feedback-label"));
            feedback = Text(planner.Status, "rm-feedback-message"); detail.Add(feedback);

            planButton = new Button(() =>
            {
                if (!ApplyPendingSettings()) return;
                algorithmChanged = false;
                planner.ResetToStartAndPlan();
                RefreshStatus();
            }) { text = "Reset & plan" };
            planButton.AddToClassList("rm-primary"); motionPage.Add(planButton);
            replayButton = new Button(() =>
            {
                if (!ApplyPendingSettings()) return;
                planner.ReplayPlannedMotion();
                RefreshStatus();
            }) { text = "Replay motion" };
            replayButton.AddToClassList("rm-secondary"); motionPage.Add(replayButton);
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
            motionTab.EnableInClassList("rm-tab--active", tab == ControlsTab.Motion);
            sceneTab.EnableInClassList("rm-tab--active", tab == ControlsTab.SceneEditor);
            scrollPanel.scrollOffset = Vector2.zero;
            scrollPanel.schedule.Execute(ConstrainPanelToViewport);
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
                slider.AddToClassList("rm-slider"); group.Add(slider);
                var scale = Element("rm-scale"); group.Add(scale);
                scale.Add(Text("Closer to obstacles", "rm-scale-label")); scale.Add(Text("More space", "rm-scale-label"));
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
                slider.AddToClassList("rm-slider"); group.Add(slider);
                var scale = Element("rm-scale"); group.Add(scale);
                scale.Add(Text(lowLabel, "rm-scale-label")); scale.Add(Text(highLabel, "rm-scale-label"));
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
            themeButton.text = colorTheme == ColorTheme.Dark ? "Light mode" : "Dark mode";
            themeButton.tooltip = "Switch to " + (colorTheme == ColorTheme.Dark ? "light" : "dark") + " appearance";
        }

        private void RefreshStatus()
        {
            bool busy = planner.IsBusy;
            settingsContainer.SetEnabled(!busy);
            scenePage.SetEnabled(!busy);
            planButton.SetEnabled(!busy);
            replayButton.SetEnabled(!busy && !algorithmChanged && planner.PlannedPath.Count > 1);
            var state = planner.CurrentMotionState;
            stateLabel.text = state == RobotBaseSixAxisMotionPlanner.MotionState.Complete ? "Complete" : state.ToString();
            stateDot.EnableInClassList("rm-state-dot--busy", busy);
            stateDot.EnableInClassList("rm-state-dot--failed", state == RobotBaseSixAxisMotionPlanner.MotionState.Failed);
            int count = planner.IsRrtSearchInProgress ? planner.CurrentRrtSearchIterations : planner.PlanningIterations;
            iterations.text = count > 0 || busy ? count.ToString(CultureInfo.InvariantCulture) : "—";
            planningTime.text = state == RobotBaseSixAxisMotionPlanner.MotionState.Planning ? "Planning…" : FormatSeconds(planner.PlanningTimeSeconds);
            travelTime.text = FormatSeconds(planner.EstimatedTravelTimeSeconds);
            feedback.text = planner.Status;
            if (planner.Status == previousStatus && state == previousState) return;
            previousStatus = planner.Status;
            previousState = state;
            switch (state)
            {
                case RobotBaseSixAxisMotionPlanner.MotionState.Planning: message.text = "Searching for a collision-free route…"; break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Moving: message.text = "Following the planned route."; break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Complete: message.text = "Motion complete. Replay is available."; break;
                case RobotBaseSixAxisMotionPlanner.MotionState.Failed: message.text = "Motion could not complete. See feedback below."; break;
                default: message.text = "Ready to plan a route."; break;
            }
        }

        private void ResizePanel(GeometryChangedEvent evt)
        {
            viewportSize = evt.newRect.size;
            float width = Mathf.Min(360f, Mathf.Max(0f, viewportSize.x - ScreenMargin * 2f));
            scrollPanel.style.width = width;
            scrollPanel.style.maxHeight = Mathf.Max(0f, viewportSize.y - ScreenMargin * 2f);
            if (!hasPanelPosition)
            {
                panelPosition = new Vector2(viewportSize.x - width - ScreenMargin, ScreenMargin);
                hasPanelPosition = true;
            }
            ConstrainPanelToViewport();
        }

        /// <summary>Folds the panel to its draggable header without interrupting robot motion.</summary>
        public void SetMinimized(bool value)
        {
            minimized = value;
            if (panelBody == null) return;
            panelBody.style.display = minimized ? DisplayStyle.None : DisplayStyle.Flex;
            scrollPanel.EnableInClassList("rm-panel--minimized", minimized);
            minimizeButton.text = minimized ? "□" : "−";
            minimizeButton.tooltip = minimized ? "Restore panel" : "Minimize panel";
            scrollPanel.verticalScrollerVisibility = minimized ? ScrollerVisibility.Hidden : ScrollerVisibility.Auto;
            scrollPanel.schedule.Execute(ConstrainPanelToViewport);
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
            if (scrollPanel == null || viewportSize.x <= 0f || viewportSize.y <= 0f) return;
            float width = scrollPanel.layout.width;
            float height = scrollPanel.layout.height;
            if (float.IsNaN(width) || width <= 0f) width = Mathf.Min(360f, Mathf.Max(0f, viewportSize.x - ScreenMargin * 2f));
            if (float.IsNaN(height) || height <= 0f) height = Mathf.Min(minimized ? 110f : 700f, viewportSize.y - ScreenMargin * 2f);
            float maxX = Mathf.Max(ScreenMargin, viewportSize.x - width - ScreenMargin);
            float maxY = Mathf.Max(ScreenMargin, viewportSize.y - height - ScreenMargin);
            panelPosition.x = Mathf.Clamp(panelPosition.x, ScreenMargin, maxX);
            panelPosition.y = Mathf.Clamp(panelPosition.y, ScreenMargin, maxY);
            scrollPanel.style.left = panelPosition.x;
            scrollPanel.style.top = panelPosition.y;
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

