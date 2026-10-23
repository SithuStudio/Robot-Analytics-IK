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
        private VisualElement panelHeader, panelBody, windowPanel, panelTabs, actionBar;
        private Foldout ikDetails, feedbackDetails, feedbackSummary;
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
            plannerMenuRoot?.RemoveFromHierarchy();
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
            motionTab.AddToClassList("rm-tab"); sceneTab.AddToClassList("rm-tab");
            tabs.Add(motionTab); tabs.Add(sceneTab);
            scrollPanel = new ScrollView(ScrollViewMode.Vertical);
            scrollPanel.AddToClassList("rm-body-scroll");
            scrollPanel.horizontalScrollerVisibility = ScrollerVisibility.Hidden;
            scrollPanel.verticalScrollerVisibility = ScrollerVisibility.Auto;
            panelBody.Add(scrollPanel);
            motionPage = Element("rm-motion-page"); motionPage.name = "motion-page"; scrollPanel.Add(motionPage);
            scenePage = Element("rm-scene-page"); scenePage.name = "scene-editor-page"; scrollPanel.Add(scenePage);

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
            var stepLog = Element("rm-step-log"); status.Add(stepLog);
            stepLog.Add(Text("Planner feedback", "rm-step-heading"));
            feedbackSummary = new Foldout { text = "Ready to plan", value = false, name = "planner-feedback-summary" };
            feedbackSummary.AddToClassList("rm-feedback-summary"); stepLog.Add(feedbackSummary);
            endpointStep = AddStep(feedbackSummary, "endpoint-step", out endpointStepTitle, out endpointStepDetail);
            planStep = AddStep(feedbackSummary, "plan-step", out planStepTitle, out planStepDetail);
            motionStep = AddStep(feedbackSummary, "motion-step", out motionStepTitle, out motionStepDetail);
            feedbackNext = Text(string.Empty, "rm-step-next"); feedbackNext.name = "feedback-next"; feedbackSummary.Add(feedbackNext);
            feedbackDetails = new Foldout { text = "Technical details", value = false, name = "planner-feedback" };
            feedbackDetails.AddToClassList("rm-feedback-details"); status.Add(feedbackDetails);
            feedback = Text(planner.Status, "rm-feedback-message"); feedbackDetails.Add(feedback);
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
            if (actionBar != null) actionBar.style.display = tab == ControlsTab.Motion ? DisplayStyle.Flex : DisplayStyle.None;
            if (message != null) message.style.display = tab == ControlsTab.Motion ? DisplayStyle.Flex : DisplayStyle.None;
            motionTab.EnableInClassList("rm-tab--active", tab == ControlsTab.Motion);
            sceneTab.EnableInClassList("rm-tab--active", tab == ControlsTab.SceneEditor);
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
            feedback.text = planner.Status;
            RefreshStepLog(state);
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
            string route = algorithm.value + $" · {planner.PlannedPath.Count:N0} waypoints";
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
                    else if (raw.IndexOf("singular", StringComparison.OrdinalIgnoreCase) >= 0)
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
                planStepDetail.text = $"{planner.PlannedPath.Count:N0} pts" + (posture.Success ? " · IK " + posture.Groups[1].Value : string.Empty) +
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

