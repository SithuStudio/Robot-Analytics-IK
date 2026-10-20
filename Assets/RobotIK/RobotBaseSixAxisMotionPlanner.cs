using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Serialization;

namespace RobotIK
{
    /// <summary>
    /// Collision-free motion planner for the analytic six-axis rig. It offers
    /// joint-space RRT-Connect and Cartesian TCP arc planners. Start and End Target
    /// define complete TCP poses; checks use the robot and obstacle colliders.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RobotBaseSixAxisRig))]
    [RequireComponent(typeof(RobotBaseSixAxisAnalyticIK))]
    public sealed class RobotBaseSixAxisMotionPlanner : MonoBehaviour
    {
        public enum PlannerAlgorithm
        {
            [InspectorName("Fast RRT-Connect (first safe route)")]
            RRTConnectFast,
            [InspectorName("Optimized RRT-Connect (shorter route search)")]
            RRTConnectOptimized,
            [InspectorName("Cartesian Arc (TCP path with collision checks)")]
            CartesianArc,
            [InspectorName("3D Grid A* (Cartesian occupancy search)")]
            GridAStar,
            [InspectorName("Joint-Space PRM (sampled roadmap)")]
            JointSpacePrm,
            [InspectorName("Informed RRT* (joint-space optimization)")]
            InformedRrtStar
        }

        private const int JointCount = RobotBaseSixAxisRig.JointCount;

        public enum MotionState { Ready, Planning, Moving, Complete, Failed }
        public MotionState CurrentMotionState { get; private set; }
        public bool IsBusy => CurrentMotionState == MotionState.Planning || CurrentMotionState == MotionState.Moving;
        public bool GameplayControlsEnabled => showGameplayControls;
        public Transform StartPose => startPose;
        public Transform EndTarget => endTarget;
        public Transform ObstaclesRoot => obstaclesRoot;
        public IReadOnlyList<Transform> IntermediateWaypoints => intermediateWaypoints;

        public bool CanEditSceneObject(Transform target)
        {
            if (target == null || IsBusy || target == transform || target.IsChildOf(transform)) return false;
            if (target == startPose || target == endTarget) return true;
            if (obstaclesRoot != null && (target == obstaclesRoot || target.IsChildOf(obstaclesRoot))) return true;
            if (intermediateWaypoints != null)
                foreach (Transform waypoint in intermediateWaypoints) if (target == waypoint) return true;
            return false;
        }

        /// <summary>Invalidates collision checks and saved motion after a runtime target/obstacle edit.</summary>
        public bool NotifySceneEdited()
        {
            if (IsBusy) return false;
            path.Clear();
            ClearPathVisualization();
            rrtCollisionCache.Clear();
            rrtTcpTravelCache.Clear();
            lastPlanSummary = string.Empty;
            adaptiveGoalUsed = false;
            ClearGridGizmoData();
            planningIterations = 0;
            planningTimeSeconds = 0f;
            estimatedTravelTimeSeconds = 0f;
            CurrentMotionState = MotionState.Ready;
            Physics.SyncTransforms();
            Status = "Scene updated. Reset & plan to check the new target/obstacle poses. Previous motion was cleared.";
            return true;
        }

        [SerializeField] private RobotBaseSixAxisRig rig;
        [SerializeField] private RobotBaseSixAxisAnalyticIK inverseKinematics;
        [SerializeField] private Transform startPose;
        [Tooltip("Ordered TCP targets between Start and End Target. Path Accuracy controls how closely the smoothed TCP path follows each corner.")]
        [SerializeField] private Transform[] intermediateWaypoints = Array.Empty<Transform>();
        [SerializeField] private Transform endTarget;
        [Tooltip("When disabled, A and B are position targets and the tool keeps its current world orientation.")]
        [SerializeField] private bool useMarkerOrientations;
        [Header("Adaptive Goal")]
        [Tooltip("If the exact End Target has no valid IK branch, try nearby goal poses within these tolerances. All robot collision and minimum clearance checks still apply. The End Target transform is not moved.")]
        [SerializeField] private bool adaptiveGoal;
        [Tooltip("Maximum distance from the requested goal, in millimeters. Set 0 to keep B's position fixed.")]
        [SerializeField, Min(0f)] private float adaptiveGoalPositionToleranceMillimeters = 10f;
        [Tooltip("Maximum change from the requested TCP orientation. Set 0 to keep its orientation fixed. Applies to the goal even when Use Marker Orientations is off.")]
        [SerializeField, Range(0f, 180f)] private float adaptiveGoalRotationToleranceDegrees = 10f;
        [Tooltip("Maximum nearby poses tested after the exact goal fails. 256 covers all default sampler candidates; lower budgets may miss a feasible nearby goal.")]
        [SerializeField, Range(1, 512)] private int adaptiveGoalMaximumSamples = 256;
        [Header("Environment & Search")]
        [SerializeField] private Transform obstaclesRoot;
        [Tooltip("Optional explicit collision geometry. If empty, all colliders under the robot are used.")]
        [SerializeField] private Collider[] robotColliders;
        [Tooltip("If the robot has no colliders, make conservative box colliders from its mesh bounds at runtime.")]
        [SerializeField] private bool generateRobotBoxCollidersIfMissing = true;
        [Tooltip("Size multiplier applied to BoxColliders in Robot Colliders, including generated fallbacks. 1 preserves authored sizes; lower values shrink each box around its center.")]
        [SerializeField, Range(0.5f, 1f)] private float robotBoxColliderScale = 0.95f;
        [SerializeField, Min(1f)] private float extensionStepDegrees = 20f;
        [SerializeField, Min(0.5f)] private float collisionResolutionDegrees = 4f;

        [Header ("Motion Planning & Constraints")]
        [Tooltip("Fast stops at the first safe route. Optimized ranks routes by TCP travel distance plus the configured J6 travel penalty.")]
        [SerializeField] private PlannerAlgorithm plannerAlgorithm = PlannerAlgorithm.RRTConnectOptimized;
        [Tooltip("Voxel width for the 3D Cartesian occupancy grid, in millimeters. Smaller cells give finer routes but require more search.")]
        [SerializeField, HideInInspector, Min(10f)] private float gridCellSizeMillimeters = 100f;
        [Tooltip("Side length in meters of the cubic 3D grid workspace. The workspace is centered on Grid Workspace Center, or halfway between Start and End Target when no center is assigned.")]
        [SerializeField, HideInInspector, Min(0.1f)] private float gridWorkspaceSizeMeters = 2f;
        [SerializeField, HideInInspector] private Transform gridWorkspaceCenter;
        [Tooltip("Inflation around obstacle bounds in the occupancy grid, in millimeters. The final route is also checked with the full robot colliders and minimum clearance.")]
        [SerializeField, HideInInspector, Min(0f)] private float gridObstacleInflationMillimeters = 50f;
        [Tooltip("Maximum A* grid cells expanded for each Cartesian segment.")]
        [SerializeField, HideInInspector, Min(100)] private int maximumGridNodes = 20000;
        [Tooltip("Maximum distinct A* routes tried per Cartesian segment when IK or full robot collision checks reject a route.")]
        [SerializeField, HideInInspector, Range(1, 32)] private int maximumGridRouteAttempts = 8;
        // Grid/Cartesian settings remain serialized for existing scenes but are hidden in the Inspector.
        [SerializeField, HideInInspector] private bool showGridOccupancyGizmos = true;
        [SerializeField, HideInInspector] private bool showGridFailureGizmo = true;
        [Tooltip("Maximum occupied grid cells drawn in the Scene view.")]
        [SerializeField, HideInInspector, Range(100, 10000)] private int maximumGridGizmoCells = 2500;
        [Tooltip("Penalty applied to squared J4-J6 changes when ranking branches and searching joint space. Higher values discourage unnecessary TCP wrist rotation, but can favor larger shoulder/elbow movements.")]
        [FormerlySerializedAs("j4MotionPenalty"), SerializeField, Min(1f)] private float wristMotionPenalty = 3f;
        [Tooltip("Optimized RRT-Connect adds this many millimeters of route cost for each degree of J6 travel. Increase it to discourage tool-roll detours; 0 ranks by TCP distance alone.")]
        [SerializeField, Min(0f)] private float joint6TravelPenaltyMillimetersPerDegree = 1f;
        [Tooltip("Additional cost per degree of TCP orientation change along a joint-space path. Increase this to favor paths with steadier tool orientation.")]
        [SerializeField, Min(0f)] private float tcpRotationMotionPenalty = 4f;
        [Tooltip("Required minimum distance between robot and obstacle collider surfaces, in world meters. Applied to endpoints and every sampled path pose.")]
        [SerializeField, Min(0f), HideInInspector] private float minimumObstacleClearance = 0.05f;
        [Tooltip("Search budget. For PRM, this is the random-sample budget (capped at 600). For RRT, it is the total iteration limit. Optimized RRT-Connect stops after at most 128 extra refinement iterations once it finds a safe route.")]
        [SerializeField, Min(1)] private int maximumIterations = 4500;
        [Tooltip("Optimized RRT-Connect ranks collision-free routes by TCP distance plus J6 travel penalty, and keeps the displayed path near this many points, including Start and Goal. Extra points are retained when needed for collision safety.")]
        [SerializeField, Range(2, 12)] private int optimizedRrtWaypointBudget = 5;
        [SerializeField, Min(0f)] private float goalBias = 0.08f;
        [Tooltip("RRT random seed. The same seed and starting joint pose produce the same plan; seed 0 uses a fixed default seed.")]
        [SerializeField] private int randomSeed = 0;

        [Header("Motion Speed")]
        [Tooltip("TCP translation speed in millimeters per second, synchronized with the rotation speed limit. Accelerates at the start and decelerates at the end; no corner-speed penalty is applied.")]
        [FormerlySerializedAs("moveSpeedDegreesPerSecond"), SerializeField, Min(1f)] private float Linear = 500f;
        [Tooltip("Maximum TCP rotation speed for orientation changes, in degrees per second.")]
        [SerializeField, Min(1f)] private float RotationDegree = 75f;
        [Tooltip("TCP translation speed ramp at the start, in millimeters per second squared. Path Accuracy controls the corner geometry separately.")]
        [SerializeField, Min(1f)] private float AccelerationSpeed = 200f;
        [Tooltip("TCP translation speed ramp at the end, in millimeters per second squared.")]
        [SerializeField, Min(1f)] private float DecelerationSpeed = 200f;
        [Tooltip("TCP angular speed ramp at the start, in degrees per second squared.")]
        [SerializeField, Min(1f)] private float RotationAcceleration = 150f;
        [Tooltip("TCP angular speed ramp at the end, in degrees per second squared.")]
        [SerializeField, Min(1f)] private float RotationDeceleration = 150f;
        [Header("Path Accuracy")]
        [InspectorName("Path Accuracy (mm)")]
        [Tooltip("Maximum distance from a waypoint corner for the rounded TCP section, in millimeters. Smaller values stay closer to each point; larger values permit wider turns. Minimum 1 mm, with no fixed upper limit. Start and End remain exact. Available segment length and collision checks can limit the curve size. This controls geometry, not playback speed.")]
        [SerializeField, Range(0, 500f)] private float pathAccuracyMillimeters = 20f;
        [Tooltip("Show accuracy boundaries and millimeter dimensions at waypoint corners in the Scene view.")]
        [SerializeField] private bool showPathAccuracyGizmos = true;

        [Header("Options")]
        [Tooltip("Automatically show the floating motion control panel in the Game view during Play mode.")]
        [SerializeField] private bool showGameplayControls = true;
        [SerializeField] private bool playOnPlan = true;
        [SerializeField] private bool planOnPlay = true;
        [SerializeField] private bool checkSelfCollision;
        [Header("Game View Waypoint Labels")]
        [SerializeField] private bool showWaypointLabels = true;
        [SerializeField, Range(8, 64)] private int waypointLabelFontSize = 36;
        [Header("Scene View TCP Rotation Gizmos")]
        [SerializeField] private bool showWaypointTcpRotations = true;
        [Tooltip("Length of each TCP local axis, in millimeters.")]
        [SerializeField, Min(1f)] private float waypointTcpAxisLengthMillimeters = 80f;
        [SerializeField, Min(0f)] private float positionClosureTolerance = 0.01f;
        [SerializeField, Min(0f)] private float rotationClosureToleranceDegrees = 2f;
        private Vector3 plannedGoalPosition;
        private Quaternion plannedGoalRotation;
        private Vector3 requestedGoalPosition;
        private Quaternion requestedGoalRotation;
        private bool adaptiveGoalUsed;
        private bool adaptiveGoalSearchAttempted;
        private int adaptiveGoalSamplesTried;
        private float adaptiveGoalPositionOffsetMillimeters;
        private float adaptiveGoalRotationOffsetDegrees;

        private readonly List<float[]> path = new List<float[]>();
        private MotionProfile motionProfile;
        [System.NonSerialized] private readonly Dictionary<BoxCollider, Vector3> originalRobotBoxSizes =
            new Dictionary<BoxCollider, Vector3>();
        private Collider[] obstacleColliders;
        private System.Random random;
        private Coroutine execution;
        private int planningIterations;
        private int jointSearchSequence;
        private int currentRrtSearchIterations;
        private int rrtSearchInProgress;
        private int gridRouteAttempts;
        private int cartesianArcAttempts;
        private string lastPlanSummary = string.Empty;
        private System.Diagnostics.Stopwatch planningTimer;
        private RobotBaseSixAxisPlannerAlgorithm algorithm;
        private CancellationTokenSource gridSearchCancellation;
        private readonly ConcurrentQueue<Action> rrtMainThreadRequests = new ConcurrentQueue<Action>();
        private readonly Dictionary<JointConfigurationKey, bool> rrtCollisionCache =
            new Dictionary<JointConfigurationKey, bool>();
        private readonly Dictionary<JointSegmentKey, float> rrtTcpTravelCache =
            new Dictionary<JointSegmentKey, float>();
        private LineRenderer pathRenderer;
        private Transform waypointLabelsRoot;
        private readonly List<TextMesh> waypointLabels = new List<TextMesh>();
        private string lastCartesianFailure = "No Cartesian arc was checked.";
        private string lastGridFailure = "No grid route was checked.";
        private string lastJointPlannerFailure = "No joint-space route was found.";
        [SerializeField, HideInInspector] private Vector3[] gizmoGridOccupiedCells = Array.Empty<Vector3>();
        [SerializeField, HideInInspector] private Vector3[] gizmoGridRoutePoints = Array.Empty<Vector3>();
        [SerializeField, HideInInspector] private Vector3 gizmoGridFailurePosition;
        [SerializeField, HideInInspector] private bool hasGridFailurePosition;
        [SerializeField, HideInInspector] private float gizmoGridCellSize;
        [SerializeField, HideInInspector] private Bounds gizmoGridWorkspace;
        [SerializeField, HideInInspector] private bool hasGridWorkspace;

        [SerializeField, TextArea(1, 3)] private string status = "Assign Start, End Target, and Obstacles.";
        [SerializeField, HideInInspector] private Vector3[] gizmoPath = Array.Empty<Vector3>();
        [SerializeField, HideInInspector] private Vector3[] gizmoWaypoints = Array.Empty<Vector3>();
        [SerializeField, HideInInspector] private Quaternion[] gizmoWaypointRotations = Array.Empty<Quaternion>();
        public string Status { get => status; private set => status = value; }
        public IReadOnlyList<float[]> PlannedPath => path;
        public float EstimatedTravelTimeSeconds => estimatedTravelTimeSeconds;
        public int SmoothedPathCorners { get; private set; }
        public int RetainedSharpCorners { get; private set; }
        [SerializeField, HideInInspector] private float estimatedTravelTimeSeconds;
        public int PlanningIterations => planningIterations;
        public int CurrentRrtSearchIterations => System.Threading.Volatile.Read(ref currentRrtSearchIterations);
        public bool IsRrtSearchInProgress => System.Threading.Volatile.Read(ref rrtSearchInProgress) != 0;
        public int GridRouteAttempts => gridRouteAttempts;
        public PlannerAlgorithm SelectedPlanner => plannerAlgorithm;
        public int CartesianArcAttempts => cartesianArcAttempts;
        public float PlanningTimeSeconds => planningTimeSeconds;
        public bool AdaptiveGoalEnabled => adaptiveGoal;
        public bool UsedAdaptiveGoal => adaptiveGoalUsed;
        public int AdaptiveGoalSamplesTried => adaptiveGoalSamplesTried;
        public float AdaptiveGoalPositionOffsetMillimeters => adaptiveGoalPositionOffsetMillimeters;
        public float AdaptiveGoalRotationOffsetDegrees => adaptiveGoalRotationOffsetDegrees;
        [SerializeField, HideInInspector] private float planningTimeSeconds;

        private void Reset()
        {
            rig = GetComponent<RobotBaseSixAxisRig>();
            inverseKinematics = GetComponent<RobotBaseSixAxisAnalyticIK>();
        }

        private void OnDisable()
        {
            CancelGridSearch();
            StopAllCoroutines();
            execution = null;
            CurrentMotionState = MotionState.Ready;
        }

        private void Awake()
        {
            if (rig == null) rig = GetComponent<RobotBaseSixAxisRig>();
            if (inverseKinematics == null) inverseKinematics = GetComponent<RobotBaseSixAxisAnalyticIK>();
            CacheColliders();
            if (showGameplayControls)
            {
                RobotMotionControls controls = GetComponent<RobotMotionControls>();
                if (controls == null) controls = gameObject.AddComponent<RobotMotionControls>();
                controls.Bind(this);
            }
        }

        public MotionProfile.Settings GetMotionSettings() => new MotionProfile.Settings
        {
            LinearSpeed = Linear,
            RotationSpeedDegree = RotationDegree,
            TcpAccelerationMillimetersPerSecondSquared = AccelerationSpeed,
            TcpDecelerationMillimetersPerSecondSquared = DecelerationSpeed,
            TcpRotationAccelerationDegreesPerSecondSquared = RotationAcceleration,
            TcpRotationDecelerationDegreesPerSecondSquared = RotationDeceleration,
            PathAccuracyMillimeters = pathAccuracyMillimeters,
            CollisionResolutionDegrees = collisionResolutionDegrees
        };

        public bool ApplyMotionSettings(MotionProfile.Settings settings)
        {
            if (settings == null || IsBusy) return false;
            foreach (float value in new[] { settings.LinearSpeed,
                settings.RotationSpeedDegree, settings.TcpAccelerationMillimetersPerSecondSquared,
                settings.TcpDecelerationMillimetersPerSecondSquared, settings.TcpRotationAccelerationDegreesPerSecondSquared,
                settings.TcpRotationDecelerationDegreesPerSecondSquared, settings.PathAccuracyMillimeters })
                if (float.IsNaN(value) || float.IsInfinity(value) || value < 1f) return false;
            Linear = settings.LinearSpeed;
            RotationDegree = settings.RotationSpeedDegree;
            AccelerationSpeed = settings.TcpAccelerationMillimetersPerSecondSquared;
            DecelerationSpeed = settings.TcpDecelerationMillimetersPerSecondSquared;
            RotationAcceleration = settings.TcpRotationAccelerationDegreesPerSecondSquared;
            RotationDeceleration = settings.TcpRotationDecelerationDegreesPerSecondSquared;
            pathAccuracyMillimeters = settings.PathAccuracyMillimeters;
            RefreshMotionPreview();
            return true;
        }

        public bool SetPlannerAlgorithm(PlannerAlgorithm value)
        {
            if (IsBusy || !Enum.IsDefined(typeof(PlannerAlgorithm), value)) return false;
            plannerAlgorithm = value;
            return true;
        }

        /// <summary>Runtime planning constraints. Distances exposed here are always millimeters.</summary>
        public sealed class PlanningSettings
        {
            public float MinimumObstacleClearanceMillimeters;
            public bool AdaptiveGoalEnabled;
            public float PositionToleranceMillimeters;
            public float RotationToleranceDegrees;
            public int MaximumGoalSamples;
        }

        public PlanningSettings GetPlanningSettings() => new PlanningSettings
        {
            MinimumObstacleClearanceMillimeters = minimumObstacleClearance * 1000f,
            AdaptiveGoalEnabled = adaptiveGoal,
            PositionToleranceMillimeters = adaptiveGoalPositionToleranceMillimeters,
            RotationToleranceDegrees = adaptiveGoalRotationToleranceDegrees,
            MaximumGoalSamples = adaptiveGoalMaximumSamples
        };

        public bool ApplyPlanningSettings(PlanningSettings settings)
        {
            if (settings == null || IsBusy || settings.MaximumGoalSamples < 1 || settings.MaximumGoalSamples > 512 ||
                settings.RotationToleranceDegrees > 180f) return false;
            foreach (float value in new[] { settings.MinimumObstacleClearanceMillimeters,
                settings.PositionToleranceMillimeters, settings.RotationToleranceDegrees })
                if (float.IsNaN(value) || float.IsInfinity(value) || value < 0f) return false;
            float clearanceMeters = settings.MinimumObstacleClearanceMillimeters * 0.001f;
            if (minimumObstacleClearance == clearanceMeters && adaptiveGoal == settings.AdaptiveGoalEnabled &&
                adaptiveGoalPositionToleranceMillimeters == settings.PositionToleranceMillimeters &&
                adaptiveGoalRotationToleranceDegrees == settings.RotationToleranceDegrees &&
                adaptiveGoalMaximumSamples == settings.MaximumGoalSamples) return true;
            minimumObstacleClearance = clearanceMeters;
            adaptiveGoal = settings.AdaptiveGoalEnabled;
            adaptiveGoalPositionToleranceMillimeters = settings.PositionToleranceMillimeters;
            adaptiveGoalRotationToleranceDegrees = settings.RotationToleranceDegrees;
            adaptiveGoalMaximumSamples = settings.MaximumGoalSamples;
            NotifySceneEdited();
            Status = "Planning settings updated. Reset & plan to check the new clearance and goal limits. Previous motion was cleared.";
            return true;
        }

        private IEnumerator RunMotionOperation(IEnumerator operation)
        {
            try { yield return StartCoroutine(operation); }
            finally
            {
                execution = null;
                if (IsBusy) CurrentMotionState = MotionState.Ready;
            }
        }

        private void Start()
        {
            if (planOnPlay) PlanAndMove();
        }

        private void LateUpdate()
        {
            Camera viewCamera = Camera.main;
            for (int i = 0; i < waypointLabels.Count; i++)
            {
                TextMesh label = waypointLabels[i];
                if (label == null) continue;
                bool visible = showWaypointLabels && i < gizmoWaypoints.Length;
                if (label.gameObject.activeSelf != visible) label.gameObject.SetActive(visible);
                label.fontSize = waypointLabelFontSize;
                if (!visible || viewCamera == null) continue;
                Vector3 toCamera = viewCamera.transform.position - label.transform.position;
                if (toCamera.sqrMagnitude > 0.0001f)
                {
                    // Legacy TextMesh renders its front face toward local -Z.
                    label.transform.rotation = Quaternion.LookRotation(-toCamera, viewCamera.transform.up);
                }
            }
        }

        private void Update()
        {
            // RRT search runs on a worker, but each segment's Physics/Transform sweep is
            // serviced here on Unity's main thread. Bound the number of segment sweeps per
            // frame so a large iteration budget does not monopolize the editor/game loop.
            if (rrtMainThreadRequests.IsEmpty) return;
            var timer = System.Diagnostics.Stopwatch.StartNew();
            while (timer.Elapsed.TotalMilliseconds < 3.0 && rrtMainThreadRequests.TryDequeue(out Action request))
                request();
        }

        private void OnValidate()
        {
            extensionStepDegrees = Mathf.Max(1f, extensionStepDegrees);
            collisionResolutionDegrees = Mathf.Max(0.5f, collisionResolutionDegrees);
            minimumObstacleClearance = Mathf.Max(0f, minimumObstacleClearance);
            wristMotionPenalty = Mathf.Max(1f, wristMotionPenalty);
            joint6TravelPenaltyMillimetersPerDegree = Mathf.Max(0f, joint6TravelPenaltyMillimetersPerDegree);
            robotBoxColliderScale = Mathf.Clamp(robotBoxColliderScale, 0.5f, 1f);
            Linear = Mathf.Max(1f, Linear);
            RotationDegree = Mathf.Max(1f, RotationDegree);
            AccelerationSpeed = Mathf.Max(1f, AccelerationSpeed);
            DecelerationSpeed = Mathf.Max(1f, DecelerationSpeed);
            RotationAcceleration = Mathf.Max(1f, RotationAcceleration);
            RotationDeceleration = Mathf.Max(1f, RotationDeceleration);
            pathAccuracyMillimeters = Mathf.Max(1f, pathAccuracyMillimeters);
            maximumIterations = Mathf.Max(1, maximumIterations);
            adaptiveGoalPositionToleranceMillimeters = Mathf.Max(0f, adaptiveGoalPositionToleranceMillimeters);
            adaptiveGoalRotationToleranceDegrees = Mathf.Clamp(adaptiveGoalRotationToleranceDegrees, 0f, 180f);
            adaptiveGoalMaximumSamples = Mathf.Clamp(adaptiveGoalMaximumSamples, 1, 512);
            goalBias = Mathf.Clamp01(goalBias);
            CorrectSwappedPoseReferences();
        }

        [ContextMenu("Plan Collision-Free Motion Start to End")]
        public void PlanAndMove()
        {
            CancelGridSearch();
            StopAllCoroutines();
            CurrentMotionState = MotionState.Planning;
            execution = StartCoroutine(RunMotionOperation(PlanAndMoveRoutine()));
        }

        private void CancelGridSearch()
        {
            System.Threading.Interlocked.Exchange(ref rrtSearchInProgress, 0);
            if (gridSearchCancellation == null) return;
            gridSearchCancellation.Cancel();
            gridSearchCancellation.Dispose();
            gridSearchCancellation = null;
        }

        /// <summary>Replays the stored plan from its first joint state without replanning.</summary>
        public void ReplayPlannedMotion()
        {
            if (!Application.isPlaying)
            {
                Status = "Enter Play mode before replaying the planned motion.";
                return;
            }
            if (path.Count < 2)
            {
                Status = "No stored motion to replay. Plan a path first.";
                return;
            }
            CancelGridSearch();
            StopAllCoroutines();
            CurrentMotionState = MotionState.Moving;
            execution = StartCoroutine(RunMotionOperation(ReplayPlannedMotionRoutine()));
        }

        /// <summary>Updates the curve and timing after Inspector edits while the robot is idle.</summary>
        public void RefreshMotionPreview()
        {
            if (!IsBusy && !IsRrtSearchInProgress && path.Count > 1)
                DrawPlannedPath();
        }

        private IEnumerator ReplayPlannedMotionRoutine()
        {
            if (inverseKinematics != null) inverseKinematics.SolveContinuously = false;
            rig.SetJointAngles(path[0]);
            Physics.SyncTransforms();
            yield return null;
            yield return StartCoroutine(ExecutePath());
            execution = null;
        }

        /// <summary>Reset the arm to a collision-free IK solution at A, then plan and execute A to B.</summary>
        public void ResetToStartAndPlan()
        {
            if (!Application.isPlaying)
            {
                Status = "Enter Play mode before resetting and planning.";
                return;
            }
            CancelGridSearch();
            StopAllCoroutines();
            CurrentMotionState = MotionState.Planning;
            execution = StartCoroutine(RunMotionOperation(ResetToStartAndPlanRoutine()));
        }

        private IEnumerator ResetToStartAndPlanRoutine()
        {
            if (inverseKinematics != null) inverseKinematics.SolveContinuously = false;
            if (!ValidateSetup()) { execution = null; yield break; }
            CacheColliders();
            if (robotColliders.Length == 0 || obstacleColliders.Length == 0)
            {
                Fail("Collision geometry is missing. Assign robot colliders and colliders beneath Obstacles.");
                execution = null;
                yield break;
            }

            float[] originalAngles = rig.GetJointAnglesDegrees();
            Quaternion startRotation = useMarkerOrientations ? startPose.rotation : GetCurrentTcp().rotation;
            if (!inverseKinematics.SolveAll(startPose.position, startRotation, out List<float[]> candidates))
            {
                Fail("Start pose is unreachable by the analytic IK.");
                execution = null;
                yield break;
            }

            List<float[]> validStarts = FilterPoseSolutions(candidates, startPose, startRotation, out PoseCheckReport report);
            rig.SetJointAngles(originalAngles);
            Physics.SyncTransforms();
            if (validStarts.Count == 0)
            {
                Fail($"Cannot reset to Start. {DescribePoseCheck(startPose, report)}");
                execution = null;
                yield break;
            }

            // IK returns its branches in a stable order. Keep the same valid branch on each reset.
            if (!rig.SetJointAngles(validStarts[0]))
            {
                Fail("Could not apply the selected Start IK solution.");
                execution = null;
                yield break;
            }
            Physics.SyncTransforms();
            yield return null;
            yield return StartCoroutine(PlanAndMoveRoutine(forcePlayAfterPlan: true));
        }

        private IEnumerator PlanAndMoveRoutine(bool forcePlayAfterPlan = false)
        {
            // Continuous IK writes joint angles every Update and would fight the
            // trajectory player. From here on, the planner owns joint motion.
            if (inverseKinematics != null) inverseKinematics.SolveContinuously = false;
            algorithm = null;
            CancelGridSearch();
            gridSearchCancellation = new CancellationTokenSource();
            planningIterations = 0;
            jointSearchSequence = 0;
            rrtCollisionCache.Clear();
            rrtTcpTravelCache.Clear();
            System.Threading.Interlocked.Exchange(ref currentRrtSearchIterations, 0);
            gridRouteAttempts = 0;
            cartesianArcAttempts = 0;
            lastPlanSummary = string.Empty;
            lastJointPlannerFailure = "No joint-space route was found.";
            planningTimeSeconds = 0f;
            planningTimer = System.Diagnostics.Stopwatch.StartNew();
            path.Clear();
            ClearGridGizmoData();
            ClearPathVisualization();
            if (!ValidateSetup()) { execution = null; yield break; }
            CacheColliders();
            if (robotColliders.Length == 0 || obstacleColliders.Length == 0)
            {
                Fail("Collision geometry is missing. Assign robot colliders and colliders beneath Obstacles.");
                execution = null;
                yield break;
            }

            Status = "Planning from current robot pose to Start, then to End Target...";
            float[] originalAngles = rig.GetJointAnglesDegrees();
            Quaternion preferredRotation = GetCurrentTcp().rotation;
            Quaternion startRotation = useMarkerOrientations ? startPose.rotation : preferredRotation;
            Quaternion goalRotation = useMarkerOrientations ? endTarget.rotation : preferredRotation;
            InitializeGoalSelection(goalRotation);
            if (!inverseKinematics.SolveAll(startPose.position, startRotation, out List<float[]> starts))
            {
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                Fail("Start pose is unreachable by the analytic IK.");
                execution = null;
                yield break;
            }

            List<float[]> validStarts = FilterPoseSolutions(starts, startPose, startRotation, out PoseCheckReport startReport);
            rig.SetJointAngles(originalAngles);
            Physics.SyncTransforms();
            List<float[]> validGoals = SolveGoalPose(plannedGoalPosition, plannedGoalRotation, false, out PoseCheckReport goalReport);
            if (validStarts.Count > 0 && validGoals.Count == 0 && adaptiveGoal)
                yield return StartCoroutine(FindAdaptiveGoalOverFrames(found => validGoals = found));
            if (validStarts.Count == 0 || validGoals.Count == 0)
            {
                Fail($"No collision-free IK solution. {DescribePoseCheck(startPose, startReport)} {DescribePoseCheck(endTarget, goalReport)}{AdaptiveGoalFailureSummary}");
                execution = null;
                yield break;
            }

            if (!TryGetIntermediateWaypointSolutions(preferredRotation,
                    out List<Transform> viaPoses, out List<List<float[]>> viaSolutions, out string waypointError))
            {
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                Fail(waypointError);
                execution = null;
                yield break;
            }

            rig.SetJointAngles(originalAngles);
            Physics.SyncTransforms();
            if (IsInCollision())
            {
                Fail("The robot's current pose collides with an obstacle, so a collision-free move cannot start.");
                execution = null;
                yield break;
            }

            if (plannerAlgorithm == PlannerAlgorithm.CartesianArc)
            {
                bool cartesianFound = false;
                List<float[]> bestCartesianPath = null;
                SortByDistance(validStarts, originalAngles);
                validGoals.Sort((a, b) => CompareDistance(a, b, originalAngles));
                Transform tcp = GetCurrentTcp();
                Vector3 currentPosition = tcp.position;
                Quaternion currentRotation = tcp.rotation;
                for (int s = 0; s < validStarts.Count && !cartesianFound; s++)
                {
                    rig.SetJointAngles(originalAngles);
                    Physics.SyncTransforms();
                    var toStart = new List<float[]>();
                    if (!TryBuildCartesianArc(originalAngles, currentPosition, currentRotation,
                            validStarts[s], startPose.position, startRotation, toStart))
                    {
                        lastCartesianFailure = $"Current TCP to Start: {lastCartesianFailure}";
                        continue;
                    }
                    var tailPoses = new List<Transform>(viaPoses) { endTarget };
                    var tailSolutions = new List<List<float[]>>(viaSolutions) { validGoals };
                    bestCartesianPath = new List<float[]>(toStart);
                    cartesianFound = TryBuildCartesianTail(validStarts[s], startPose.position, startRotation,
                        tailPoses, tailSolutions, 0, bestCartesianPath, preferredRotation);
                    if (!cartesianFound) lastCartesianFailure = $"Start through intermediate waypoint(s) to End Target: {lastCartesianFailure}";
                }
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                if (!cartesianFound)
                {
                    Fail($"Cartesian Arc failed. {lastCartesianFailure}");
                    execution = null;
                    yield break;
                }
                path.AddRange(bestCartesianPath);
                // Keep the Cartesian samples: joint-space shortcutting would pull the
                // TCP away from the selected Cartesian arc even if it remains collision-free.
                SetPlanSummary($"Plan found with Cartesian Arc after {cartesianArcAttempts} arc attempts in {FinishPlanningTimer()} ({path.Count} waypoints).");
                Status = lastPlanSummary;
                Debug.Log($"Motion planner: {Status}", this);
                DrawPlannedPath();
                if (playOnPlan || forcePlayAfterPlan) yield return StartCoroutine(ExecutePath());
                execution = null;
                yield break;
            }

            if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
            {
                bool gridFound = false;
                Transform tcp = GetCurrentTcp();
                var tailPoses = new List<Transform>(viaPoses) { endTarget };
                var tailSolutions = new List<List<float[]>>(viaSolutions) { validGoals };
                for (int s = 0; s < validStarts.Count && !gridFound; s++)
                {
                    rig.SetJointAngles(originalAngles);
                    Physics.SyncTransforms();
                    path.Clear();
                    var prefix = new List<float[]>();
                    bool prefixFound = false;
                    yield return StartCoroutine(TryBuildGridSegmentOnWorker(originalAngles, tcp.position, tcp.rotation,
                        validStarts[s], startPose.position, startRotation, prefix, ok => prefixFound = ok));
                    if (!prefixFound) continue;
                    path.AddRange(prefix);
                    bool tailFound = false;
                    yield return StartCoroutine(TryBuildGridTailOnWorker(validStarts[s], startPose.position, startRotation,
                        tailPoses, tailSolutions, 0, path, preferredRotation, ok => tailFound = ok));
                    gridFound = tailFound;
                }
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                if (!gridFound)
                {
                    Fail($"3D Grid A* failed after {gridRouteAttempts} route attempt(s) and {planningIterations} grid nodes expanded. {lastGridFailure}");
                    execution = null;
                    yield break;
                }
                hasGridFailurePosition = false;
                SetPlanSummary($"Plan found with 3D Grid A* after {gridRouteAttempts} route attempt(s) and {planningIterations} grid nodes expanded in {FinishPlanningTimer()} ({path.Count} waypoints).");
                Status = lastPlanSummary;
                Debug.Log($"Motion planner: {Status}", this);
                DrawPlannedPath();
                if (playOnPlan || forcePlayAfterPlan) yield return StartCoroutine(ExecutePath());
                execution = null;
                yield break;
            }

            random = new System.Random(randomSeed == 0 ? 1 : randomSeed);
            SortByDistance(validStarts, originalAngles);
            validGoals.Sort((a, b) => CompareDistance(a, b, originalAngles));
            var jointSpaceStages = new List<List<float[]>>(viaSolutions) { validGoals };
            bool foundPath = false;
            for (int s = 0; s < validStarts.Count && !foundPath; s++)
            {
                path.Clear();
                bool reachedStart = false;
                yield return StartCoroutine(TryConnectOverFrames(originalAngles, validStarts[s], originalAngles,
                    result => reachedStart = result, "Current pose to Start"));
                if (!reachedStart) continue;

                bool reachedGoal = false;
                yield return StartCoroutine(TryConnectSequenceOverFrames(validStarts[s], jointSpaceStages,
                    0, originalAngles, result => reachedGoal = result));
                foundPath = reachedGoal;
            }

            rig.SetJointAngles(originalAngles);
            Physics.SyncTransforms();
            if (!foundPath)
            {
                Fail($"{lastJointPlannerFailure} Search used {planningIterations} iterations. Try more iterations or adjust the obstacles/targets.");
                execution = null;
                yield break;
            }

            if (!HasIntermediateWaypoints()) SimplifyPath();
            SetPlanSummary($"Plan found with {GetPlannerAlgorithmLabel()} after {planningIterations} search iterations in {FinishPlanningTimer()} ({path.Count} waypoints).");
            Status = lastPlanSummary;
            Debug.Log($"Motion planner: {Status}", this);
            DrawPlannedPath();
            if (playOnPlan || forcePlayAfterPlan) yield return StartCoroutine(ExecutePath());
            execution = null;
        }

        private IEnumerator TryConnectOverFrames(float[] start, float[] goal, float[] restoreAngles,
            Action<bool> completed, string segmentLabel = "Joint segment")
        {
            Status = $"{segmentLabel}: searching with {GetPlannerAlgorithmLabel()} on a worker thread...";
            CancellationToken token = gridSearchCancellation != null
                ? gridSearchCancellation.Token : CancellationToken.None;
            System.Threading.Interlocked.Exchange(ref currentRrtSearchIterations, 0);
            System.Threading.Interlocked.Exchange(ref rrtSearchInProgress, 1);
            RobotBaseSixAxisPlannerAlgorithm.Kind kind = GetAlgorithmKind();
            Vector3 noPosition = Vector3.zero;
            Quaternion noRotation = Quaternion.identity;
            RobotBaseSixAxisPlannerAlgorithm.World workerWorld = CreateRrtWorkerWorld(restoreAngles, token);
            Task<RrtWorkerOutcome> search = Task.Run(() =>
            {
                try
                {
                    var workerAlgorithm = new RobotBaseSixAxisPlannerAlgorithm(workerWorld);
                    bool found = workerAlgorithm.TryPlan(kind, start, goal, noPosition, noRotation,
                        noPosition, noRotation, out RobotBaseSixAxisPlannerAlgorithm.Result result);
                    return new RrtWorkerOutcome { Found = found, Result = result };
                }
                catch (OperationCanceledException)
                {
                    return new RrtWorkerOutcome { Cancelled = true };
                }
                catch (Exception exception)
                {
                    return new RrtWorkerOutcome { Error = exception };
                }
            });

            while (!search.IsCompleted)
            {
                if (token.IsCancellationRequested)
                {
                    System.Threading.Interlocked.Exchange(ref rrtSearchInProgress, 0);
                    rig.SetJointAngles(restoreAngles);
                    Physics.SyncTransforms();
                    completed(false);
                    yield break;
                }
                yield return null;
            }

            RrtWorkerOutcome outcome = search.GetAwaiter().GetResult();
            System.Threading.Interlocked.Exchange(ref rrtSearchInProgress, 0);
            bool connected = outcome.Found && !outcome.Cancelled;
            if (outcome.Error != null)
            {
                Debug.LogException(outcome.Error, this);
                Status = $"{GetPlannerAlgorithmLabel()} search failed: {outcome.Error.Message}";
            }
            if (outcome.Result != null)
            {
                planningIterations += outcome.Result.Iterations;
                if (connected) path.AddRange(outcome.Result.JointPath);
                else if (!string.IsNullOrEmpty(outcome.Result.FailureReason))
                    lastJointPlannerFailure = $"{segmentLabel}: {outcome.Result.FailureReason}";
            }
            rig.SetJointAngles(restoreAngles);
            Physics.SyncTransforms();
            completed(connected);
        }

        private sealed class RrtWorkerOutcome
        {
            public bool Found;
            public bool Cancelled;
            public Exception Error;
            public RobotBaseSixAxisPlannerAlgorithm.Result Result;
        }

        private sealed class RrtMainThreadQuery<T>
        {
            private readonly Func<T> query;
            private readonly CancellationToken cancellationToken;
            private readonly ManualResetEventSlim completed = new ManualResetEventSlim(false);
            private T result;
            private Exception error;

            public RrtMainThreadQuery(Func<T> query, CancellationToken cancellationToken)
            { this.query = query; this.cancellationToken = cancellationToken; }

            public void Execute()
            {
                try
                {
                    if (cancellationToken.IsCancellationRequested)
                        throw new OperationCanceledException(cancellationToken);
                    result = query();
                }
                catch (Exception exception) { error = exception; }
                finally { completed.Set(); }
            }

            public T Wait()
            {
                while (!completed.Wait(50)) cancellationToken.ThrowIfCancellationRequested();
                if (error != null) throw error;
                cancellationToken.ThrowIfCancellationRequested();
                return result;
            }
        }

        private T InvokeRrtMainThread<T>(Func<T> query, CancellationToken cancellationToken)
        {
            var request = new RrtMainThreadQuery<T>(query, cancellationToken);
            rrtMainThreadRequests.Enqueue(request.Execute);
            return request.Wait();
        }

        private RobotBaseSixAxisPlannerAlgorithm.World CreateRrtWorkerWorld(float[] restoreAngles,
            CancellationToken cancellationToken)
        {
            Vector2[] limits = rig.AngleLimitsDegrees;
            var copiedLimits = new Vector2[limits.Length];
            Array.Copy(limits, copiedLimits, limits.Length);
            // Keep complete replans deterministic while giving different endpoint
            // branches/segments distinct sample streams within the same plan.
            int searchSeed = unchecked((randomSeed == 0 ? 1 : randomSeed) + jointSearchSequence++ * 104729);
            var workerRandom = new System.Random(searchSeed);
            var sweptSegmentCache = new Dictionary<ExactJointSegmentKey, bool>();
            float copiedWristPenalty = wristMotionPenalty;
            var world = new RobotBaseSixAxisPlannerAlgorithm.World
            {
                MaximumIterations = maximumIterations,
                ExtensionStepDegrees = extensionStepDegrees,
                CollisionResolutionDegrees = collisionResolutionDegrees,
                GoalBias = goalBias,
                PrioritizeShortestPath = plannerAlgorithm == PlannerAlgorithm.RRTConnectOptimized,
                Joint6TravelCostPerDegree = joint6TravelPenaltyMillimetersPerDegree * 0.001f,
                IterationProgress = value => System.Threading.Interlocked.Exchange(ref currentRrtSearchIterations, value),
                CancellationToken = cancellationToken,
                Random = workerRandom,
                JointLimits = copiedLimits,
                SolveIk = (position, rotation) => null,
                GetTcpPosition = angles => Vector3.zero,
                GetTcpRotation = angles => Quaternion.identity,
                RandomConfiguration = () =>
                {
                    var angles = new float[copiedLimits.Length];
                    for (int joint = 0; joint < angles.Length; joint++)
                        angles[joint] = copiedLimits[joint].x +
                            (copiedLimits[joint].y - copiedLimits[joint].x) * (float)workerRandom.NextDouble();
                    return angles;
                },
                IsConfigurationInCollision = angles => InvokeRrtMainThread(() =>
                {
                    try { return !IsRrtConfigurationFree(angles); }
                    finally
                    {
                        rig.SetJointAngles(restoreAngles);
                        Physics.SyncTransforms();
                    }
                }, cancellationToken),
                MotionCost = (a, b) => ComputeWorkerMotionCost(a, b, copiedWristPenalty),
                RouteCost = (a, b) => ComputeWorkerMotionCost(a, b, copiedWristPenalty),
                TcpTravelCost = (a, b) => InvokeRrtMainThread(() => GetCachedRrtTcpTravelDistance(a, b), cancellationToken),
                TcpPathCost = states => InvokeRrtMainThread(() => GetTcpPathTravelDistance(states), cancellationToken),
                IsJointSegmentFree = (a, b) =>
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var key = new ExactJointSegmentKey(a, b);
                    if (sweptSegmentCache.TryGetValue(key, out bool free)) return free;
                    free = IsJointSegmentFreeOnWorker(a, b, restoreAngles, cancellationToken);
                    sweptSegmentCache[key] = free;
                    return free;
                }
            };
            return world;
        }

        private bool IsJointSegmentFreeOnWorker(float[] from, float[] to, float[] restoreAngles,
            CancellationToken cancellationToken)
        {
            double squaredDistance = 0.0;
            for (int joint = 0; joint < from.Length; joint++)
            {
                double delta = from[joint] - to[joint];
                squaredDistance += delta * delta;
            }
            int steps = Math.Max(1, (int)Math.Ceiling(Math.Sqrt(squaredDistance) / collisionResolutionDegrees));
            return InvokeRrtMainThread(() =>
            {
                try
                {
                    for (int step = 0; step <= steps; step++)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        float t = step / (float)steps;
                        var sample = new float[from.Length];
                        for (int joint = 0; joint < sample.Length; joint++)
                            sample[joint] = from[joint] + (to[joint] - from[joint]) * t;
                        if (!IsRrtConfigurationFree(sample)) return false;
                    }
                    return true;
                }
                finally
                {
                    rig.SetJointAngles(restoreAngles);
                    Physics.SyncTransforms();
                }
            }, cancellationToken);
        }

        private bool IsRrtConfigurationFree(float[] angles)
        {
            var key = new JointConfigurationKey(angles);
            if (rrtCollisionCache.TryGetValue(key, out bool free)) return free;
            bool set = rig.SetJointAngles(angles);
            Physics.SyncTransforms();
            free = set && !IsInCollision();
            rrtCollisionCache[key] = free;
            return free;
        }

        private RobotBaseSixAxisPlannerAlgorithm.Kind GetAlgorithmKind() => plannerAlgorithm switch
        {
            PlannerAlgorithm.RRTConnectFast => RobotBaseSixAxisPlannerAlgorithm.Kind.RrtConnectFast,
            PlannerAlgorithm.RRTConnectOptimized => RobotBaseSixAxisPlannerAlgorithm.Kind.RrtConnectOptimized,
            PlannerAlgorithm.JointSpacePrm => RobotBaseSixAxisPlannerAlgorithm.Kind.JointSpacePrm,
            PlannerAlgorithm.InformedRrtStar => RobotBaseSixAxisPlannerAlgorithm.Kind.InformedRrtStar,
            PlannerAlgorithm.CartesianArc => RobotBaseSixAxisPlannerAlgorithm.Kind.CartesianArc,
            _ => RobotBaseSixAxisPlannerAlgorithm.Kind.GridAStar
        };

        private float GetTcpPathTravelDistance(IReadOnlyList<float[]> states)
        {
            float total = 0f;
            for (int i = 1; i < states.Count; i++)
                total += GetCachedRrtTcpTravelDistance(states[i - 1], states[i]);
            return total;
        }

        private float GetCachedRrtTcpTravelDistance(float[] from, float[] to)
        {
            var key = new JointSegmentKey(from, to);
            if (rrtTcpTravelCache.TryGetValue(key, out float distance)) return distance;
            distance = GetTcpTravelDistance(from, to);
            rrtTcpTravelCache[key] = distance;
            return distance;
        }

        private readonly struct JointConfigurationKey : IEquatable<JointConfigurationKey>
        {
            private readonly int a, b, c, d, e, f;
            public JointConfigurationKey(float[] angles)
            {
                a = Mathf.RoundToInt(angles[0] * 100f); b = Mathf.RoundToInt(angles[1] * 100f);
                c = Mathf.RoundToInt(angles[2] * 100f); d = Mathf.RoundToInt(angles[3] * 100f);
                e = Mathf.RoundToInt(angles[4] * 100f); f = Mathf.RoundToInt(angles[5] * 100f);
            }
            public bool Equals(JointConfigurationKey other) => a == other.a && b == other.b && c == other.c &&
                d == other.d && e == other.e && f == other.f;
            public override bool Equals(object obj) => obj is JointConfigurationKey other && Equals(other);
            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = a;
                    hash = hash * 31 + b; hash = hash * 31 + c; hash = hash * 31 + d;
                    hash = hash * 31 + e; hash = hash * 31 + f;
                    return hash;
                }
            }
        }

        private readonly struct JointSegmentKey : IEquatable<JointSegmentKey>
        {
            private readonly JointConfigurationKey first, second;
            public JointSegmentKey(float[] from, float[] to)
            {
                first = new JointConfigurationKey(from);
                second = new JointConfigurationKey(to);
            }
            public bool Equals(JointSegmentKey other) =>
                (first.Equals(other.first) && second.Equals(other.second)) ||
                (first.Equals(other.second) && second.Equals(other.first));
            public override bool Equals(object obj) => obj is JointSegmentKey other && Equals(other);
            public override int GetHashCode() => first.GetHashCode() ^ second.GetHashCode();
        }

        // Exact endpoints: a nearby edge is never accepted merely because a cached
        // edge was free. Owned by one worker search, so no main-thread queue is
        // required when the identical swept segment is requested again.
        private readonly struct ExactJointSegmentKey : IEquatable<ExactJointSegmentKey>
        {
            private readonly float[] first, second;
            public ExactJointSegmentKey(float[] from, float[] to)
            { first = (float[])from.Clone(); second = (float[])to.Clone(); }
            public bool Equals(ExactJointSegmentKey other) =>
                (Same(first, other.first) && Same(second, other.second)) ||
                (Same(first, other.second) && Same(second, other.first));
            public override bool Equals(object obj) => obj is ExactJointSegmentKey other && Equals(other);
            public override int GetHashCode() => Hash(first) ^ Hash(second);
            private static bool Same(float[] a, float[] b)
            {
                if (ReferenceEquals(a, b)) return true;
                if (a == null || b == null || a.Length != b.Length) return false;
                for (int i = 0; i < a.Length; i++) if (!a[i].Equals(b[i])) return false;
                return true;
            }
            private static int Hash(float[] value)
            {
                if (value == null) return 0;
                int hash = 17;
                unchecked { for (int i = 0; i < value.Length; i++) hash = hash * 31 + value[i].GetHashCode(); }
                return hash;
            }
        }

        private static float ComputeWorkerMotionCost(float[] a, float[] b, float wristPenalty)
        {
            double sum = 0.0;
            for (int joint = 0; joint < a.Length; joint++)
            {
                double delta = a[joint] - b[joint];
                double weight = joint >= 3 ? wristPenalty : 1.0;
                sum += weight * delta * delta;
            }
            return (float)Math.Sqrt(sum);
        }

        private IEnumerator TryConnectSequenceOverFrames(float[] from, List<List<float[]>> stages,
            int stageIndex, float[] restoreAngles, Action<bool> completed)
        {
            if (stageIndex >= stages.Count)
            {
                completed(true);
                yield break;
            }

            List<float[]> candidates = stages[stageIndex];
            SortByDistance(candidates, from);
            int pathPrefixCount = path.Count;
            for (int i = 0; i < candidates.Count; i++)
            {
                if (path.Count > pathPrefixCount) path.RemoveRange(pathPrefixCount, path.Count - pathPrefixCount);
                bool connected = false;
                string segmentLabel = (stageIndex == 0 ? "Start" : $"Intermediate waypoint {stageIndex}") +
                    " to " + (stageIndex == stages.Count - 1 ? "End Target" : $"intermediate waypoint {stageIndex + 1}");
                yield return StartCoroutine(TryConnectOverFrames(from, candidates[i], restoreAngles,
                    result => connected = result, segmentLabel));
                if (!connected) continue;

                bool remainderConnected = false;
                yield return StartCoroutine(TryConnectSequenceOverFrames(candidates[i], stages,
                    stageIndex + 1, restoreAngles, result => remainderConnected = result));
                if (remainderConnected)
                {
                    completed(true);
                    yield break;
                }
            }

            if (path.Count > pathPrefixCount) path.RemoveRange(pathPrefixCount, path.Count - pathPrefixCount);
            completed(false);
        }

        private float GetSegmentPlanningCost(float[] from, float[] to)
        {
            float cost = MotionCost(from, to);
            if (tcpRotationMotionPenalty <= 0f) return cost;

            float[] restore = rig.GetJointAnglesDegrees();
            Quaternion previousRotation = default;
            float rotationTravelDegrees = 0f;
            // Ranking only: collision checks use the finer configured resolution.
            int samples = Mathf.Max(1, Mathf.CeilToInt(Distance(from, to) / 10f));
            for (int sample = 0; sample <= samples; sample++)
            {
                rig.SetJointAngles(Interpolate(from, to, (float)sample / samples));
                Quaternion rotation = GetCurrentTcp().rotation;
                if (sample > 0) rotationTravelDegrees += Quaternion.Angle(previousRotation, rotation);
                previousRotation = rotation;
            }

            rig.SetJointAngles(restore);
            return cost + tcpRotationMotionPenalty * rotationTravelDegrees;
        }

        private float GetTcpTravelDistance(float[] from, float[] to)
        {
            float[] restore = rig.GetJointAnglesDegrees();
            // Route scoring is a heuristic evaluated for many RRT candidates. FK reads
            // Transform positions directly, so physics synchronization is unnecessary;
            // reserve Physics.SyncTransforms for actual collider checks. Cap samples to
            // keep optimized search responsive even for long joint-space shortcuts.
            int samples = Mathf.Clamp(Mathf.CeilToInt(Distance(from, to) / 15f), 1, 8);
            rig.SetJointAngles(from);
            Vector3 previous = GetCurrentTcp().position;
            float distance = 0f;
            for (int sample = 1; sample <= samples; sample++)
            {
                rig.SetJointAngles(Interpolate(from, to, sample / (float)samples));
                Vector3 current = GetCurrentTcp().position;
                distance += Vector3.Distance(previous, current);
                previous = current;
            }
            rig.SetJointAngles(restore);
            return distance;
        }

        private string GetPlannerAlgorithmLabel() =>
            plannerAlgorithm == PlannerAlgorithm.RRTConnectFast ? "Fast RRT-Connect" :
            plannerAlgorithm == PlannerAlgorithm.JointSpacePrm ? "Joint-Space PRM" :
            plannerAlgorithm == PlannerAlgorithm.InformedRrtStar ? "Informed RRT*" :
            plannerAlgorithm == PlannerAlgorithm.CartesianArc ? "Cartesian Arc" :
            plannerAlgorithm == PlannerAlgorithm.GridAStar ? "3D Grid A*" : "Optimized RRT-Connect";

        private struct GridCell : IEquatable<GridCell>
        {
            public readonly int x, y, z;
            public GridCell(int x, int y, int z) { this.x = x; this.y = y; this.z = z; }
            public bool Equals(GridCell other) => x == other.x && y == other.y && z == other.z;
            public override bool Equals(object obj) => obj is GridCell other && Equals(other);
            public override int GetHashCode() => unchecked((x * 73856093) ^ (y * 19349663) ^ (z * 83492791));
        }

        private struct GridOpenEntry : IComparable<GridOpenEntry>
        {
            public readonly GridCell cell;
            public readonly float score;
            public readonly int order;
            public GridOpenEntry(GridCell cell, float score, int order) { this.cell = cell; this.score = score; this.order = order; }
            public int CompareTo(GridOpenEntry other)
            {
                int result = score.CompareTo(other.score);
                return result != 0 ? result : order.CompareTo(other.order);
            }
        }

        private IEnumerator TryBuildGridSegmentOnWorker(float[] startAngles, Vector3 startPosition,
            Quaternion startRotation, float[] endAngles, Vector3 endPosition, Quaternion endRotation,
            List<float[]> result, Action<bool> completed)
        {
            result.Clear();
            float cellSize = Mathf.Max(0.01f, gridCellSizeMillimeters * 0.001f);
            float inflation = Mathf.Max(gridObstacleInflationMillimeters * 0.001f, minimumObstacleClearance) +
                cellSize * 0.8660254f;
            Bounds workspace = GetGridWorkspace(startPosition, endPosition);
            if (!workspace.Contains(startPosition) || !workspace.Contains(endPosition))
            {
                lastGridFailure = "Start or End Target lies outside the configured cubic grid workspace. Move Grid Workspace Center or increase Grid Workspace Size Meters.";
                Status = lastGridFailure;
                completed(false);
                yield break;
            }
            var inflated = new List<Bounds>();
            for (int i = 0; i < obstacleColliders.Length; i++)
            {
                Collider obstacle = obstacleColliders[i];
                if (obstacle == null || !obstacle.enabled || obstacle.isTrigger) continue;
                Bounds bounds = obstacle.bounds;
                bounds.Expand(2f * inflation);
                inflated.Add(bounds);
            }
            CaptureGridGizmoSnapshot(workspace, inflated, cellSize);
            Vector3 origin = workspace.min;
            Vector3Int ToCell(Vector3 p) => new Vector3Int(
                Mathf.Clamp(Mathf.FloorToInt((p.x - origin.x) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.x / cellSize) - 1)),
                Mathf.Clamp(Mathf.FloorToInt((p.y - origin.y) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.y / cellSize) - 1)),
                Mathf.Clamp(Mathf.FloorToInt((p.z - origin.z) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.z / cellSize) - 1)));
            var forbidden = new List<Vector3Int>();
            string lastRejected = null;
            for (int attempt = 0; attempt < maximumGridRouteAttempts; attempt++)
            {
                if (gridSearchCancellation == null || gridSearchCancellation.IsCancellationRequested)
                { completed(false); yield break; }
                var snapshot = new RobotBaseSixAxisPlannerAlgorithm.GridSearchSnapshot
                {
                    Workspace = workspace,
                    InflatedObstacles = inflated.ToArray(),
                    StartPosition = startPosition,
                    GoalPosition = endPosition,
                    CellSize = cellSize,
                    MaximumNodes = maximumGridNodes,
                    ForbiddenCells = forbidden.ToArray(),
                    CancellationToken = gridSearchCancellation.Token
                };
                lastGridFailure = $"Searching occupancy grid on worker thread (route {attempt + 1}/{maximumGridRouteAttempts})...";
                Task<RobotBaseSixAxisPlannerAlgorithm.GridSearchResult> search = Task.Run(
                    () => RobotBaseSixAxisPlannerAlgorithm.SearchGridSnapshot(snapshot));
                while (!search.IsCompleted)
                {
                    Status = $"3D Grid A*: searching route {attempt + 1}/{maximumGridRouteAttempts} on a worker thread...";
                    yield return null;
                }
                if (gridSearchCancellation == null || gridSearchCancellation.IsCancellationRequested)
                { completed(false); yield break; }
                if (search.IsFaulted)
                {
                    lastGridFailure = $"Background grid search failed: {search.Exception?.GetBaseException().Message}";
                    completed(false);
                    yield break;
                }
                RobotBaseSixAxisPlannerAlgorithm.GridSearchResult route = search.Result;
                gridRouteAttempts++;
                planningIterations += route.ExpandedNodes;
                if (!route.Found)
                {
                    lastGridFailure = route.FailureReason;
                    gizmoGridFailurePosition = route.FailurePosition;
                    hasGridFailurePosition = true;
                    completed(false);
                    yield break;
                }
                gizmoGridRoutePoints = route.Points.ToArray();
                if (TryValidateGridPoints(startAngles, endAngles, startPosition, startRotation,
                        endPosition, endRotation, route.Points, result, out int failedPoint))
                {
                    completed(true);
                    yield break;
                }
                lastRejected = lastGridFailure;
                lastGridFailure = $"Worker route {attempt + 1} rejected by main-thread IK/collider validation: {lastRejected}";
                int rejectIndex = failedPoint > 0 && failedPoint < route.Points.Count - 1
                    ? failedPoint : route.Points.Count / 2;
                if (rejectIndex <= 0 || rejectIndex >= route.Points.Count - 1)
                {
                    completed(false);
                    yield break;
                }
                Vector3Int rejected = ToCell(route.Points[rejectIndex]);
                if (!forbidden.Contains(rejected)) forbidden.Add(rejected);
                else
                {
                    completed(false);
                    yield break;
                }
            }
            lastGridFailure = $"Tried {maximumGridRouteAttempts} worker-thread grid routes. Last rejected route: {lastRejected}";
            completed(false);
        }

        private void CaptureGridGizmoSnapshot(Bounds workspace, List<Bounds> inflated, float cellSize)
        {
            gizmoGridCellSize = cellSize;
            gizmoGridWorkspace = workspace;
            hasGridWorkspace = true;
            Vector3 origin = workspace.min;
            Vector3Int ToCell(Vector3 p) => new Vector3Int(
                Mathf.Clamp(Mathf.FloorToInt((p.x - origin.x) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.x / cellSize) - 1)),
                Mathf.Clamp(Mathf.FloorToInt((p.y - origin.y) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.y / cellSize) - 1)),
                Mathf.Clamp(Mathf.FloorToInt((p.z - origin.z) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.z / cellSize) - 1)));
            Vector3 Center(Vector3Int c) => origin + new Vector3((c.x + 0.5f) * cellSize, (c.y + 0.5f) * cellSize, (c.z + 0.5f) * cellSize);
            float estimate = 0f;
            for (int i = 0; i < inflated.Count; i++)
            {
                Vector3 size = inflated[i].size;
                estimate += Mathf.Max(1f, size.x / cellSize) * Mathf.Max(1f, size.y / cellSize) * Mathf.Max(1f, size.z / cellSize);
            }
            int stride = Mathf.Max(1, Mathf.CeilToInt(Mathf.Pow(estimate / Mathf.Max(1, maximumGridGizmoCells), 1f / 3f)));
            var cells = new List<Vector3>(Mathf.Min(maximumGridGizmoCells, 2048));
            for (int i = 0; i < inflated.Count && cells.Count < maximumGridGizmoCells; i++)
            {
                Bounds b = inflated[i];
                if (!workspace.Intersects(b)) continue;
                Vector3Int low = ToCell(b.min), high = ToCell(b.max);
                for (int x = low.x; x <= high.x && cells.Count < maximumGridGizmoCells; x += stride)
                for (int y = low.y; y <= high.y && cells.Count < maximumGridGizmoCells; y += stride)
                for (int z = low.z; z <= high.z && cells.Count < maximumGridGizmoCells; z += stride)
                    cells.Add(Center(new Vector3Int(x, y, z)));
            }
            gizmoGridOccupiedCells = cells.ToArray();
        }

        private Bounds GetGridWorkspace(Vector3 startPosition, Vector3 endPosition)
        {
            Vector3 center = gridWorkspaceCenter != null
                ? gridWorkspaceCenter.position
                : startPose != null && endTarget != null
                    ? (startPose.position + endTarget.position) * 0.5f
                    : (startPosition + endPosition) * 0.5f;
            float size = Mathf.Max(0.1f, gridWorkspaceSizeMeters);
            gizmoGridWorkspace = new Bounds(center, Vector3.one * size);
            hasGridWorkspace = true;
            return gizmoGridWorkspace;
        }

        private IEnumerator TryBuildGridTailOnWorker(float[] fromAngles, Vector3 fromPosition,
            Quaternion fromRotation, List<Transform> poses, List<List<float[]>> poseSolutions,
            int stageIndex, List<float[]> accumulatedPath, Quaternion defaultRotation, Action<bool> completed)
        {
            if (stageIndex >= poses.Count) { completed(true); yield break; }
            Transform pose = poses[stageIndex];
            GetPlanningPose(pose, defaultRotation, out Vector3 targetPosition, out Quaternion targetRotation);
            List<float[]> candidates = poseSolutions[stageIndex];
            SortByDistance(candidates, fromAngles);
            int prefixCount = accumulatedPath.Count;
            for (int i = 0; i < candidates.Count; i++)
            {
                var segment = new List<float[]>();
                bool segmentFound = false;
                yield return StartCoroutine(TryBuildGridSegmentOnWorker(fromAngles, fromPosition, fromRotation,
                    candidates[i], targetPosition, targetRotation, segment, ok => segmentFound = ok));
                if (!segmentFound) continue;
                for (int p = 1; p < segment.Count; p++) accumulatedPath.Add(segment[p]);
                bool tailFound = false;
                yield return StartCoroutine(TryBuildGridTailOnWorker(candidates[i], targetPosition, targetRotation,
                    poses, poseSolutions, stageIndex + 1, accumulatedPath, defaultRotation, ok => tailFound = ok));
                if (tailFound) { completed(true); yield break; }
                accumulatedPath.RemoveRange(prefixCount, accumulatedPath.Count - prefixCount);
            }
            completed(false);
        }

        private bool TryBuildGridSegment(float[] startAngles, Vector3 startPosition, Quaternion startRotation,
            float[] endAngles, Vector3 endPosition, Quaternion endRotation, List<float[]> result)
        {
            result.Clear();
            float cellSize = Mathf.Max(0.01f, gridCellSizeMillimeters * 0.001f);
            float inflation = Mathf.Max(gridObstacleInflationMillimeters * 0.001f, minimumObstacleClearance) +
                cellSize * 0.8660254f;
            Bounds workspace = GetGridWorkspace(startPosition, endPosition);
            if (!workspace.Contains(startPosition) || !workspace.Contains(endPosition))
            {
                lastGridFailure = "Start or End Target lies outside the configured cubic grid workspace. Move Grid Workspace Center or increase Grid Workspace Size Meters.";
                Status = lastGridFailure;
                return false;
            }
            Vector3 origin = workspace.min;
            GridCell ToCell(Vector3 p) => new GridCell(
                Mathf.Clamp(Mathf.FloorToInt((p.x - origin.x) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.x / cellSize) - 1)),
                Mathf.Clamp(Mathf.FloorToInt((p.y - origin.y) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.y / cellSize) - 1)),
                Mathf.Clamp(Mathf.FloorToInt((p.z - origin.z) / cellSize), 0, Mathf.Max(0, Mathf.CeilToInt(workspace.size.z / cellSize) - 1)));
            Vector3 Center(GridCell c) => origin + new Vector3((c.x + 0.5f) * cellSize, (c.y + 0.5f) * cellSize, (c.z + 0.5f) * cellSize);
            bool Blocked(GridCell c)
            {
                Vector3 p = Center(c);
                for (int i = 0; i < obstacleColliders.Length; i++)
                {
                    Collider obstacle = obstacleColliders[i];
                    if (obstacle == null || !obstacle.enabled || obstacle.isTrigger) continue;
                    Bounds b = obstacle.bounds;
                    b.Expand(2f * inflation);
                    if (b.Contains(p)) return true;
                }
                return false;
            }

            void CaptureGridGizmos()
            {
                gizmoGridCellSize = cellSize;
                var cellsForGizmos = new List<Vector3>();
                float estimatedCells = 0f;
                for (int i = 0; i < obstacleColliders.Length; i++)
                {
                    Collider obstacle = obstacleColliders[i];
                    if (obstacle == null || !obstacle.enabled || obstacle.isTrigger) continue;
                    Bounds b = obstacle.bounds;
                    b.Expand(2f * inflation);
                    Vector3 size = b.size;
                    estimatedCells += Mathf.Max(1f, size.x / cellSize) *
                        Mathf.Max(1f, size.y / cellSize) * Mathf.Max(1f, size.z / cellSize);
                }
                int stride = Mathf.Max(1, Mathf.CeilToInt(Mathf.Pow(
                    estimatedCells / Mathf.Max(1, maximumGridGizmoCells), 1f / 3f)));
                for (int i = 0; i < obstacleColliders.Length && cellsForGizmos.Count < maximumGridGizmoCells; i++)
                {
                    Collider obstacle = obstacleColliders[i];
                    if (obstacle == null || !obstacle.enabled || obstacle.isTrigger) continue;
                    Bounds b = obstacle.bounds;
                    b.Expand(2f * inflation);
                    GridCell low = ToCell(b.min), high = ToCell(b.max);
                    for (int x = low.x; x <= high.x && cellsForGizmos.Count < maximumGridGizmoCells; x += stride)
                    for (int y = low.y; y <= high.y && cellsForGizmos.Count < maximumGridGizmoCells; y += stride)
                    for (int z = low.z; z <= high.z && cellsForGizmos.Count < maximumGridGizmoCells; z += stride)
                    {
                        var cell = new GridCell(x, y, z);
                        if (Blocked(cell)) cellsForGizmos.Add(Center(cell));
                    }
                }
                gizmoGridOccupiedCells = cellsForGizmos.ToArray();
            }

            GridCell start = ToCell(startPosition), goal = ToCell(endPosition);
            lastGridFailure = "Searching the occupancy grid.";
            CaptureGridGizmos();
            if (start.Equals(goal))
            {
                var directPoints = new List<Vector3> { startPosition, endPosition };
                gizmoGridRoutePoints = directPoints.ToArray();
                return TryValidateGridPoints(startAngles, endAngles, startPosition, startRotation,
                    endPosition, endRotation, directPoints, result, out _);
            }
            var forbiddenCells = new HashSet<GridCell>();
            string lastRejectedRoute = null;
            for (int routeAttempt = 0; routeAttempt < maximumGridRouteAttempts; routeAttempt++)
            {
                gridRouteAttempts++;
                var open = new SortedSet<GridOpenEntry>();
                var costs = new Dictionary<GridCell, float> { [start] = 0f };
                var parents = new Dictionary<GridCell, GridCell>();
                int order = 0, expanded = 0;
                open.Add(new GridOpenEntry(start, Vector3.Distance(Center(start), Center(goal)), order++));
                bool found = false;
                while (open.Count > 0 && expanded < maximumGridNodes)
                {
                    GridOpenEntry currentEntry = open.Min;
                    open.Remove(currentEntry);
                    GridCell current = currentEntry.cell;
                    float currentCost = costs[current];
                    float h = Vector3.Distance(Center(current), Center(goal));
                    if (Mathf.Abs(currentEntry.score - (currentCost + h)) > 0.0001f) continue;
                    expanded++;
                    if (current.Equals(goal)) { found = true; break; }
                    for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        var next = new GridCell(current.x + dx, current.y + dy, current.z + dz);
                        if (next.x < 0 || next.y < 0 || next.z < 0 ||
                            next.x > Mathf.CeilToInt(workspace.size.x / cellSize) ||
                            next.y > Mathf.CeilToInt(workspace.size.y / cellSize) ||
                            next.z > Mathf.CeilToInt(workspace.size.z / cellSize)) continue;
                        if ((!next.Equals(goal) && !next.Equals(start) && Blocked(next)) || forbiddenCells.Contains(next)) continue;
                        bool cornerBlocked = false;
                        int changedAxes = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
                        int subsetCount = 1 << changedAxes;
                        for (int subset = 1; subset < subsetCount; subset++)
                        {
                            int bit = 0, sx = current.x, sy = current.y, sz = current.z;
                            if (dx != 0) { if ((subset & (1 << bit)) != 0) sx += dx; bit++; }
                            if (dy != 0) { if ((subset & (1 << bit)) != 0) sy += dy; bit++; }
                            if (dz != 0 && (subset & (1 << bit)) != 0) sz += dz;
                            var lineCell = new GridCell(sx, sy, sz);
                            if ((!lineCell.Equals(start) && !lineCell.Equals(goal) && Blocked(lineCell)) ||
                                forbiddenCells.Contains(lineCell))
                            { cornerBlocked = true; break; }
                        }
                        if (cornerBlocked) continue;
                        float nextCost = currentCost + Vector3.Distance(Center(current), Center(next));
                        if (costs.TryGetValue(next, out float oldCost) && oldCost <= nextCost) continue;
                        costs[next] = nextCost;
                        parents[next] = current;
                        open.Add(new GridOpenEntry(next, nextCost + Vector3.Distance(Center(next), Center(goal)), order++));
                    }
                }
                planningIterations += expanded;
                if (!found)
                {
                    GridCell frontier = start;
                    float closestDistance = Vector3.Distance(Center(start), Center(goal));
                    foreach (GridCell visited in costs.Keys)
                    {
                        float distance = Vector3.Distance(Center(visited), Center(goal));
                        if (distance < closestDistance) { closestDistance = distance; frontier = visited; }
                    }
                    gizmoGridFailurePosition = Center(frontier);
                    hasGridFailurePosition = true;
                    if (routeAttempt > 0 && lastRejectedRoute != null)
                        lastGridFailure = $"No additional grid route was available after trying {routeAttempt} route(s). Last rejected route: {lastRejectedRoute}";
                    else
                        lastGridFailure = expanded >= maximumGridNodes
                            ? $"A* reached the {maximumGridNodes} node limit before reaching the goal. Increase Maximum Grid Nodes or use a larger Grid Cell Size Millimeters."
                            : "A* could not connect the start and goal cells. Reduce Grid Obstacle Inflation Millimeters or use a smaller Grid Cell Size Millimeters.";
                    return false;
                }
                var cells = new List<GridCell> { goal };
                GridCell cursor = goal;
                while (!cursor.Equals(start))
                {
                    if (!parents.TryGetValue(cursor, out cursor))
                    {
                        lastGridFailure = "A* reached the goal, but the parent chain could not be reconstructed.";
                        return false;
                    }
                    cells.Add(cursor);
                }
                cells.Reverse();
                var simplifiedCells = new List<GridCell> { cells[0] };
                int anchor = 0;
                while (anchor < cells.Count - 1)
                {
                    int furthest = anchor + 1;
                    for (int candidate = cells.Count - 1; candidate > anchor + 1; candidate--)
                    {
                        Vector3 from = Center(cells[anchor]), to = Center(cells[candidate]);
                        int samples = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(from, to) / (cellSize * 0.5f)));
                        bool clear = true;
                        for (int sample = 1; sample < samples; sample++)
                        {
                            GridCell lineCell = ToCell(Vector3.Lerp(from, to, sample / (float)samples));
                            if ((!lineCell.Equals(start) && !lineCell.Equals(goal) && Blocked(lineCell)) ||
                                forbiddenCells.Contains(lineCell))
                            { clear = false; break; }
                        }
                        if (!clear) continue;
                        furthest = candidate;
                        break;
                    }
                    simplifiedCells.Add(cells[furthest]);
                    anchor = furthest;
                }
                var points = new List<Vector3>(simplifiedCells.Count) { startPosition };
                for (int i = 1; i < simplifiedCells.Count - 1; i++) points.Add(Center(simplifiedCells[i]));
                points.Add(endPosition);
                gizmoGridRoutePoints = points.ToArray();
                if (TryValidateGridPoints(startAngles, endAngles, startPosition, startRotation,
                        endPosition, endRotation, points, result, out int failedPointIndex)) return true;

                lastRejectedRoute = lastGridFailure;
                GridCell rejectedCell;
                if (failedPointIndex > 0 && failedPointIndex < simplifiedCells.Count - 1)
                    rejectedCell = simplifiedCells[failedPointIndex];
                else if (cells.Count > 2)
                    rejectedCell = cells[Mathf.Clamp(cells.Count / 2, 1, cells.Count - 2)];
                else
                {
                    lastGridFailure = $"The only grid cell route was rejected: {lastRejectedRoute}";
                    return false;
                }
                forbiddenCells.Add(rejectedCell);
                lastGridFailure = $"Route {routeAttempt + 1} rejected: {lastRejectedRoute}; searching an alternative.";
            }
            lastGridFailure = $"Tried {maximumGridRouteAttempts} grid routes. Last rejection: {lastRejectedRoute}";
            return false;
        }

        private bool TryValidateGridPoints(float[] startAngles, float[] endAngles,
            Vector3 startPosition, Quaternion startRotation, Vector3 endPosition, Quaternion endRotation,
            List<Vector3> points, List<float[]> result, out int failedPointIndex)
        {
            failedPointIndex = -1;
            hasGridFailurePosition = false;
            float[] restore = rig.GetJointAnglesDegrees();
            var chosenPath = new List<float[]>(points.Count) { (float[])startAngles.Clone() };
            float[] previous = (float[])startAngles.Clone();
            float totalLength = 0f;
            for (int i = 1; i < points.Count; i++) totalLength += Vector3.Distance(points[i - 1], points[i]);
            float passed = 0f;
            for (int i = 1; i < points.Count; i++)
            {
                passed += Vector3.Distance(points[i - 1], points[i]);
                float t = totalLength <= 1e-6f ? 1f : passed / totalLength;
                Vector3 position = i == points.Count - 1 ? endPosition : points[i];
                Quaternion rotation = Quaternion.Slerp(startRotation, endRotation, t);
                float[] selected = null;
                if (i == points.Count - 1)
                {
                    selected = (float[])endAngles.Clone();
                    if (!IsSegmentFree(previous, selected))
                    {
                        IsInCollision(out string collisionPair);
                        lastGridFailure = $"The final joint transition collides or violates minimum clearance" +
                            (string.IsNullOrEmpty(collisionPair) ? "." : $" ({collisionPair}).");
                        selected = null;
                    }
                }
                else if (!inverseKinematics.SolveAll(position, rotation, out List<float[]> solutions))
                    lastGridFailure = $"The analytic IK solver found no solution for grid waypoint {i}. Try a smaller Grid Cell Size Millimeters or a different TCP orientation.";
                else
                {
                    solutions.Sort((a, b) => MotionCost(previous, a).CompareTo(MotionCost(previous, b)));
                    bool closureMatched = false, collisionFreePose = false, freeTransition = false;
                    string firstPoseCollision = null;
                    for (int k = 0; k < solutions.Count; k++)
                    {
                        float[] q = solutions[k];
                        if (!rig.SetJointAngles(q)) continue;
                        Physics.SyncTransforms();
                        Transform tcp = GetCurrentTcp();
                        if (Vector3.Distance(tcp.position, position) > positionClosureTolerance ||
                            Quaternion.Angle(tcp.rotation, rotation) > rotationClosureToleranceDegrees) continue;
                        closureMatched = true;
                        if (IsInCollision(out string poseCollisionPair))
                        {
                            if (string.IsNullOrEmpty(firstPoseCollision)) firstPoseCollision = poseCollisionPair;
                            continue;
                        }
                        collisionFreePose = true;
                        if (!IsSegmentFree(previous, q)) continue;
                        freeTransition = true;
                        selected = (float[])q.Clone();
                        break;
                    }
                    if (selected == null)
                        lastGridFailure = !closureMatched
                            ? $"No IK branch matched grid waypoint {i} within pose closure tolerances. Try a smaller Grid Cell Size Millimeters."
                            : !collisionFreePose
                                ? $"All IK branches at grid waypoint {i} collide or violate minimum clearance" +
                                  (string.IsNullOrEmpty(firstPoseCollision) ? "." : $" ({firstPoseCollision}).")
                                : !freeTransition
                                    ? $"The joint transition into grid waypoint {i} collides or violates minimum clearance. Try a smaller Grid Cell Size Millimeters."
                                    : $"No collision-free IK branch for grid waypoint {i}.";
                }
                if (selected == null)
                {
                    failedPointIndex = i;
                    if (!hasGridFailurePosition)
                        SetGridFailurePosition(position);
                    rig.SetJointAngles(restore);
                    Physics.SyncTransforms();
                    return false;
                }
                chosenPath.Add(selected);
                previous = selected;
            }
            rig.SetJointAngles(restore);
            Physics.SyncTransforms();
            result.AddRange(chosenPath);
            return true;
        }

        private bool TryBuildGridTail(float[] fromAngles, Vector3 fromPosition, Quaternion fromRotation,
            List<Transform> poses, List<List<float[]>> poseSolutions, int stageIndex,
            List<float[]> accumulatedPath, Quaternion defaultRotation)
        {
            if (stageIndex >= poses.Count) return true;
            Transform pose = poses[stageIndex];
            GetPlanningPose(pose, defaultRotation, out Vector3 targetPosition, out Quaternion targetRotation);
            List<float[]> candidates = poseSolutions[stageIndex];
            SortByDistance(candidates, fromAngles);
            int prefixCount = accumulatedPath.Count;
            for (int i = 0; i < candidates.Count; i++)
            {
                var segment = new List<float[]>();
                if (!TryBuildGridSegment(fromAngles, fromPosition, fromRotation,
                        candidates[i], targetPosition, targetRotation, segment)) continue;
                for (int p = 1; p < segment.Count; p++) accumulatedPath.Add(segment[p]);
                if (TryBuildGridTail(candidates[i], targetPosition, targetRotation,
                        poses, poseSolutions, stageIndex + 1, accumulatedPath, defaultRotation)) return true;
                accumulatedPath.RemoveRange(prefixCount, accumulatedPath.Count - prefixCount);
            }
            return false;
        }

        /// <summary>Delegates Cartesian arc search to the plain C# planning algorithm.</summary>
        private bool TryBuildCartesianArc(float[] startAngles, Vector3 startPosition, Quaternion startRotation,
            float[] endAngles, Vector3 endPosition, Quaternion endRotation, List<float[]> result)
        {
            result.Clear();
            if (algorithm == null) algorithm = new RobotBaseSixAxisPlannerAlgorithm(CreateAlgorithmWorld());
            if (!algorithm.TryPlan(RobotBaseSixAxisPlannerAlgorithm.Kind.CartesianArc,
                    startAngles, endAngles, startPosition, startRotation, endPosition, endRotation,
                    out RobotBaseSixAxisPlannerAlgorithm.Result plan))
            {
                cartesianArcAttempts += plan.RoutesTried;
                lastCartesianFailure = $"Tried {plan.RoutesTried} arcs. {plan.FailureReason} Clearance is {minimumObstacleClearance * 1000f:F0} mm.";
                rig.SetJointAngles(startAngles);
                Physics.SyncTransforms();
                return false;
            }
            cartesianArcAttempts += plan.RoutesTried;
            result.AddRange(plan.JointPath);
            rig.SetJointAngles(startAngles);
            Physics.SyncTransforms();
            return result.Count > 1;
        }

        private bool TryBuildCartesianTail(float[] fromAngles, Vector3 fromPosition, Quaternion fromRotation,
            List<Transform> poses, List<List<float[]>> poseSolutions, int stageIndex,
            List<float[]> accumulatedPath, Quaternion defaultRotation)
        {
            if (stageIndex >= poses.Count) return true;
            Transform pose = poses[stageIndex];
            List<float[]> candidates = poseSolutions[stageIndex];
            SortByDistance(candidates, fromAngles);
            int prefixCount = accumulatedPath.Count;
            GetPlanningPose(pose, defaultRotation, out Vector3 targetPosition, out Quaternion targetRotation);
            for (int i = 0; i < candidates.Count; i++)
            {
                var segment = new List<float[]>();
                if (!TryBuildCartesianArc(fromAngles, fromPosition, fromRotation,
                        candidates[i], targetPosition, targetRotation, segment)) continue;
                for (int p = 1; p < segment.Count; p++) accumulatedPath.Add(segment[p]);
                if (TryBuildCartesianTail(candidates[i], targetPosition, targetRotation,
                        poses, poseSolutions, stageIndex + 1, accumulatedPath, defaultRotation)) return true;
                accumulatedPath.RemoveRange(prefixCount, accumulatedPath.Count - prefixCount);
            }
            return false;
        }

        private bool TryGetIntermediateWaypointSolutions(Quaternion defaultRotation,
            out List<Transform> poses, out List<List<float[]>> solutions, out string error)
        {
            poses = new List<Transform>();
            solutions = new List<List<float[]>>();
            error = null;
            if (intermediateWaypoints == null) return true;
            for (int i = 0; i < intermediateWaypoints.Length; i++)
            {
                Transform waypoint = intermediateWaypoints[i];
                if (waypoint == null) continue;
                Quaternion rotation = useMarkerOrientations ? waypoint.rotation : defaultRotation;
                if (!inverseKinematics.SolveAll(waypoint.position, rotation, out List<float[]> candidates))
                {
                    error = $"Intermediate waypoint '{waypoint.name}' is unreachable by the analytic IK.";
                    return false;
                }
                List<float[]> valid = FilterPoseSolutions(candidates, waypoint, rotation, out PoseCheckReport report);
                if (valid.Count == 0)
                {
                    error = $"No collision-free IK solution for intermediate waypoint '{waypoint.name}'. {DescribePoseCheck(waypoint, report)}";
                    return false;
                }
                poses.Add(waypoint);
                solutions.Add(valid);
            }
            return true;
        }

        private bool HasIntermediateWaypoints()
        {
            if (intermediateWaypoints == null) return false;
            for (int i = 0; i < intermediateWaypoints.Length; i++)
                if (intermediateWaypoints[i] != null) return true;
            return false;
        }

        [ContextMenu("Plan Collision-Free Motion (Do Not Move)")]
        public bool TryPlan()
        {
            algorithm = null;
            planningIterations = 0;
            rrtCollisionCache.Clear();
            rrtTcpTravelCache.Clear();
            gridRouteAttempts = 0;
            cartesianArcAttempts = 0;
            lastPlanSummary = string.Empty;
            planningTimeSeconds = 0f;
            planningTimer = System.Diagnostics.Stopwatch.StartNew();
            path.Clear();
            ClearGridGizmoData();
            ClearPathVisualization();
            if (!ValidateSetup()) return false;
            CacheColliders();
            if (robotColliders.Length == 0 || obstacleColliders.Length == 0)
                return Fail("Collision geometry is missing. Assign robot colliders and colliders beneath Obstacles.");

            float[] originalAngles = rig.GetJointAnglesDegrees();
            try
            {
                Quaternion preferredRotation = GetCurrentTcp().rotation;
                Quaternion startRotation = useMarkerOrientations ? startPose.rotation : preferredRotation;
                Quaternion goalRotation = useMarkerOrientations ? endTarget.rotation : preferredRotation;
                InitializeGoalSelection(goalRotation);
                if (!inverseKinematics.SolveAll(startPose.position, startRotation, out List<float[]> starts))
                    return Fail("Start pose is unreachable by the analytic IK.");

                List<float[]> validStarts = FilterPoseSolutions(starts, startPose, startRotation, out PoseCheckReport startReport);
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                List<float[]> validGoals = SolveGoalPose(plannedGoalPosition, plannedGoalRotation, false, out PoseCheckReport goalReport);
                if (validStarts.Count > 0 && validGoals.Count == 0 && adaptiveGoal)
                    FindAdaptiveGoal(out validGoals);
                if (validStarts.Count == 0 || validGoals.Count == 0)
                    return Fail($"No collision-free IK solution. {DescribePoseCheck(startPose, startReport)} {DescribePoseCheck(endTarget, goalReport)}{AdaptiveGoalFailureSummary}");

                if (!TryGetIntermediateWaypointSolutions(preferredRotation,
                        out List<Transform> viaPoses, out List<List<float[]>> viaSolutions, out string waypointError))
                    return Fail(waypointError);

                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                if (IsInCollision())
                    return Fail("The robot's current pose collides with an obstacle, so a collision-free move cannot start.");

                random = new System.Random(randomSeed == 0 ? 1 : randomSeed);
                // Prefer the start and goal branches closest to the current pose, then
                // try every remaining pair because different IK branches may connect.
                SortByDistance(validStarts, originalAngles);
                validGoals.Sort((a, b) => CompareDistance(a, b, originalAngles));
                var jointSpaceStages = new List<List<float[]>>(viaSolutions) { validGoals };
                for (int s = 0; s < validStarts.Count; s++)
                {
                    path.Clear();
                    if (plannerAlgorithm == PlannerAlgorithm.CartesianArc)
                    {
                        Transform tcp = GetCurrentTcp();
                        var prefix = new List<float[]>();
                        if (!TryBuildCartesianArc(originalAngles, tcp.position, tcp.rotation,
                                validStarts[s], startPose.position, startRotation, prefix))
                        {
                            lastCartesianFailure = $"Current TCP to Start: {lastCartesianFailure}";
                            continue;
                        }
                        path.AddRange(prefix);
                        var tailPoses = new List<Transform>(viaPoses) { endTarget };
                        var tailSolutions = new List<List<float[]>>(viaSolutions) { validGoals };
                        if (TryBuildCartesianTail(validStarts[s], startPose.position, startRotation,
                                tailPoses, tailSolutions, 0, path, preferredRotation))
                        {
                            SetPlanSummary($"Plan found with Cartesian Arc after {cartesianArcAttempts} arc attempts in {FinishPlanningTimer()} ({path.Count} waypoints).");
                            Status = lastPlanSummary;
                            Debug.Log($"Motion planner: {Status}", this);
                            DrawPlannedPath();
                            return true;
                        }
                        lastCartesianFailure = $"Start through intermediate waypoint(s) to End Target: {lastCartesianFailure}";
                        continue;
                    }
                    if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                    {
                        Transform tcp = GetCurrentTcp();
                        var prefix = new List<float[]>();
                        if (!TryBuildGridSegment(originalAngles, tcp.position, tcp.rotation,
                                validStarts[s], startPose.position, startRotation, prefix)) continue;
                        path.AddRange(prefix);
                        var tailPoses = new List<Transform>(viaPoses) { endTarget };
                        var tailSolutions = new List<List<float[]>>(viaSolutions) { validGoals };
                        if (TryBuildGridTail(validStarts[s], startPose.position, startRotation,
                                tailPoses, tailSolutions, 0, path, preferredRotation))
                        {
                            hasGridFailurePosition = false;
                            SetPlanSummary($"Plan found with 3D Grid A* after {gridRouteAttempts} route attempt(s) and {planningIterations} grid nodes expanded in {FinishPlanningTimer()} ({path.Count} waypoints).");
                            Status = lastPlanSummary;
                            Debug.Log($"Motion planner: {Status}", this);
                            DrawPlannedPath();
                            return true;
                        }
                        continue;
                    }
                    if (!TryConnect(originalAngles, validStarts[s])) continue;
                    if (TryConnectSequence(validStarts[s], jointSpaceStages, 0))
                    {
                        if (!HasIntermediateWaypoints()) SimplifyPath();
                        SetPlanSummary($"Plan found with {GetPlannerAlgorithmLabel()} after {planningIterations} search iterations in {FinishPlanningTimer()} ({path.Count} waypoints).");
                        Status = lastPlanSummary;
                        Debug.Log($"Motion planner: {Status}", this);
                        DrawPlannedPath();
                        return true;
                    }
                }

                if (plannerAlgorithm == PlannerAlgorithm.CartesianArc)
                    return Fail($"Cartesian Arc failed. {lastCartesianFailure}");
                if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                    return Fail($"3D Grid A* failed after {gridRouteAttempts} route attempt(s) and {planningIterations} grid nodes expanded. {lastGridFailure}");
                return Fail($"No collision-free path found after {planningIterations} search iterations. Try more iterations or adjust the obstacles/targets.");
            }
            finally
            {
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
            }
        }

        public void StopMotion()
        {
            CancelGridSearch();
            StopAllCoroutines();
            execution = null;
            Status = "Motion stopped.";
            CurrentMotionState = MotionState.Ready;
        }

        private List<float[]> FilterPoseSolutions(List<float[]> candidates, Transform pose,
            Quaternion targetRotation, out PoseCheckReport report)
            => FilterPoseSolutions(candidates, pose.position, targetRotation, out report);

        private List<float[]> FilterPoseSolutions(List<float[]> candidates, Vector3 targetPosition,
            Quaternion targetRotation, out PoseCheckReport report, bool enforceAdaptiveBounds = false)
        {
            var valid = new List<float[]>();
            report = new PoseCheckReport { candidateCount = candidates.Count, bestPositionError = float.PositiveInfinity,
                bestRotationError = float.PositiveInfinity };
            for (int i = 0; i < candidates.Count; i++)
            {
                if (!rig.SetJointAngles(candidates[i])) continue;
                Physics.SyncTransforms();
                Transform tcp = rig.ToolFrame != null ? rig.ToolFrame : rig.Joints[5];
                float positionError = Vector3.Distance(tcp.position, targetPosition);
                float rotationError = Quaternion.Angle(tcp.rotation, targetRotation);
                report.bestPositionError = Mathf.Min(report.bestPositionError, positionError);
                report.bestRotationError = Mathf.Min(report.bestRotationError, rotationError);
                if (positionError > positionClosureTolerance || rotationError > rotationClosureToleranceDegrees) continue;
                if (enforceAdaptiveBounds &&
                    (Vector3.Distance(tcp.position, requestedGoalPosition) > adaptiveGoalPositionToleranceMillimeters * 0.001f + 0.00001f ||
                     Quaternion.Angle(tcp.rotation, requestedGoalRotation) > adaptiveGoalRotationToleranceDegrees + 0.001f)) continue;
                report.poseClosedCount++;
                if (IsInCollision(out string collisionPair))
                {
                    report.collisionCount++;
                    if (string.IsNullOrEmpty(report.firstCollisionPair)) report.firstCollisionPair = collisionPair;
                    continue;
                }
                valid.Add((float[])candidates[i].Clone());
                report.validCount++;
            }
            return valid;
        }

        private void InitializeGoalSelection(Quaternion rotation)
        {
            requestedGoalPosition = plannedGoalPosition = endTarget.position;
            requestedGoalRotation = plannedGoalRotation = rotation;
            adaptiveGoalUsed = false;
            adaptiveGoalSearchAttempted = false;
            adaptiveGoalSamplesTried = 0;
            adaptiveGoalPositionOffsetMillimeters = 0f;
            adaptiveGoalRotationOffsetDegrees = 0f;
        }

        private List<float[]> SolveGoalPose(Vector3 position, Quaternion rotation, bool adaptiveCandidate,
            out PoseCheckReport report)
        {
            float[] restore = rig.GetJointAnglesDegrees();
            try
            {
                if (!inverseKinematics.SolveAll(position, rotation, out List<float[]> solutions))
                {
                    report = new PoseCheckReport { bestPositionError = float.PositiveInfinity, bestRotationError = float.PositiveInfinity };
                    return new List<float[]>();
                }
                // Uses exactly the same IsInCollision/clearance checks as a strict goal.
                return FilterPoseSolutions(solutions, position, rotation, out report, adaptiveCandidate);
            }
            finally
            {
                rig.SetJointAngles(restore);
                Physics.SyncTransforms();
            }
        }

        private List<RobotBaseSixAxisAdaptiveGoalSearch.GoalPose> CreateAdaptiveGoalCandidates()
        {
            adaptiveGoalSearchAttempted = true;
            return RobotBaseSixAxisAdaptiveGoalSearch.CreateCandidates(requestedGoalPosition, requestedGoalRotation,
                adaptiveGoalPositionToleranceMillimeters, adaptiveGoalRotationToleranceDegrees, adaptiveGoalMaximumSamples);
        }

        private bool TryAcceptAdaptiveGoal(RobotBaseSixAxisAdaptiveGoalSearch.GoalPose candidate, out List<float[]> goals)
        {
            adaptiveGoalSamplesTried++;
            goals = SolveGoalPose(candidate.Position, candidate.Rotation, true, out _);
            if (goals.Count == 0) return false;
            plannedGoalPosition = candidate.Position;
            plannedGoalRotation = candidate.Rotation;
            adaptiveGoalPositionOffsetMillimeters = candidate.PositionOffsetMillimeters;
            adaptiveGoalRotationOffsetDegrees = candidate.RotationOffsetDegrees;
            adaptiveGoalUsed = true;
            return true;
        }

        private IEnumerator FindAdaptiveGoalOverFrames(Action<List<float[]>> completed)
        {
            var candidates = CreateAdaptiveGoalCandidates();
            var frameTimer = System.Diagnostics.Stopwatch.StartNew();
            for (int i = 0; i < candidates.Count; i++)
            {
                if (gridSearchCancellation == null || gridSearchCancellation.IsCancellationRequested)
                { completed(new List<float[]>()); yield break; }
                Status = $"Adaptive Goal: testing pose {i + 1}/{candidates.Count}; minimum clearance remains {minimumObstacleClearance * 1000f:F1} mm.";
                if (TryAcceptAdaptiveGoal(candidates[i], out List<float[]> goals))
                { completed(goals); yield break; }
                if (frameTimer.Elapsed.TotalMilliseconds >= 3.0)
                {
                    yield return null;
                    frameTimer.Restart();
                }
            }
            completed(new List<float[]>());
        }

        private bool FindAdaptiveGoal(out List<float[]> goals)
        {
            var candidates = CreateAdaptiveGoalCandidates();
            for (int i = 0; i < candidates.Count; i++)
                if (TryAcceptAdaptiveGoal(candidates[i], out goals)) return true;
            goals = new List<float[]>();
            return false;
        }

        private string AdaptiveGoalFailureSummary => !adaptiveGoalSearchAttempted ? string.Empty :
            $" Adaptive goal: no valid pose in {adaptiveGoalSamplesTried} samples within {adaptiveGoalPositionToleranceMillimeters:F1} mm / " +
            $"{adaptiveGoalRotationToleranceDegrees:F1} degrees. Required clearance remains {minimumObstacleClearance * 1000f:F1} mm.";

        private void SetPlanSummary(string summary)
        {
            lastPlanSummary = summary + (!adaptiveGoalUsed ? string.Empty :
                $" Adaptive goal: {adaptiveGoalPositionOffsetMillimeters:F2} mm position offset / {adaptiveGoalRotationOffsetDegrees:F2} degrees rotation " +
                $"({adaptiveGoalSamplesTried} samples; minimum clearance {minimumObstacleClearance * 1000f:F1} mm).");
        }

        private void GetPlanningPose(Transform pose, Quaternion defaultRotation, out Vector3 position, out Quaternion rotation)
        {
            position = pose == endTarget ? plannedGoalPosition : pose.position;
            rotation = pose == endTarget ? plannedGoalRotation : useMarkerOrientations ? pose.rotation : defaultRotation;
        }

        private string GoalWaypointLabel => adaptiveGoalUsed ? "B / Adaptive Goal" : "B / Goal";

        private string DescribePoseCheck(Transform pose, PoseCheckReport report)
        {
            if (report.candidateCount == 0)
                return $"{pose.name}: IK returned no branch (unreachable, joint limit, or posture restriction).";
            if (report.poseClosedCount == 0)
                return $"{pose.name}: {report.candidateCount} IK branch(es), none passed FK closure; best errors " +
                       $"{report.bestPositionError:F3} m / {report.bestRotationError:F1}°. ";
            if (report.validCount > 0)
                return $"{pose.name}: {report.validCount} collision-free branch(es). ";
            return $"{pose.name}: {report.poseClosedCount} pose-valid branch(es), all collide with an obstacle " +
                   $"(first: {report.firstCollisionPair}). ";
        }

        private bool TryConnect(float[] start, float[] goal)
        {
            if (algorithm == null) algorithm = new RobotBaseSixAxisPlannerAlgorithm(CreateAlgorithmWorld());
            RobotBaseSixAxisPlannerAlgorithm.Kind kind = GetAlgorithmKind();
            if (!algorithm.TryPlan(kind, start, goal, Vector3.zero, Quaternion.identity,
                    Vector3.zero, Quaternion.identity, out RobotBaseSixAxisPlannerAlgorithm.Result result))
            {
                planningIterations += result.Iterations;
                return false;
            }
            planningIterations += result.Iterations;
            path.AddRange(result.JointPath);
            return result.JointPath.Count > 1;
        }

        private RobotBaseSixAxisPlannerAlgorithm.World CreateAlgorithmWorld()
        {
            Bounds workspace = new Bounds(startPose != null ? startPose.position : transform.position, Vector3.zero);
            if (endTarget != null) workspace.Encapsulate(endTarget.position);
            var bounds = obstacleColliders ?? Array.Empty<Collider>();
            var obstacleBounds = new Vector3[bounds.Length];
            for (int i = 0; i < bounds.Length; i++)
                if (bounds[i] != null) { workspace.Encapsulate(bounds[i].bounds); obstacleBounds[i] = bounds[i].bounds.size; }
            float cell = Mathf.Max(0.01f, gridCellSizeMillimeters * 0.001f);
            float inflate = Mathf.Max(minimumObstacleClearance, gridObstacleInflationMillimeters * 0.001f) + cell * 2f;
            workspace.Expand(Vector3.one * (inflate * 2f));
            var world = new RobotBaseSixAxisPlannerAlgorithm.World
            {
                IsJointSegmentFree = IsSegmentFree,
                SolveIk = (position, rotation) => inverseKinematics.SolveAll(position, rotation, out List<float[]> solved) ? solved : null,
                GetTcpPosition = angles => { rig.SetJointAngles(angles); Physics.SyncTransforms(); return GetCurrentTcp().position; },
                GetTcpRotation = angles => { rig.SetJointAngles(angles); Physics.SyncTransforms(); return GetCurrentTcp().rotation; },
                IsConfigurationInCollision = angles => { rig.SetJointAngles(angles); Physics.SyncTransforms(); return IsInCollision(); },
                RandomConfiguration = RandomConfiguration,
                MotionCost = MotionCost,
                RouteCost = GetSegmentPlanningCost,
                TcpTravelCost = GetTcpTravelDistance,
                PrioritizeShortestPath = plannerAlgorithm == PlannerAlgorithm.RRTConnectOptimized,
                Joint6TravelCostPerDegree = joint6TravelPenaltyMillimetersPerDegree * 0.001f,
                IsCartesianPointBlocked = (point, clearance) =>
                {
                    for (int i = 0; i < bounds.Length; i++)
                    {
                        if (bounds[i] == null || !bounds[i].enabled || bounds[i].isTrigger) continue;
                        Bounds expanded = bounds[i].bounds;
                        expanded.Expand(2f * Mathf.Max(clearance, gridObstacleInflationMillimeters * 0.001f));
                        if (expanded.Contains(point)) return true;
                    }
                    return false;
                },
                JointLimits = rig != null ? rig.AngleLimitsDegrees : Array.Empty<Vector2>(),
                Workspace = workspace,
                ObstacleBounds = obstacleBounds,
                GridOrigin = workspace.min,
                GridCellSize = cell,
                Clearance = minimumObstacleClearance,
                ExtensionStepDegrees = extensionStepDegrees,
                CollisionResolutionDegrees = collisionResolutionDegrees,
                MaximumIterations = maximumIterations,
                MaximumGridNodes = maximumGridNodes,
                MaximumGridRouteAttempts = maximumGridRouteAttempts,
                GoalBias = goalBias,
                Random = random ?? new System.Random(randomSeed == 0 ? 1 : randomSeed)
            };
            return world;
        }

        private bool TryConnectSequence(float[] from, List<List<float[]>> stages, int stageIndex)
        {
            if (stageIndex >= stages.Count) return true;
            List<float[]> candidates = stages[stageIndex];
            SortByDistance(candidates, from);
            int pathPrefixCount = path.Count;
            for (int i = 0; i < candidates.Count; i++)
            {
                if (path.Count > pathPrefixCount) path.RemoveRange(pathPrefixCount, path.Count - pathPrefixCount);
                rig.SetJointAngles(from);
                Physics.SyncTransforms();
                if (!TryConnect(from, candidates[i])) continue;
                if (TryConnectSequence(candidates[i], stages, stageIndex + 1)) return true;
            }
            if (path.Count > pathPrefixCount) path.RemoveRange(pathPrefixCount, path.Count - pathPrefixCount);
            return false;
        }

        private bool IsSegmentFree(float[] from, float[] to)
        {
            int steps = Mathf.Max(1, Mathf.CeilToInt(Distance(from, to) / collisionResolutionDegrees));
            for (int step = 0; step <= steps; step++)
            {
                float[] sample = Interpolate(from, to, (float)step / steps);
                if (plannerAlgorithm == PlannerAlgorithm.RRTConnectFast ||
                    plannerAlgorithm == PlannerAlgorithm.RRTConnectOptimized ||
                    plannerAlgorithm == PlannerAlgorithm.JointSpacePrm ||
                    plannerAlgorithm == PlannerAlgorithm.InformedRrtStar)
                {
                    if (!IsRrtConfigurationFree(sample)) return false;
                }
                else
                {
                    if (!rig.SetJointAngles(sample)) return false;
                    Physics.SyncTransforms();
                    if (IsInCollision()) return false;
                }
            }
            return true;
        }

        private bool IsInCollision()
        {
            return IsInCollision(out _);
        }

        private bool IsInCollision(out string collisionPair)
        {
            collisionPair = null;
            for (int i = 0; i < robotColliders.Length; i++)
            {
                Collider robotCollider = robotColliders[i];
                if (robotCollider == null || !robotCollider.enabled || robotCollider.isTrigger) continue;
                for (int j = 0; j < obstacleColliders.Length; j++)
                {
                    Collider obstacle = obstacleColliders[j];
                    if (obstacle == null || !obstacle.enabled || obstacle.isTrigger) continue;
                    if (Physics.ComputePenetration(robotCollider, robotCollider.transform.position,
                            robotCollider.transform.rotation, obstacle, obstacle.transform.position,
                            obstacle.transform.rotation, out _, out _))
                    {
                        collisionPair = $"{robotCollider.name} ({robotCollider.GetType().Name}) / {obstacle.name} ({obstacle.GetType().Name})";
                        if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                            SetGridFailurePosition((robotCollider.ClosestPoint(obstacle.bounds.center) +
                                obstacle.ClosestPoint(robotCollider.bounds.center)) * 0.5f);
                        return true;
                    }
                    float clearance = GetColliderSeparation(robotCollider, obstacle);
                    if (minimumObstacleClearance > 0f && clearance < minimumObstacleClearance)
                    {
                        collisionPair = $"{robotCollider.name} / {obstacle.name} (clearance {clearance:F3} m < {minimumObstacleClearance:F3} m)";
                        if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                            SetGridFailurePosition((robotCollider.ClosestPoint(obstacle.bounds.center) +
                                obstacle.ClosestPoint(robotCollider.bounds.center)) * 0.5f);
                        return true;
                    }
                }
            }

            if (!checkSelfCollision) return false;
            for (int a = 0; a < robotColliders.Length; a++)
            for (int b = a + 1; b < robotColliders.Length; b++)
            {
                Collider first = robotColliders[a];
                Collider second = robotColliders[b];
                if (first == null || second == null || first.isTrigger || second.isTrigger ||
                    AreSameOrAdjacentLink(first.transform, second.transform)) continue;
                if (Physics.ComputePenetration(first, first.transform.position, first.transform.rotation,
                        second, second.transform.position, second.transform.rotation, out _, out _))
                {
                    if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                        SetGridFailurePosition((first.ClosestPoint(second.bounds.center) +
                            second.ClosestPoint(first.bounds.center)) * 0.5f);
                    return true;
                }
            }
            return false;
        }

        // Alternate closest-point projections on the actual collider surfaces.
        // Try seeds from both directions so meshes with concave sections are less
        // likely to settle on a poor local closest-point pair.
        private static float GetColliderSeparation(Collider a, Collider b)
        {
            Vector3 aPoint = a.ClosestPoint(b.bounds.center);
            float best = RefineClosestPoints(a, b, aPoint);

            Vector3 bPoint = b.ClosestPoint(a.bounds.center);
            aPoint = a.ClosestPoint(bPoint);
            return Mathf.Min(best, RefineClosestPoints(a, b, aPoint));
        }

        private static float RefineClosestPoints(Collider a, Collider b, Vector3 aPoint)
        {
            Vector3 bPoint = b.ClosestPoint(aPoint);
            for (int i = 0; i < 6; i++)
            {
                aPoint = a.ClosestPoint(bPoint);
                bPoint = b.ClosestPoint(aPoint);
            }
            return Vector3.Distance(aPoint, bPoint);
        }

        private bool AreSameOrAdjacentLink(Transform a, Transform b)
        {
            int linkA = GetOwningJointIndex(a);
            int linkB = GetOwningJointIndex(b);
            return linkA < 0 || linkB < 0 || Mathf.Abs(linkA - linkB) <= 1;
        }

        private int GetOwningJointIndex(Transform item)
        {
            Transform[] joints = rig.Joints;
            int result = -1;
            for (int i = 0; i < joints.Length; i++)
                if (item == joints[i] || item.IsChildOf(joints[i])) result = i;
            return result;
        }

        private float[] RandomConfiguration()
        {
            var result = new float[JointCount];
            Vector2[] limits = rig.AngleLimitsDegrees;
            for (int i = 0; i < JointCount; i++)
                result[i] = Mathf.Lerp(limits[i].x, limits[i].y, (float)random.NextDouble());
            return result;
        }

        private void SimplifyPath()
        {
            if (path.Count < 3) return;
            var original = new List<float[]>(path.Count);
            for (int i = 0; i < path.Count; i++)
                if (original.Count == 0 || Distance(original[original.Count - 1], path[i]) > 0.001f)
                    original.Add(path[i]);
            if (original.Count < 3) { path.Clear(); path.AddRange(original); return; }
            List<float[]> simplified;
            if (plannerAlgorithm == PlannerAlgorithm.RRTConnectOptimized &&
                TryBuildShortestShortcutPath(original, optimizedRrtWaypointBudget, out List<float[]> shortest))
                simplified = shortest;
            else
                simplified = BuildGreedyShortcutPath(original);
            path.Clear();
            path.AddRange(simplified);

            // Relax corners only when it shortens the selected TCP travel and both
            // replacement swept segments remain collision free.
            const int smoothingPasses = 24;
            const float smoothingStrength = 0.25f;
            for (int pass = 0; pass < smoothingPasses; pass++)
            {
                bool changed = false;
                for (int i = 1; i < path.Count - 1; i++)
                {
                    float[] neighborMidpoint = Interpolate(path[i - 1], path[i + 1], 0.5f);
                    float[] candidate = Interpolate(path[i], neighborMidpoint, smoothingStrength);
                    if (!IsSegmentFree(path[i - 1], candidate) || !IsSegmentFree(candidate, path[i + 1])) continue;
                    float currentCost = GetShortcutCost(path[i - 1], path[i]) + GetShortcutCost(path[i], path[i + 1]);
                    float candidateCost = GetShortcutCost(path[i - 1], candidate) + GetShortcutCost(candidate, path[i + 1]);
                    if (candidateCost > currentCost + 0.01f) continue;
                    path[i] = candidate;
                    changed = true;
                }
                if (!changed) break;
            }
        }

        private List<float[]> BuildGreedyShortcutPath(List<float[]> original)
        {
            var originalCostPrefix = new float[original.Count];
            for (int i = 1; i < original.Count; i++)
                originalCostPrefix[i] = originalCostPrefix[i - 1] + GetShortcutCost(original[i - 1], original[i]);
            var simplified = new List<float[]> { original[0] };
            int current = 0;
            while (current < original.Count - 1)
            {
                int next = current + 1;
                for (int candidate = original.Count - 1; candidate > current + 1; candidate--)
                {
                    if (!IsSegmentFree(original[current], original[candidate])) continue;
                    float shortcutCost = GetShortcutCost(original[current], original[candidate]);
                    float retainedCost = originalCostPrefix[candidate] - originalCostPrefix[current];
                    if (shortcutCost <= retainedCost + 0.01f) { next = candidate; break; }
                }
                simplified.Add(original[next]);
                current = next;
            }
            return simplified;
        }

        private bool TryBuildShortestShortcutPath(List<float[]> points, int maxWaypointCount,
            out List<float[]> shortestPath)
        {
            shortestPath = null;
            // The dynamic-programming shortcut search checks every forward edge.
            // Limiting its sampled candidate set keeps edge checks quadratic in a
            // small set while retaining the original RRT path as the safe fallback.
            const int maximumCandidatePoints = 20;
            if (points.Count > maximumCandidatePoints)
            {
                var candidates = new List<float[]>(maximumCandidatePoints) { points[0] };
                for (int i = 1; i < maximumCandidatePoints - 1; i++)
                {
                    int index = Mathf.RoundToInt(i * (points.Count - 1f) / (maximumCandidatePoints - 1));
                    if (index > 0 && index < points.Count - 1) candidates.Add(points[index]);
                }
                candidates.Add(points[points.Count - 1]);
                points = candidates;
            }
            int count = points.Count;
            int budget = Mathf.Clamp(maxWaypointCount, 2, count);
            var costs = new float[budget + 1, count];
            var parents = new int[budget + 1, count];
            var safeEdges = new bool[count, count];
            var edgeCosts = new float[count, count];
            for (int from = 0; from < count - 1; from++)
            for (int to = from + 1; to < count; to++)
            {
                safeEdges[from, to] = IsSegmentFree(points[from], points[to]);
                if (safeEdges[from, to])
                    edgeCosts[from, to] = GetShortcutCost(points[from], points[to]);
            }
            for (int n = 0; n <= budget; n++)
            for (int i = 0; i < count; i++) { costs[n, i] = float.PositiveInfinity; parents[n, i] = -1; }
            costs[1, 0] = 0f;
            for (int n = 1; n < budget; n++)
            for (int from = 0; from < count - 1; from++)
            {
                if (float.IsPositiveInfinity(costs[n, from])) continue;
                for (int to = from + 1; to < count; to++)
                {
                    if (!safeEdges[from, to]) continue;
                    float candidate = costs[n, from] + edgeCosts[from, to];
                    if (candidate >= costs[n + 1, to]) continue;
                    costs[n + 1, to] = candidate;
                    parents[n + 1, to] = from;
                }
            }

            int bestCount = -1;
            float bestCost = float.PositiveInfinity;
            for (int n = 2; n <= budget; n++)
                if (costs[n, count - 1] < bestCost) { bestCost = costs[n, count - 1]; bestCount = n; }
            if (bestCount < 0) return false;
            var indices = new List<int>(bestCount);
            int cursor = count - 1;
            for (int n = bestCount; n >= 1; n--)
            {
                indices.Add(cursor);
                if (n > 1) cursor = parents[n, cursor];
            }
            indices.Reverse();
            shortestPath = new List<float[]>(indices.Count);
            for (int i = 0; i < indices.Count; i++) shortestPath.Add(points[indices[i]]);
            return true;
        }

        private float GetShortcutCost(float[] from, float[] to) =>
            plannerAlgorithm == PlannerAlgorithm.RRTConnectOptimized
                ? GetCachedRrtTcpTravelDistance(from, to) +
                    joint6TravelPenaltyMillimetersPerDegree * 0.001f * Mathf.Abs(to[5] - from[5])
                : GetSegmentPlanningCost(from, to);

        private void BuildMotionProfile()
        {
            motionProfile = null;
            SmoothedPathCorners = RetainedSharpCorners = 0;
            if (path.Count < 2) return;
            float[] restore = rig.GetJointAnglesDegrees();
            try
            {
                motionProfile = MotionProfile.Create(path, angles =>
                {
                    if (!rig.SetJointAngles(angles)) throw new InvalidOperationException("Could not evaluate robot joint state.");
                    Transform tcp = GetCurrentTcp();
                    return new Pose(tcp.position, tcp.rotation);
                }, new MotionProfile.Settings
                {
                    LinearSpeed = Linear,
                    RotationSpeedDegree = RotationDegree,
                    TcpAccelerationMillimetersPerSecondSquared = AccelerationSpeed,
                    TcpDecelerationMillimetersPerSecondSquared = DecelerationSpeed,
                    TcpRotationAccelerationDegreesPerSecondSquared = RotationAcceleration,
                    TcpRotationDecelerationDegreesPerSecondSquared = RotationDeceleration,
                    PathAccuracyMillimeters = pathAccuracyMillimeters,
                    CollisionResolutionDegrees = collisionResolutionDegrees
                }, angles =>
                {
                    if (!rig.SetJointAngles(angles)) return false;
                    Physics.SyncTransforms();
                    return !IsInCollision();
                });
                SmoothedPathCorners = motionProfile.SmoothedPathCorners;
                RetainedSharpCorners = motionProfile.RetainedSharpCorners;
            }
            finally
            {
                rig.SetJointAngles(restore);
                Physics.SyncTransforms();
            }
        }

        private IEnumerator ExecutePath()
        {
            // Rebuild before replay as well: Inspector limits and obstacles may have changed.
            DrawPlannedPath();
            MotionProfile profile = motionProfile;
            if (profile == null) { execution = null; yield break; }
            CurrentMotionState = MotionState.Moving;
            Status = "Executing planned motion.";
            float elapsed = 0f;
            while (elapsed < profile.DurationSeconds)
            {
                elapsed += Time.deltaTime;
                rig.SetJointAngles(profile.Evaluate(elapsed));
                yield return null;
            }
            rig.SetJointAngles(profile.Evaluate(profile.DurationSeconds));
            Physics.SyncTransforms();
            float finalPositionError = Vector3.Distance(GetCurrentTcp().position, plannedGoalPosition);
            float finalRotationError = Quaternion.Angle(GetCurrentTcp().rotation, plannedGoalRotation);
            if (finalPositionError > positionClosureTolerance ||
                (useMarkerOrientations && finalRotationError > rotationClosureToleranceDegrees))
            {
                Status = $"Path ended short of End Target: TCP error {finalPositionError:F3} m / {finalRotationError:F1}°. Check the final waypoint and IK closure.";
                Debug.LogError($"Motion planner: {Status}", this);
                CurrentMotionState = MotionState.Failed;
                yield break;
            }
            execution = null;
            Status = string.IsNullOrEmpty(lastPlanSummary) ? "Motion complete." : $"Motion complete. {lastPlanSummary}";
            CurrentMotionState = MotionState.Complete;
            Debug.Log($"Motion planner: {Status}", this);
        }

        private void DrawPlannedPath()
        {
            if (path.Count < 2)
            {
                motionProfile = null;
                SmoothedPathCorners = RetainedSharpCorners = 0;
                estimatedTravelTimeSeconds = 0f;
                gizmoPath = Array.Empty<Vector3>();
                gizmoWaypoints = Array.Empty<Vector3>();
                gizmoWaypointRotations = Array.Empty<Quaternion>();
                return;
            }
            var points = new List<Vector3>();
            var waypoints = new List<Vector3>(path.Count);
            var waypointRotations = new List<Quaternion>(path.Count);
            float[] restore = rig.GetJointAnglesDegrees();
            for (int i = 0; i < path.Count; i++)
            {
                rig.SetJointAngles(path[i]);
                Transform tcp = rig.ToolFrame != null ? rig.ToolFrame : rig.Joints[5];
                waypoints.Add(tcp.position);
                waypointRotations.Add(tcp.rotation);
            }
            UpdateEstimatedTravelTime();
            if (motionProfile != null)
                foreach (float[] angles in motionProfile.SampleJointPath())
                {
                    rig.SetJointAngles(angles);
                    points.Add(GetCurrentTcp().position);
                }
            rig.SetJointAngles(restore);
            Physics.SyncTransforms();
            gizmoPath = points.ToArray();
            gizmoWaypoints = waypoints.ToArray();
            gizmoWaypointRotations = waypointRotations.ToArray();

            if (pathRenderer == null)
            {
                Transform existing = transform.Find("Planned TCP Path");
                GameObject lineObject = existing != null ? existing.gameObject : new GameObject("Planned TCP Path");
                if (existing == null) lineObject.transform.SetParent(transform, false);
                pathRenderer = lineObject.GetComponent<LineRenderer>();
                if (pathRenderer == null) pathRenderer = lineObject.AddComponent<LineRenderer>();
                pathRenderer.useWorldSpace = true;
                pathRenderer.widthMultiplier = 0.006f;
                pathRenderer.startColor = Color.cyan;
                pathRenderer.endColor = Color.cyan;
                Shader shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
                if (shader != null) pathRenderer.material = new Material(shader);
            }
            pathRenderer.positionCount = points.Count;
            pathRenderer.SetPositions(points.ToArray());
            UpdateWaypointLabels(waypoints);
            Physics.SyncTransforms();
        }

        private void UpdateEstimatedTravelTime()
        {
            BuildMotionProfile();
            estimatedTravelTimeSeconds = motionProfile != null ? motionProfile.DurationSeconds : 0f;
        }

        private void UpdateWaypointLabels(List<Vector3> waypoints)
        {
            if (!Application.isPlaying) return;
            if (waypointLabelsRoot == null)
            {
                Transform existing = transform.Find("Planned TCP Waypoint Labels");
                if (existing != null) waypointLabelsRoot = existing;
                else
                {
                    var labelsObject = new GameObject("Planned TCP Waypoint Labels");
                    labelsObject.transform.SetParent(transform, false);
                    waypointLabelsRoot = labelsObject.transform;
                }
            }

            for (int i = 0; i < waypoints.Count; i++)
            {
                while (waypointLabels.Count <= i)
                {
                    var labelObject = new GameObject($"Waypoint Label {waypointLabels.Count}");
                    labelObject.transform.SetParent(waypointLabelsRoot, false);
                    TextMesh text = labelObject.AddComponent<TextMesh>();
                    text.anchor = TextAnchor.MiddleCenter;
                    text.alignment = TextAlignment.Center;
                    text.fontSize = waypointLabelFontSize;
                    text.characterSize = 0.015f;
                    text.richText = false;
                    waypointLabels.Add(text);
                }

                TextMesh label = waypointLabels[i];
                label.fontSize = waypointLabelFontSize;
                label.gameObject.SetActive(showWaypointLabels);
                label.transform.position = waypoints[i] + Vector3.up * 0.035f;
                bool isStart = i == 0;
                bool isEnd = i == waypoints.Count - 1;
                label.text = isStart ? "Path Start" : isEnd ? GoalWaypointLabel : $"WP {i}";
                label.color = Color.white;
                if (label.TryGetComponent(out Renderer textRenderer)) textRenderer.sortingOrder = 1;
            }

            for (int i = waypoints.Count; i < waypointLabels.Count; i++)
                waypointLabels[i].gameObject.SetActive(false);
        }

        private void ClearPathVisualization()
        {
            motionProfile = null;
            SmoothedPathCorners = RetainedSharpCorners = 0;
            gizmoPath = Array.Empty<Vector3>();
            gizmoWaypoints = Array.Empty<Vector3>();
            gizmoWaypointRotations = Array.Empty<Quaternion>();
            estimatedTravelTimeSeconds = 0f;
            if (pathRenderer != null) pathRenderer.positionCount = 0;
            SetWaypointLabelsActive(false);
        }

        private void SetWaypointLabelsActive(bool active)
        {
            for (int i = 0; i < waypointLabels.Count; i++)
            {
                if (waypointLabels[i] != null) waypointLabels[i].gameObject.SetActive(active);
            }
        }

        private bool ValidateSetup()
        {
            CorrectSwappedPoseReferences();
            if (rig == null || inverseKinematics == null || !rig.IsReady)
                return Fail("RobotBaseSixAxisRig and RobotBaseSixAxisAnalyticIK must be configured on this robot.");
            if (startPose == null || endTarget == null || obstaclesRoot == null)
                return Fail("Assign Start, End Target, and the Obstacles root in the inspector.");
            return true;
        }

        private void CorrectSwappedPoseReferences()
        {
            if (startPose != null && endTarget != null &&
                startPose.name == "End Target" && endTarget.name == "Start")
            {
                Transform swap = startPose;
                startPose = endTarget;
                endTarget = swap;
            }
        }

        private void CacheColliders()
        {
            if (rig != null && (robotColliders == null || robotColliders.Length == 0))
            {
                robotColliders = rig.GetComponentsInChildren<Collider>(true);
                if (robotColliders.Length == 0 && generateRobotBoxCollidersIfMissing)
                {
                    GenerateMeshBoundBoxes();
                    robotColliders = rig.GetComponentsInChildren<Collider>(true);
                }
            }
            ApplyRobotBoxColliderScale();
            if (obstaclesRoot != null)
                obstacleColliders = obstaclesRoot.GetComponentsInChildren<Collider>(true);
            else obstacleColliders = Array.Empty<Collider>();
        }

        private Transform GetCurrentTcp() => rig.ToolFrame != null ? rig.ToolFrame : rig.Joints[5];

        private void GenerateMeshBoundBoxes()
        {
            MeshFilter[] meshes = rig.GetComponentsInChildren<MeshFilter>(true);
            for (int i = 0; i < meshes.Length; i++)
            {
                MeshFilter meshFilter = meshes[i];
                if (meshFilter.sharedMesh == null || meshFilter.GetComponent<Collider>() != null) continue;
                BoxCollider box = meshFilter.gameObject.AddComponent<BoxCollider>();
                Bounds bounds = meshFilter.sharedMesh.bounds;
                box.center = bounds.center;
                box.size = bounds.size;
            }
        }

        /// <summary>Applies the current box-size scale immediately to the cached robot colliders.</summary>
        public void ApplyRobotBoxColliderScale()
        {
            if (robotColliders == null) return;
            for (int i = 0; i < robotColliders.Length; i++)
            {
                if (!(robotColliders[i] is BoxCollider box)) continue;
                if (!originalRobotBoxSizes.TryGetValue(box, out Vector3 originalSize))
                {
                    originalSize = box.size;
                    originalRobotBoxSizes.Add(box, originalSize);
                }
                box.size = originalSize * robotBoxColliderScale;
            }
            Physics.SyncTransforms();
        }

        private string FinishPlanningTimer()
        {
            if (planningTimer == null) return "unknown time";
            if (planningTimer.IsRunning) planningTimer.Stop();
            planningTimeSeconds = (float)planningTimer.Elapsed.TotalSeconds;
            return planningTimer.Elapsed.TotalSeconds >= 1d
                ? $"{planningTimer.Elapsed.TotalSeconds:F2} s"
                : $"{planningTimer.Elapsed.TotalMilliseconds:F0} ms";
        }

        private bool Fail(string message)
        {
            CurrentMotionState = MotionState.Failed;
            if (planningTimer != null && planningTimer.IsRunning)
                message += $" Planning time: {FinishPlanningTimer()}.";
            if (plannerAlgorithm == PlannerAlgorithm.GridAStar && hasGridFailurePosition)
                message += $" Failure location: {gizmoGridFailurePosition.x:F3}, {gizmoGridFailurePosition.y:F3}, {gizmoGridFailurePosition.z:F3} m.";
            Status = message;
            Debug.LogWarning($"Motion planner: {message}", this);
#if UNITY_EDITOR
            UnityEditor.SceneView.RepaintAll();
#endif
            return false;
        }

        private void ClearGridGizmoData()
        {
            gizmoGridOccupiedCells = Array.Empty<Vector3>();
            gizmoGridRoutePoints = Array.Empty<Vector3>();
            gizmoGridFailurePosition = Vector3.zero;
            gizmoGridCellSize = Mathf.Max(0.01f, gridCellSizeMillimeters * 0.001f);
            hasGridFailurePosition = false;
            hasGridWorkspace = false;
        }

        private void SetGridFailurePosition(Vector3 position)
        {
            gizmoGridFailurePosition = position;
            hasGridFailurePosition = true;
        }

        private void SortByDistance(List<float[]> values, float[] reference)
        {
            if (plannerAlgorithm == PlannerAlgorithm.RRTConnectOptimized)
            {
                Vector2 limit = rig.AngleLimitsDegrees[5];
                for (int i = 0; i < values.Count; i++)
                {
                    float angle = values[i][5];
                    int minimumTurn = Mathf.CeilToInt((limit.x - angle) / 360f);
                    int maximumTurn = Mathf.FloorToInt((limit.y - angle) / 360f);
                    if (minimumTurn > maximumTurn) continue;
                    int turn = Mathf.Clamp(Mathf.RoundToInt((reference[5] - angle) / 360f), minimumTurn, maximumTurn);
                    values[i][5] = angle + turn * 360f;
                }
            }
            values.Sort((a, b) => CompareDistance(a, b, reference));
        }

        private int CompareDistance(float[] a, float[] b, float[] reference) =>
            MotionCost(a, reference).CompareTo(MotionCost(b, reference));

        private static float Distance(float[] a, float[] b)
        {
            float sum = 0f;
            for (int i = 0; i < JointCount; i++) { float d = a[i] - b[i]; sum += d * d; }
            return Mathf.Sqrt(sum);
        }

        private float MotionCost(float[] a, float[] b)
        {
            float sum = 0f;
            for (int i = 0; i < JointCount; i++)
            {
                float delta = a[i] - b[i];
                float weight = i >= 3 ? wristMotionPenalty : 1f;
                sum += weight * delta * delta;
            }
            return Mathf.Sqrt(sum);
        }

        private static float[] Interpolate(float[] a, float[] b, float t)
        {
            var result = new float[JointCount];
            for (int i = 0; i < JointCount; i++) result[i] = Mathf.Lerp(a[i], b[i], t);
            return result;
        }

        private struct PoseCheckReport
        {
            public int candidateCount;
            public int poseClosedCount;
            public int collisionCount;
            public int validCount;
            public float bestPositionError;
            public float bestRotationError;
            public string firstCollisionPair;
        }

        private void OnDrawGizmos()
        {
            if (adaptiveGoalUsed)
            {
                Gizmos.color = new Color(1f, 0.7f, 0.1f);
                Gizmos.DrawLine(requestedGoalPosition, plannedGoalPosition);
                Gizmos.DrawWireSphere(plannedGoalPosition, 0.02f);
            }
            if (showGridOccupancyGizmos && hasGridWorkspace)
            {
                Gizmos.color = new Color(0.2f, 0.75f, 1f, 0.7f);
                Gizmos.DrawWireCube(gizmoGridWorkspace.center, gizmoGridWorkspace.size);
            }
            if (showGridOccupancyGizmos && gizmoGridCellSize > 0f && gizmoGridOccupiedCells != null)
            {
                Gizmos.color = new Color(1f, 0.2f, 0.15f, 0.6f);
                Vector3 cellSize = Vector3.one * gizmoGridCellSize;
                for (int i = 0; i < gizmoGridOccupiedCells.Length; i++)
                    Gizmos.DrawWireCube(gizmoGridOccupiedCells[i], cellSize);
            }
            if (gizmoGridRoutePoints != null && gizmoGridRoutePoints.Length > 1)
            {
                Gizmos.color = new Color(1f, 0.8f, 0.1f, 0.9f);
                for (int i = 1; i < gizmoGridRoutePoints.Length; i++)
                    Gizmos.DrawLine(gizmoGridRoutePoints[i - 1], gizmoGridRoutePoints[i]);
            }
            if (showGridFailureGizmo && hasGridFailurePosition)
            {
                float size = Mathf.Max(0.03f, gizmoGridCellSize);
                Gizmos.color = new Color(1f, 0.05f, 0.05f, 0.22f);
                Gizmos.DrawCube(gizmoGridFailurePosition, Vector3.one * size);
                Gizmos.color = Color.red;
                Gizmos.DrawWireCube(gizmoGridFailurePosition, Vector3.one * size * 1.08f);
                Gizmos.DrawSphere(gizmoGridFailurePosition, Mathf.Max(0.008f, size * 0.08f));
#if UNITY_EDITOR
                UnityEditor.Handles.color = Color.red;
                UnityEditor.Handles.Label(gizmoGridFailurePosition + Vector3.up * size * 0.6f,
                    "Grid failure area");
#endif
            }
#if UNITY_EDITOR
            if (showPathAccuracyGizmos) DrawPathAccuracyGizmos();
#endif
            if (gizmoPath == null || gizmoPath.Length < 2) return;
            Gizmos.color = Color.cyan;
            for (int i = 1; i < gizmoPath.Length; i++)
            {
                Gizmos.DrawLine(gizmoPath[i - 1], gizmoPath[i]);
            }
            for (int i = 0; i < gizmoWaypoints.Length; i++)
            {
                bool isStart = i == 0;
                bool isEnd = i == gizmoWaypoints.Length - 1;
                Gizmos.color = isStart ? Color.green : isEnd ? Color.red : new Color(1f, 0.65f, 0f);
                Gizmos.DrawSphere(gizmoWaypoints[i], isStart || isEnd ? 0.018f : 0.012f);
#if UNITY_EDITOR
                UnityEditor.Handles.color = Color.white;
                UnityEditor.Handles.Label(gizmoWaypoints[i] + Vector3.up * 0.025f,
                    isStart ? "Path Start" : isEnd ? GoalWaypointLabel : $"WP {i}");
#endif
                if (showWaypointTcpRotations && gizmoWaypointRotations != null && i < gizmoWaypointRotations.Length)
                {
                    Quaternion tcpRotation = gizmoWaypointRotations[i];
                    float axisLength = waypointTcpAxisLengthMillimeters * 0.001f;
                    DrawTcpAxis(gizmoWaypoints[i], tcpRotation * Vector3.right, axisLength, Color.red);
                    DrawTcpAxis(gizmoWaypoints[i], tcpRotation * Vector3.up, axisLength, Color.green);
                    DrawTcpAxis(gizmoWaypoints[i], tcpRotation * Vector3.forward, axisLength, Color.blue);
                }
            }
        }

#if UNITY_EDITOR
        private void DrawPathAccuracyGizmos()
        {
            if (gizmoWaypoints != null && gizmoWaypoints.Length > 2)
            {
                var drawn = new HashSet<Vector3>();
                for (int i = 1; i < gizmoWaypoints.Length - 1; i++)
                {
                    Vector3 center = gizmoWaypoints[i];
                    if (!drawn.Add(center)) continue;
                    int before = i - 1, after = i + 1;
                    while (before > 0 && (gizmoWaypoints[before] - center).sqrMagnitude < 0.00000001f) before--;
                    while (after < gizmoWaypoints.Length - 1 && (gizmoWaypoints[after] - center).sqrMagnitude < 0.00000001f) after++;
                    DrawPathAccuracyDimension(gizmoWaypoints[before], center, gizmoWaypoints[after]);
                }
                return;
            }
            // Show assigned targets even before a path is planned.
            if (intermediateWaypoints == null) return;
            var markers = new List<Transform>();
            if (startPose != null) markers.Add(startPose);
            foreach (Transform marker in intermediateWaypoints)
                if (marker != null) markers.Add(marker);
            if (endTarget != null) markers.Add(endTarget);
            for (int i = 1; i < markers.Count - 1; i++)
                DrawPathAccuracyDimension(markers[i - 1].position, markers[i].position, markers[i + 1].position);
        }

        private void DrawPathAccuracyDimension(Vector3 before, Vector3 center, Vector3 after)
        {
            Vector3 incoming = before - center, outgoing = after - center;
            if (incoming.sqrMagnitude < 0.00000001f || outgoing.sqrMagnitude < 0.00000001f) return;
            incoming.Normalize();
            outgoing.Normalize();
            if (Vector3.Dot(incoming, outgoing) < -0.9999f) return;
            Vector3 normal = Vector3.Cross(incoming, outgoing).normalized;
            if (normal.sqrMagnitude < 0.0001f)
                normal = Vector3.Cross(incoming, Mathf.Abs(incoming.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
            Vector3 direction = (incoming + outgoing).normalized;
            if (direction.sqrMagnitude < 0.0001f) direction = Vector3.Cross(normal, incoming).normalized;
            float accuracyMillimeters = Mathf.Max(1f, pathAccuracyMillimeters);
            float radius = accuracyMillimeters * 0.001f;
            Vector3 boundary = center + direction * radius;
            float tickSize = Mathf.Min(radius * 0.15f, UnityEditor.HandleUtility.GetHandleSize(center) * 0.025f);
            Vector3 tickDirection = Vector3.Cross(normal, direction).normalized;
            Vector3 tick = tickDirection * tickSize;
            Color previousColor = UnityEditor.Handles.color;
            UnityEditor.Handles.color = new Color(1f, 0.75f, 0.1f, 0.55f);
            UnityEditor.Handles.DrawWireDisc(center, normal, radius);
            UnityEditor.Handles.color = new Color(1f, 0.85f, 0.2f);
            UnityEditor.Handles.DrawLine(center, boundary);
            UnityEditor.Handles.DrawLine(center - tick, center + tick);
            UnityEditor.Handles.DrawLine(boundary - tick, boundary + tick);
            Vector3 labelOffset = tickDirection * Mathf.Max(tickSize * 2f, UnityEditor.HandleUtility.GetHandleSize(center) * 0.035f);
            UnityEditor.Handles.Label((center + boundary) * 0.5f + labelOffset,
                $"Path accuracy: {accuracyMillimeters:0.##} mm", UnityEditor.EditorStyles.helpBox);
            UnityEditor.Handles.color = previousColor;
        }
#endif

        private static void DrawTcpAxis(Vector3 origin, Vector3 direction, float length, Color color)
        {
            Gizmos.color = color;
            Vector3 end = origin + direction * length;
            Gizmos.DrawLine(origin, end);
            Gizmos.DrawSphere(end, Mathf.Max(0.002f, length * 0.06f));
        }
    }
}
