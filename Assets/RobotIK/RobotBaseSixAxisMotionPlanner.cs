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
    public sealed partial class RobotBaseSixAxisMotionPlanner : MonoBehaviour
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
            InformedRrtStar,
            [InspectorName("Cartesian Linear (MoveL, rounded corners)")]
            CartesianLinear
        }

        private const int JointCount = RobotBaseSixAxisRig.JointCount;

        public enum MotionState { Ready, Planning, Moving, Complete, Failed, Stopped }
        public MotionState CurrentMotionState { get; private set; }
        public bool LastStopWasPlanning { get; private set; }
        public bool IsBusy => endpointReachabilityGateActive || coordinatedSearch != null || stationSearch != null || CurrentMotionState == MotionState.Planning || CurrentMotionState == MotionState.Moving;
        public bool GameplayControlsEnabled => showGameplayControls;
        public Transform StartPose => startPose;
        public Transform EndTarget => endTarget;
        public Transform ObstaclesRoot => obstaclesRoot;
        public IReadOnlyList<Transform> IntermediateWaypoints => intermediateWaypoints;
        /// <summary>Returns a snapshot of the collision geometry used by the planner for diagnostic components.</summary>
        public Collider[] GetRobotCollider()
        {
            if (rig == null) rig = GetComponent<RobotBaseSixAxisRig>();
            CacheColliders();
            return robotColliders == null ? Array.Empty<Collider>() : (Collider[])robotColliders.Clone();
        }
        [SerializeField] private bool checkEndpointIkBeforePlanning = true;
        public bool CheckEndpointIkBeforePlanning { get => checkEndpointIkBeforePlanning; set => checkEndpointIkBeforePlanning = value; }
        public bool IntermediatePositionRetry { get => intermediatePositionRetry; set => intermediatePositionRetry = value; }
        public bool IntermediateRotationRetry { get => intermediateRotationRetry; set => intermediateRotationRetry = value; }
        private bool ExactIntermediateRetryEnabled => plannerAlgorithm == PlannerAlgorithm.CartesianLinear &&
            (intermediatePositionRetry || intermediateRotationRetry);
        public enum EndpointIkState { NotChecked, Valid, Collision, Invalid }
        public EndpointIkState StartIkState { get; private set; }
        public EndpointIkState EndIkState { get; private set; }
        public string StartIkSummary { get; private set; } = "Start: Not checked";
        public string EndIkSummary { get; private set; } = "End: Not checked";

        /// <summary>Checks exact marker poses without planning or leaving the robot moved.</summary>
        public bool CheckStartAndEndIk()
        {
            if (IsBusy || IsRrtSearchInProgress) return false;
            if (!PrepareTrackInspection()) return false;
            return CheckStartAndEndIkCore();
        }

        /// <summary>Displays an endpoint IK pose for inspection, including colliding poses with red highlights.</summary>
        public bool ShowEndpointIkPose(bool showStart)
        {
            if (IsBusy || IsRrtSearchInProgress) return false;
            if (!PrepareTrackInspection()) return false;
            string label = showStart ? "Start" : "End";
            void Store(EndpointIkState state, string text)
            {
                if (showStart) { StartIkState = state; StartIkSummary = text; }
                else { EndIkState = state; EndIkSummary = text; }
            }
            if (!ValidateSetup()) { Store(EndpointIkState.Invalid, label + ": " + Status); return false; }
            CacheColliders();
            if (robotColliders.Length == 0 || obstacleColliders.Length == 0)
            {
                Store(EndpointIkState.Invalid, label + ": Missing collision geometry");
                return false;
            }
            float[] restore = rig.GetJointAnglesDegrees();
            bool continuous = inverseKinematics.SolveContinuously;
            bool applied = false;
            Quaternion defaultRotation = GetCurrentTcp().rotation;
            try
            {
                inverseKinematics.SolveContinuously = false;
                var previewWorld = plannerAlgorithm == PlannerAlgorithm.CartesianLinear ? CreateAlgorithmWorld() : null;
                if (previewWorld != null)
                    previewWorld.IsConfigurationInCollision = angles =>
                    {
                        rig.SetJointAngles(angles); Physics.SyncTransforms();
                        return IsInCollision(out _, includeClearance: false);
                    };
                var evaluator = previewWorld != null ? new CartesianLinearPlanner(previewWorld) : null;
                var check = CheckEndpointIk(showStart ? startPose : endTarget, label, defaultRotation, evaluator, contactOnly: true);
                Store(check.State, check.Text);
                if (check.Angles == null || (check.State != EndpointIkState.Valid && check.State != EndpointIkState.Collision))
                {
                    Status = check.Text;
                    return false;
                }
                if (!rig.SetJointAngles(check.Angles))
                {
                    Store(EndpointIkState.Invalid, label + ": Could not apply IK solution");
                    return false;
                }
                Physics.SyncTransforms();
                applied = true;
                ClearCollisionPreviewHighlights();
                var contacts = new HashSet<Collider>();
                bool collision = IsInCollision(out _, contacts, includeClearance: false);
                foreach (var contact in contacts) HighlightCollisionPart(contact);
                CurrentMotionState = collision ? MotionState.Failed : MotionState.Ready;
                Status = "Showing " + label + " pose. " + check.Text;
                return true;
            }
            finally
            {
                if (!applied)
                {
                    rig.SetJointAngles(restore); Physics.SyncTransforms();
                    inverseKinematics.SolveContinuously = continuous;
                }
                // Leave continuous IK off after a successful preview so it cannot
                // overwrite the selected pose on the next frame.
            }
        }

        private (EndpointIkState State, string Text, float[] Angles) CheckEndpointIk(
            Transform marker, string label, Quaternion defaultRotation, CartesianLinearPlanner evaluator, bool contactOnly = false)
        {
            Quaternion rotation = RequireMarkerOrientations ? marker.rotation : defaultRotation;
            if (!inverseKinematics.SolveAll(marker.position, rotation, out var candidates) || candidates == null)
                return (EndpointIkState.Invalid, label + ": Invalid — no IK solution", null);
            var valid = FilterPoseSolutions(candidates, marker.position, rotation, out var report, includeClearance: !contactOnly);
            string validationFailure = null;
            int accepted = 0;
            float[] selected = null;
            foreach (var angles in valid)
            {
                string failure = evaluator?.GetPoseRejection(angles, marker.position, rotation);
                if (failure == null) { accepted++; if (selected == null) selected = angles; }
                else validationFailure = failure;
            }
            if (accepted > 0) return (EndpointIkState.Valid, $"{label}: Valid — {accepted} collision-free IK solution(s)", selected);
            if (valid.Count > 0) return (EndpointIkState.Invalid, label + ": Invalid — " + validationFailure, null);
            if (report.collisionCount > 0) return (EndpointIkState.Collision, label + (contactOnly ? ": Collision — " : ": Collision / clearance — ") + report.firstCollisionPair, report.firstCollidingAngles);
            return (EndpointIkState.Invalid, label + ": Invalid — " + DescribePoseCheck(marker, report), null);
        }

        private bool CheckStartAndEndIkCore()
        {
            InvalidateEndpointChecks();
            if (!ValidateSetup())
            {
                StartIkState = EndIkState = EndpointIkState.Invalid;
                StartIkSummary = "Start: " + Status; EndIkSummary = "End: " + Status;
                return false;
            }
            CacheColliders();
            if (robotColliders.Length == 0 || obstacleColliders.Length == 0)
            {
                StartIkState = EndIkState = EndpointIkState.Invalid;
                StartIkSummary = "Start: Missing collision geometry"; EndIkSummary = "End: Missing collision geometry";
                return false;
            }
            float[] restore = rig.GetJointAnglesDegrees();
            bool continuous = inverseKinematics.SolveContinuously;
            Quaternion defaultRotation = GetCurrentTcp().rotation;
            try
            {
                inverseKinematics.SolveContinuously = false;
                var evaluator = plannerAlgorithm == PlannerAlgorithm.CartesianLinear ? new CartesianLinearPlanner(CreateAlgorithmWorld()) : null;
                var start = CheckEndpointIk(startPose, "Start", defaultRotation, evaluator);
                StartIkState = start.State; StartIkSummary = start.Text;
                // Each check starts from the same seed, independent of previous IK branches.
                rig.SetJointAngles(restore); Physics.SyncTransforms();
                var end = CheckEndpointIk(endTarget, "End", defaultRotation, evaluator);
                EndIkState = end.State; EndIkSummary = end.Text;
                return StartIkState == EndpointIkState.Valid && EndIkState == EndpointIkState.Valid;
            }
            finally
            {
                rig.SetJointAngles(restore); Physics.SyncTransforms();
                inverseKinematics.SolveContinuously = continuous;
            }
        }

        private void InvalidateEndpointChecks()
        {
            StartIkState = EndIkState = EndpointIkState.NotChecked;
            StartIkSummary = "Start: Not checked"; EndIkSummary = "End: Not checked";
        }

        private bool CheckEndpointsForPlanning()
        {
            bool valid = CheckStartAndEndIkCore();
            // The result always describes the exact End marker. Adaptive Goal may
            // still search for a different valid End pose within its configured limits.
            return valid || (StartIkState == EndpointIkState.Valid && AllowAdaptiveGoal && !ExactIntermediateRetryEnabled);
        }

        /// <summary>Insert an editable pose target after a route marker (or append when none is selected).</summary>
        public Transform AddIntermediateWaypoint(Transform after)
        {
            if (IsBusy || startPose == null || endTarget == null) return null;
            var points = new List<Transform>();
            if (intermediateWaypoints != null)
                foreach (Transform point in intermediateWaypoints) if (point != null) points.Add(point);
            int index = after == startPose ? 0 : points.IndexOf(after) + 1;
            if (after != startPose && !points.Contains(after)) index = points.Count;
            Transform from = index == 0 ? startPose : points[index - 1];
            Transform to = index < points.Count ? points[index] : endTarget;
            var marker = new GameObject("MoveL Waypoint " + (points.Count + 1)).transform;
            // A sibling of Start stays independent of the moving robot hierarchy.
            marker.SetParent(startPose.parent, true);
            marker.SetPositionAndRotation(Vector3.Lerp(from.position, to.position, 0.5f),
                Quaternion.Slerp(from.rotation, to.rotation, 0.5f));
            points.Insert(index, marker);
            intermediateWaypoints = points.ToArray();
            NotifySceneEdited();
            Status = "Waypoint inserted. Move it to the desired clearance position, then reset & plan to validate each MoveL segment.";
            return marker;
        }

        /// <summary>Remove a marker from the route without deleting the scene object.</summary>
        public bool RemoveIntermediateWaypoint(Transform marker)
        {
            if (IsBusy || marker == null || intermediateWaypoints == null) return false;
            var points = new List<Transform>(intermediateWaypoints);
            if (!points.Remove(marker)) return false;
            intermediateWaypoints = points.ToArray();
            NotifySceneEdited();
            return true;
        }

        public bool ReorderIntermediateWaypoint(Transform marker, int offset)
        {
            if (IsBusy || marker == null || intermediateWaypoints == null) return false;
            var points = new List<Transform>(intermediateWaypoints);
            int index = points.IndexOf(marker), next = index + offset;
            if (index < 0 || next < 0 || next >= points.Count) return false;
            points.RemoveAt(index);
            points.Insert(next, marker);
            intermediateWaypoints = points.ToArray();
            NotifySceneEdited();
            return true;
        }
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
            InvalidateFailureVisuals();
            plannedStretchPolicy = null; PeakPlannedStretchPercent = float.NaN; StretchFeedback = string.Empty;
            ClearCollisionPreviewHighlights();
            InvalidateEndpointChecks();
            exactStationPlan = false;
            ClearCoordinatedPlan();
            ClearTrackReachabilityDecision();
            stationSearchSummary = string.Empty; stationSearchLog.Clear();
            path.Clear();
            ClearPathVisualization();
            InvalidateCollisionContext();
            lastPlanSummary = string.Empty;
            hasTrackContext = false;
            plannedTrackState = null;
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
        [Tooltip("Optional locked E1 adapter, attached to this Robot Base. Leave empty for automatic same-object lookup.")]
        [SerializeField] private RobotLinearTrackRig linearTrack;
        public RobotLinearTrackRig LinearTrack => linearTrack != null ? linearTrack : GetComponent<RobotLinearTrackRig>();
        private RobotLinearTrackRig subscribedTrack, lockedTrack;
        private RobotLinearTrackRig.StateStamp plannedTrackState;
        private bool hasTrackContext;
        private Collider[] collisionRobotBodies;
        [SerializeField] private Transform startPose;
        [Tooltip("Ordered TCP targets between Start and End Target. Path Accuracy controls how closely the smoothed TCP path follows each corner.")]
        [SerializeField] private Transform[] intermediateWaypoints = Array.Empty<Transform>();
        [SerializeField] private Transform endTarget;
        [Tooltip("When disabled, A and B are position targets and the tool keeps its current world orientation.")]
        [SerializeField] private bool useMarkerOrientations;
        [Header("Cartesian Linear Routing")]
        [Tooltip("When a direct MoveL segment is blocked, search for intermediate poses connected by validated straight segments. Disable for strict straight-line-only behavior.")]
        [SerializeField] private bool automaticallyFindLinearWaypoints = true;
        [Tooltip("After all Start IK postures fail, try a temporary intermediate pose offset by up to 20 mm. Start and End remain exact.")]
        [SerializeField] private bool intermediatePositionRetry;
        [Tooltip("After all Start IK postures fail, try a temporary intermediate orientation offset by up to 5 degrees. Start and End remain exact.")]
        [SerializeField] private bool intermediateRotationRetry;
        public enum LinearRoutingMode { AutomaticFreeSpace, ForceLiftTraverseDescend }
        [Tooltip("Automatic searches overhead, underneath and beside obstacles. Force mode explicitly restricts the route to rise/traverse/descend.")]
        [SerializeField] private LinearRoutingMode linearRoutingMode = LinearRoutingMode.AutomaticFreeSpace;
        [Tooltip("Optional scanned free-space map for automatic MoveL waypoint search. Leave empty to use geometry-based routing.")]
        [SerializeField] private global::GridOccupancyScanner linearOccupancyScanner;
        [Tooltip("Complete a fresh occupancy scan before every automatic Cartesian Linear planning run. Runtime scans yield between batches to keep the Scene responsive.")]
        [SerializeField] private bool scanLinearOccupancyBeforePlanning = true;
        [SerializeField, Min(1)] private int linearGridMaximumNodes = 50000;
        [Tooltip("Maximum grid corridors to try after MoveL validation failures. Attempts share the existing time, node and edge budgets.")]
        [SerializeField, Min(1)] private int linearGridMaximumRouteAttempts = 12;
        [Tooltip("Minimum rise above the higher endpoint in millimeters. Obstacle height and clearance can require a higher traverse.")]
        [SerializeField, Min(10f)] private float linearMinimumLiftMillimeters = 150f;
        [SerializeField, Range(8, 512)] private int linearWaypointMaximumEdgeChecks = 80;
        [Tooltip("Computation budget per automatic routing attempt. Includes scene-query execution but excludes waiting for frame-budgeted main-thread requests.")]
        [SerializeField, Min(0.1f)] private float linearWaypointSearchSeconds = 15f;
        [Tooltip("Maximum real elapsed seconds per runtime Cartesian segment, including queue waits and the initial direct-line attempt.")]
        [SerializeField, Min(1f)] private float cartesianSegmentWallTimeSeconds = 30f;
        [Tooltip("After finding a valid MoveL route, try shorter low-clearance corridors, redundant-corner removal, and smaller detours. Every replacement is fully validated.")]
        [SerializeField] private bool optimizeLinearPath = true;
        [SerializeField, Min(0f)] private float linearPathOptimizationSeconds = 4f;
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
        [Tooltip("Additional obstacle colliders outside the Obstacles hierarchy. Used by pose checks, planning, clearance and playback validation without reparenting scene objects.")]
        [SerializeField] private Collider[] additionalObstacleColliders = Array.Empty<Collider>();
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
        [SerializeField] private bool showGridOccupancyGizmos = true;
        [SerializeField] private bool showGridFailureGizmo = true;
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
        private readonly Dictionary<ArmTrackStateKey, ArmTrackCollisionCheck> rrtCollisionCache =
            new Dictionary<ArmTrackStateKey, ArmTrackCollisionCheck>();
        private readonly Dictionary<ArmTrackSegmentKey, float> rrtTcpTravelCache =
            new Dictionary<ArmTrackSegmentKey, float>();
        private LineRenderer pathRenderer;
        private RobotMotionControls screenPathOverlayOwner;
        [Header("Path visualization")]
        [SerializeField] private bool pathOverlayVisible = true;
        [SerializeField, Range(1, 12)] private float pathWidthPixels = 2.5f;
        [SerializeField, Range(0, 6)] private float pathOutlinePixels;
        [SerializeField, Range(0, 100)] private float pathTransparencyPercent;
        [SerializeField] private Color pathDisplayColor = new Color(72f / 255, 165f / 255, 1);
        [SerializeField] private bool pathEndpointLabels = true, pathWaypointDots = true;
        [SerializeField] private string pathStartText = "Start", pathEndText = "End";
        public bool PathEndpointLabels => pathEndpointLabels;
        public bool PathWaypointDots => pathWaypointDots;
        public string PathStartText => pathStartText;
        public string PathEndText => pathEndText;
        public void SetPathAnnotations(bool labels, bool dots, string startText, string endText)
        {
            pathEndpointLabels = labels; pathWaypointDots = dots;
            pathStartText = string.IsNullOrWhiteSpace(startText) ? "Start" : startText.Trim();
            pathEndText = string.IsNullOrWhiteSpace(endText) ? "End" : endText.Trim();
            if (pathStartText.Length > 40) pathStartText = pathStartText.Substring(0, 40);
            if (pathEndText.Length > 40) pathEndText = pathEndText.Substring(0, 40);
#if UNITY_EDITOR
            UnityEditor.SceneView.RepaintAll();
#endif
        }
        public bool PathOverlayVisible => pathOverlayVisible;
        public float PathWidthPixels => pathWidthPixels;
        public float PathOutlinePixels => pathOutlinePixels;
        public float PathTransparencyPercent => pathTransparencyPercent;
        public Color PathDisplayColor => pathDisplayColor;
        internal Color PathStrokeColor { get { var color = pathDisplayColor; color.a = 1 - pathTransparencyPercent / 100; return color; } }
        public bool SetPathVisualization(bool visible, float width, float outline, float transparency, Color color)
        {
            foreach (float v in new[] { width, outline, transparency, color.r, color.g, color.b })
                if (float.IsNaN(v) || float.IsInfinity(v)) return false;
            pathOverlayVisible = visible; pathWidthPixels = Mathf.Clamp(width, 1, 12); pathOutlinePixels = Mathf.Clamp(outline, 0, 6);
            pathTransparencyPercent = Mathf.Clamp(transparency, 0, 100); color.a = 1; pathDisplayColor = color;
            if (pathRenderer != null) pathRenderer.enabled = visible && transparency < 100;
#if UNITY_EDITOR
            UnityEditor.SceneView.RepaintAll();
#endif
            return true;
        }
        internal IReadOnlyList<Vector3> DisplayPathPoints => gizmoPath;
        internal IReadOnlyList<Vector3> DisplayWaypointPoints => gizmoWaypoints;
        internal void SetScreenPathOverlay(RobotMotionControls owner, bool active)
        {
            if (active) screenPathOverlayOwner = owner;
            else if (screenPathOverlayOwner == owner) screenPathOverlayOwner = null;
            if (pathRenderer != null) pathRenderer.forceRenderingOff = screenPathOverlayOwner != null;
            foreach (var marker in waypointMarkers) if (marker != null) marker.forceRenderingOff = screenPathOverlayOwner != null;
            foreach (var label in waypointLabels) if (label != null && label.TryGetComponent<Renderer>(out var renderer)) renderer.forceRenderingOff = screenPathOverlayOwner != null;
        }
        private Material pathMaterial;
        private Transform waypointLabelsRoot;
        private readonly List<TextMesh> waypointLabels = new List<TextMesh>();
        private Transform waypointMarkersRoot;
        private readonly List<Renderer> waypointMarkers = new List<Renderer>();
        private readonly List<Vector3> automaticLinearWaypoints = new List<Vector3>();
        public IReadOnlyList<Vector3> AutomaticLinearWaypoints => automaticLinearWaypoints;
        private string lastCollisionPair;
        private Vector3 lastCollisionTcpPosition;
        private bool hasCartesianFailurePosition;
        private Vector3 cartesianFailurePosition;
        private string cartesianFailureLabel;
#if UNITY_EDITOR
        private Vector2 cartesianDiagnosticScroll;
#endif
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
        public float LivePlanningTimeSeconds => CurrentMotionState == MotionState.Planning && planningTimer != null
            ? (float)planningTimer.Elapsed.TotalSeconds : planningTimeSeconds;
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
            activeStateEvaluation?.Dispose();
            endpointReachabilityGateActive = false;
            CancelCoordinatedSearch();
            RollbackStationSearch();
            CancelGridSearch();
            StopAllCoroutines();
            execution = null;
            ReleaseTrackLock();
            CurrentMotionState = MotionState.Ready;
        }

        private void OnDestroy()
        {
            DisposeFailureVisuals();
            if (subscribedTrack != null) subscribedTrack.Changed -= OnTrackChanged;
            ClearCollisionPreviewHighlights();
            if (collisionPreviewMaterial != null)
            {
                if (Application.isPlaying) Destroy(collisionPreviewMaterial); else DestroyImmediate(collisionPreviewMaterial);
            }
            if (pathMaterial == null) return;
            if (Application.isPlaying) Destroy(pathMaterial); else DestroyImmediate(pathMaterial);
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
            if (plannerAlgorithm != value) InvalidateEndpointChecks();
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
                endpointReachabilityGateActive = false;
                CancelCoordinatedSearch();
                RollbackStationSearch();
                ReleaseTrackLock();
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
            RefreshFailureHighlight();
            if (LastPlanReport != null && LastPlanReport.Outcome == PlanReportOutcome.Running && CurrentMotionState == MotionState.Complete)
                CompleteDiagnosticReport(PlanDiagnosticCode.None, Status, PlanReportOutcome.Complete);
            Camera viewCamera = Camera.main;
            for (int i = 0; i < waypointLabels.Count; i++)
            {
                TextMesh label = waypointLabels[i];
                if (label == null) continue;
                bool visible = showWaypointLabels && i < gizmoWaypoints.Length && (i > 0 && i < gizmoWaypoints.Length - 1 || pathEndpointLabels);
                if (label.gameObject.activeSelf != visible) label.gameObject.SetActive(visible);
                label.fontSize = waypointLabelFontSize;
                if (i < gizmoWaypoints.Length) label.transform.position = gizmoWaypoints[i] + Vector3.up * .035f;
                if (i < gizmoWaypoints.Length) label.text = i == 0 ? pathStartText : i == gizmoWaypoints.Length - 1 ? pathEndText : $"WP {i}";
                if (!visible || viewCamera == null) continue;
                Vector3 toCamera = viewCamera.transform.position - label.transform.position;
                if (toCamera.sqrMagnitude > 0.0001f)
                {
                    // Legacy TextMesh renders its front face toward local -Z.
                    label.transform.rotation = Quaternion.LookRotation(-toCamera, viewCamera.transform.up);
                }
            }
            for (int i = 0; i < waypointMarkers.Count; i++)
            {
                var marker = waypointMarkers[i]; if (marker == null) continue;
                bool visible = showWaypointLabels && pathWaypointDots && i < gizmoWaypoints.Length;
                marker.gameObject.SetActive(visible);
                if (visible) marker.transform.position = gizmoWaypoints[i];
            }
        }

        private void Update()
        {
            if (!CheckStretchPolicyGuard()) return;
            if (!CheckTrackReachabilityGuard()) return;
            if (!CheckCoordinatedGuard()) return;
            if (!CheckStationSearchGuard()) return;
            SubscribeTrack();
            if (hasTrackContext && (IsBusy || path.Count > 0) && !TrackContextMatches(out string trackError))
            { AbortTrackChange(trackError); return; }
            if (collisionContext != null && (IsBusy || path.Count > 0) && !collisionContext.Matches(this))
            { AbortTrackChange("Collision geometry, clearance or joint limits changed. Reset & plan again."); return; }
            // RRT and Cartesian searches run on workers, but Physics/Transform queries are
            // serviced here on Unity's main thread. Bound the number of segment sweeps per
            // frame so a large iteration budget does not monopolize the editor/game loop.
            if (rrtMainThreadRequests.IsEmpty) return;
            // Cartesian validation is a sequential request/response pipeline. Keep
            // the existing frame window open while the worker posts the next batch.
            bool cartesian = IsCartesianPlanner;
            PlanningQueryUtilities.Drain(rrtMainThreadRequests, 3.0,
                () => cartesian && IsRrtSearchInProgress);
        }

        private void SubscribeTrack()
        {
            var current = LinearTrack;
            if (current == subscribedTrack) return;
            if (subscribedTrack != null) subscribedTrack.Changed -= OnTrackChanged;
            subscribedTrack = current;
            if (current != null) current.Changed += OnTrackChanged;
        }

        private void OnTrackChanged()
        {
            if (IsBusy) { AbortTrackChange("Track changed during motion. Previous plan was discarded."); return; }
            NotifySceneEdited();
            Status = "Track setup updated. Reset & plan at the selected locked E1.";
        }

        private bool PrepareTrackInspection()
        {
            SubscribeTrack();
            var track = LinearTrack;
            if (track == null || !track.TrackEnabled) return true;
            if (track.transform != transform || !track.TryApplySelected(out string error))
            { Status = "Track setup invalid: " + (track.transform != transform ? "Attach the track rig to this Robot Base." : track.LastError ?? "Could not apply E1."); return false; }
            return ValidateTrackTargets(track);
        }

        private bool ValidateTrackTargets(RobotLinearTrackRig track)
        {
            bool MovingTarget(Transform target) => target != null && target.IsChildOf(track.Carriage);
            if (MovingTarget(startPose) || MovingTarget(endTarget))
                return Fail("Start and End targets must remain outside the moving Carriage hierarchy.");
            foreach (var point in intermediateWaypoints ?? Array.Empty<Transform>())
                if (MovingTarget(point)) return Fail("TCP waypoints must remain outside the moving Carriage hierarchy.");
            return true;
        }

        private bool BeginTrackOperation(bool replay)
        {
            if (!replay && stationSearch == null) { exactStationPlan = false; ClearCoordinatedPlan(); ClearTrackReachabilityDecision(); stationSearchSummary = string.Empty; stationSearchLog.Clear(); }
            SubscribeTrack();
            if (replay && !TrackContextMatches(out string changed)) { AbortTrackChange(changed); return false; }
            var track = LinearTrack;
            if (track != null && track.TrackEnabled)
            {
                if (track.transform != transform) return Fail("Attach the linear track rig to this Robot Base.");
                if (!track.ValidateConfiguration(out string configurationError)) return Fail("Track setup invalid: " + configurationError);
                if (!ValidateTrackTargets(track)) return false;
                if (!track.TryAcquire(this, out var stamp, out string error)) return Fail("Track setup invalid: " + error);
                lockedTrack = track; plannedTrackState = stamp;
            }
            else plannedTrackState = null;
            hasTrackContext = true;
            CacheColliders(); CaptureCollisionContext();
            return true;
        }

        private bool TrackContextMatches(out string error)
        {
            error = null;
            if (!hasTrackContext) return true;
            var track = LinearTrack;
            if (plannedTrackState != null) return plannedTrackState.Matches(track, out error);
            if (track == null || !track.TrackEnabled) return true;
            error = "A track was enabled after this plan was captured. Reset & plan again.";
            return false;
        }

        private void ReleaseTrackLock()
        {
            if (lockedTrack != null) lockedTrack.Release(this);
            lockedTrack = null;
        }

        private void AbortTrackChange(string error)
        {
            StopMotion();
            NotifySceneEdited();
            Status = error ?? "Track changed. Reset & plan again.";
            CurrentMotionState = MotionState.Stopped;
            CompleteDiagnosticReport(PlanDiagnosticCode.ConfigurationChanged, Status, PlanReportOutcome.Stopped);
        }

        private void OnValidate()
        {
            InvalidateEndpointChecks();
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
            stationSpacingMillimeters = LinearTrackAxis.Finite(stationSpacingMillimeters) ? Mathf.Max(1, stationSpacingMillimeters) : 250;
            maximumStationCandidates = Mathf.Clamp(maximumStationCandidates, 3, 128);
            maximumStationRouteAttempts = Mathf.Clamp(maximumStationRouteAttempts, 1, 128);
            stationSearchSeconds = LinearTrackAxis.Finite(stationSearchSeconds) ? Mathf.Max(.1f, stationSearchSeconds) : 30;
            coordinatedCartesianStepMillimeters = LinearTrackAxis.Finite(coordinatedCartesianStepMillimeters) ? Mathf.Max(1, coordinatedCartesianStepMillimeters) : 20;
            coordinatedTrackStepMillimeters = LinearTrackAxis.Finite(coordinatedTrackStepMillimeters) ? Mathf.Max(1, coordinatedTrackStepMillimeters) : 40;
            coordinatedBeamWidth = Mathf.Clamp(coordinatedBeamWidth, 2, 64);
            coordinatedSearchSeconds = LinearTrackAxis.Finite(coordinatedSearchSeconds) ? Mathf.Max(.1f, coordinatedSearchSeconds) : 30;
            coordinatedTrackSpeedMillimetersPerSecond = LinearTrackAxis.Finite(coordinatedTrackSpeedMillimetersPerSecond) ? Mathf.Max(1, coordinatedTrackSpeedMillimetersPerSecond) : 250;
            transferStationSpacingMillimeters = LinearTrackAxis.Finite(transferStationSpacingMillimeters) ? Mathf.Max(1, transferStationSpacingMillimeters) : 250;
            transferMaximumStations = Mathf.Clamp(transferMaximumStations, 3, 65);
            transferApproachMillimeters = LinearTrackAxis.Finite(transferApproachMillimeters) ? Mathf.Max(10, transferApproachMillimeters) : 100;
            transferPostureIterations = Mathf.Clamp(transferPostureIterations, 100, 10000);
            transferTargetWaypoints = Mathf.Clamp(transferTargetWaypoints, 2, 10000);
            maximumStretchPercent = LinearTrackAxis.Finite(maximumStretchPercent) ? Mathf.Clamp(maximumStretchPercent, 50, 100) : 84;
            transferCorridorDropMm = LinearTrackAxis.Finite(transferCorridorDropMm) ? Mathf.Clamp(transferCorridorDropMm, 10, 2000) : 200;
            if (requireLowerTransferCorridor) preferLowerTransferCorridor = true;
            transferReductionPositionMm = LinearTrackAxis.Finite(transferReductionPositionMm) ? Mathf.Max(.01f, transferReductionPositionMm) : 1;
            transferReductionRotationDegrees = LinearTrackAxis.Finite(transferReductionRotationDegrees) ? Mathf.Max(.001f, transferReductionRotationDegrees) : .1f;
            transferOptimizationBudgetSeconds = LinearTrackAxis.Finite(transferOptimizationBudgetSeconds) ? Mathf.Clamp(transferOptimizationBudgetSeconds, .1f, 30) : 2;
            transferTrackAcceleration = LinearTrackAxis.Finite(transferTrackAcceleration) ? Mathf.Max(1, transferTrackAcceleration) : 1000;
        }

        [ContextMenu("Plan Collision-Free Motion Start to End")]
        public void PlanAndMove()
        {
            if (!Application.isPlaying) { Status = "Enter Play mode before planning and moving."; return; }
            CancelGridSearch();
            StopAllCoroutines();
            ReleaseTrackLock();
            BeginDiagnosticReport();
            if (!BeginTrackOperation(false)) { CompleteDiagnosticReport(PlanDiagnosticCode.InvalidSetup, Status); return; }
            CurrentMotionState = MotionState.Planning;
            execution = StartCoroutine(RunMotionOperation(PlanAndMoveRoutine()));
        }

        private void CancelGridSearch()
        {
            ClearCollisionPreviewHighlights();
            System.Threading.Interlocked.Exchange(ref rrtSearchInProgress, 0);
            if (gridSearchCancellation == null) return;
            gridSearchCancellation.Cancel();
            gridSearchCancellation.Dispose();
            gridSearchCancellation = null;
        }

        /// <summary>Replays the stored plan from its first joint state without replanning.</summary>
        public void ReplayPlannedMotion()
        {
            ClearFailureVisuals();
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
            if (!TrackContextMatches(out string trackError)) { AbortTrackChange(trackError); return; }
            CancelGridSearch();
            StopAllCoroutines();
            ReleaseTrackLock();
            if (!BeginTrackOperation(true)) return;
            CurrentMotionState = MotionState.Moving;
            execution = StartCoroutine(RunMotionOperation(ReplayPlannedMotionRoutine()));
        }

        /// <summary>Updates the curve and timing after Inspector edits while the robot is idle.</summary>
        public void RefreshMotionPreview()
        {
            if (!TrackContextMatches(out string trackError)) { AbortTrackChange(trackError); return; }
            if (!IsBusy && !IsRrtSearchInProgress && path.Count > 1)
                DrawPlannedPath();
        }

        private IEnumerator ReplayPlannedMotionRoutine()
        {
            if (coordinatedPath.Count > 1) { yield return StartCoroutine(ExecuteCoordinatedPath()); yield break; }
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
            ReleaseTrackLock();
            BeginDiagnosticReport();
            if (!BeginTrackOperation(false)) { CompleteDiagnosticReport(PlanDiagnosticCode.InvalidSetup, Status); return; }
            CurrentMotionState = MotionState.Planning;
            exactStationPlan = false;
            execution = StartCoroutine(RunMotionOperation(TrackReachabilityGateApplicable ? ResetWithTrackReachabilityRoutine() : ResetToStartAndPlanRoutine()));
        }

        private IEnumerator ResetToStartAndPlanRoutine()
        {
            if (inverseKinematics != null) inverseKinematics.SolveContinuously = false;
            if (!ValidateSetup()) { execution = null; yield break; }
            CacheColliders();
            if (robotColliders.Length == 0 || obstacleColliders.Length == 0)
            {
                Fail("Collision geometry is missing. Assign robot colliders and colliders beneath Obstacles.", PlanDiagnosticCode.InvalidSetup);
                execution = null;
                yield break;
            }

            float[] originalAngles = rig.GetJointAnglesDegrees();
            if (checkEndpointIkBeforePlanning && !CheckEndpointsForPlanning())
            {
                Fail("Endpoint IK check failed. " + StartIkSummary + ". " + EndIkSummary, PlanDiagnosticCode.EndpointValidation);
                execution = null; yield break;
            }
            Quaternion startRotation = RequireMarkerOrientations ? startPose.rotation : GetCurrentTcp().rotation;
            if (!inverseKinematics.SolveAll(startPose.position, startRotation, out List<float[]> candidates))
            {
                Fail("Start pose is unreachable by the analytic IK.", PlanDiagnosticCode.IkNoSolution);
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

            // Reset selects a posture before motion, never a branch-changing MoveL edge.
            if (!rig.SetJointAngles(validStarts[0]))
            {
                Fail("Could not apply the selected Start IK solution.");
                execution = null;
                yield break;
            }
            Physics.SyncTransforms();
            yield return null;
            yield return StartCoroutine(PlanAndMoveRoutine(forcePlayAfterPlan: true,
                endpointsAlreadyChecked: checkEndpointIkBeforePlanning,
                resetStartCandidates: IsCartesianPlanner ? validStarts : null));
        }

        private IEnumerator PlanAndMoveRoutine(bool forcePlayAfterPlan = false, bool endpointsAlreadyChecked = false,
            List<float[]> resetStartCandidates = null, bool suppressExecution = false)
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
                Fail("Collision geometry is missing. Assign robot colliders and colliders beneath Obstacles.", PlanDiagnosticCode.InvalidSetup);
                execution = null;
                yield break;
            }

            Status = "Planning from current robot pose to Start, then to End Target...";
            if (checkEndpointIkBeforePlanning && !endpointsAlreadyChecked && !CheckEndpointsForPlanning())
            {
                Fail("Endpoint IK check failed. " + StartIkSummary + ". " + EndIkSummary, PlanDiagnosticCode.EndpointValidation);
                execution = null; yield break;
            }
            if (ShouldScanLinearOccupancy)
            {
                Status = "Scanning occupancy before Cartesian Linear planning...";
                var scan = linearOccupancyScanner.ScanOverFrames(gridSearchCancellation.Token);
                try
                {
                    while (scan.MoveNext())
                    {
                        Status = $"Scanning occupancy before Cartesian Linear planning: {linearOccupancyScanner.ScanProgress:P0}";
                        yield return scan.Current;
                    }
                }
                finally { (scan as IDisposable)?.Dispose(); }
                if (!linearOccupancyScanner.TryGetPlanningSnapshot(out _, out string scanError))
                { Fail("Occupancy scan failed: " + scanError); execution = null; yield break; }
                Status = "Occupancy scan complete. Planning Cartesian Linear motion...";
                yield return null;
            }
            float[] originalAngles = rig.GetJointAnglesDegrees();
            Quaternion preferredRotation = GetCurrentTcp().rotation;
            Quaternion startRotation = RequireMarkerOrientations ? startPose.rotation : preferredRotation;
            Quaternion goalRotation = RequireMarkerOrientations ? endTarget.rotation : preferredRotation;
            InitializeGoalSelection(goalRotation);
            if (!inverseKinematics.SolveAll(startPose.position, startRotation, out List<float[]> starts))
            {
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                Fail("Start pose is unreachable by the analytic IK.", PlanDiagnosticCode.IkNoSolution);
                execution = null;
                yield break;
            }

            List<float[]> validStarts = FilterPoseSolutions(starts, startPose, startRotation, out PoseCheckReport startReport);
            rig.SetJointAngles(originalAngles);
            Physics.SyncTransforms();
            List<float[]> validGoals = SolveGoalPose(plannedGoalPosition, plannedGoalRotation, false, out PoseCheckReport goalReport);
            if (validStarts.Count > 0 && validGoals.Count == 0 && AllowAdaptiveGoal && !ExactIntermediateRetryEnabled)
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
            if (resetStartCandidates == null && IsInCollision())
            {
                Fail("The robot's current pose collides with an obstacle, so a collision-free move cannot start.");
                execution = null;
                yield break;
            }

            if (IsCartesianPlanner)
            {
                bool cartesianFound = false;
                List<float[]> bestCartesianPath = null;
                bool resetAtStart = resetStartCandidates != null;
                if (resetAtStart) validStarts = new List<float[]>(resetStartCandidates);
                SortByDistance(validStarts, originalAngles);
                validGoals.Sort((a, b) => CompareDistance(a, b, originalAngles));
                Transform tcp = GetCurrentTcp();
                Vector3 currentPosition = tcp.position;
                Quaternion currentRotation = tcp.rotation;
                int startAttempts = 0;
                int selectedStart = 0;
                bool usedPoseRetry = false;
                int passes = ExactIntermediateRetryEnabled ? 2 : 1;
                for (int pass = 0; pass < passes && !cartesianFound; pass++)
                {
                bool poseRetry = pass == 1;
                for (int s = 0; s < validStarts.Count && !cartesianFound &&
                    (resetAtStart || plannerAlgorithm != PlannerAlgorithm.CartesianLinear || s == 0); s++)
                {
                    startAttempts++;
                    Status = resetAtStart
                        ? $"{(poseRetry ? "Trying intermediate pose" : "Planning")} from Start IK posture {s + 1}/{validStarts.Count}; wrist jumps remain forbidden..."
                        : "Planning Cartesian motion...";
                    automaticLinearWaypoints.Clear();
                    rig.SetJointAngles(originalAngles);
                    Physics.SyncTransforms();
                    var toStart = new List<float[]>();
                    bool reachedStart = false;
                    if (resetAtStart)
                    {
                        // Each trial starts independently at Start. Do not connect two
                        // analytical solutions of the same pose with an artificial edge.
                        toStart.Add((float[])validStarts[s].Clone());
                        reachedStart = true;
                    }
                    else
                        yield return StartCoroutine(TryBuildCartesianSegmentOverFrames(originalAngles, currentPosition, currentRotation,
                            validStarts[s], startPose.position, startRotation, toStart, ok => reachedStart = ok,
                            isApproachToStart: true));
                    if (!reachedStart)
                    {
                        lastCartesianFailure = $"Current TCP to Start: {lastCartesianFailure}";
                        continue;
                    }
                    var tailPoses = new List<Transform>(viaPoses) { endTarget };
                    var tailSolutions = new List<List<float[]>>(viaSolutions) { validGoals };
                    bestCartesianPath = new List<float[]>(toStart);
                    yield return StartCoroutine(TryBuildCartesianTailOverFrames(toStart[toStart.Count - 1], startPose.position, startRotation,
                        tailPoses, tailSolutions, 0, bestCartesianPath, preferredRotation, ok => cartesianFound = ok, poseRetry));
                    if (cartesianFound) { selectedStart = s + 1; usedPoseRetry = poseRetry; }
                    if (!cartesianFound) lastCartesianFailure = $"Start through intermediate waypoint(s) to End Target: {lastCartesianFailure}";
                }
                }
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                if (!cartesianFound)
                {
                    Fail($"{GetPlannerAlgorithmLabel()} failed. {lastCartesianFailure}" +
                        (resetAtStart ? $" Tried {startAttempts} Start posture attempts across {validStarts.Count} valid IK postures{(passes > 1 ? ", including enabled intermediate pose adjustments" : string.Empty)}. No wrist-branch jump was permitted; no partial path will execute." : string.Empty));
                    execution = null;
                    yield break;
                }
                hasCartesianFailurePosition = false;
                path.AddRange(bestCartesianPath);
                // Keep the Cartesian samples: joint-space shortcutting would pull the
                // TCP away from the selected Cartesian arc even if it remains collision-free.
                SetPlanSummary($"Plan found with {GetPlannerAlgorithmLabel()} after {cartesianArcAttempts} Cartesian route attempts in {FinishPlanningTimer()} ({path.Count} waypoints)." +
                    (resetAtStart ? $" Selected Start IK posture {selectedStart}/{validStarts.Count} before motion; no wrist-branch jumps." : string.Empty) +
                    (usedPoseRetry ? " Used an automatic intermediate pose adjustment; Start and End remain exact." : string.Empty));
                Status = lastPlanSummary;
                Debug.Log($"Motion planner: {Status}", this);
                DrawPlannedPath();
                if (!suppressExecution && (playOnPlan || forcePlayAfterPlan))
                {
                    if (resetAtStart)
                    {
                        rig.SetJointAngles(path[0]);
                        Physics.SyncTransforms();
                        yield return null;
                    }
                    yield return StartCoroutine(ExecutePath());
                }
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

        // Share the RRT worker/main-thread bridge. All captured scene delegates below
        // execute only inside InvokeRrtMainThread; worker code sees copied value data.
        private RobotBaseSixAxisPlannerAlgorithm.World CreateCartesianWorkerWorld(float[] restoreAngles,
            CancellationToken token, Func<bool> optimizationShouldStop)
        {
            var searchReport = LastPlanReport;
            var sceneWorld = CreateAlgorithmWorld();
            var evaluator = new CartesianLinearPlanner(sceneWorld);
            bool checkSingularity = plannerAlgorithm == PlannerAlgorithm.CartesianLinear;
            float wristPenalty = wristMotionPenalty;
            float occupancyInflation = gridObstacleInflationMillimeters * 0.001f;
            var obstacles = (Bounds[])sceneWorld.LinearRoutingObstacles.Clone();
            var cache = new Dictionary<ArmTrackStateKey, RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck>();
            long queryContext = collisionContextRevision;
            float? queryE1 = plannedTrackState != null ? plannedTrackState.E1Millimeters : (float?)null;
            float[] ikSeed = (float[])restoreAngles.Clone();
            RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck lastCheck = null;
            ArmTrackStateKey Key(float[] q) => new ArmTrackStateKey(new ArmTrackState(q, queryE1), queryContext);
            var computationClock = new PlanningComputationClock();

            T OnMain<T>(Func<T> query)
            {
                computationClock.SuspendForQuery();
                long executionTicks = 0;
                try
                {
                    var answer = InvokeRrtMainThread(() =>
                    {
                        AssertCollisionContext();
                        var queryTimer = System.Diagnostics.Stopwatch.StartNew();
                        T value = query();
                        return (value, queryTimer.ElapsedTicks);
                    }, token);
                    executionTicks = answer.Item2;
                    return answer.Item1;
                }
                finally { computationClock.ResumeAfterQuery(executionTicks); }
            }

            RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck QueryScene(float[] q)
            {
                token.ThrowIfCancellationRequested();
                if (LastPlanReport?.Outcome == PlanReportOutcome.Running) LastPlanReport.ConfigurationQueries++;
                using var evaluation = new StateEvaluation(this, new ArmTrackState(q, queryE1));
                Transform tcp = GetCurrentTcp();
                var check = new RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck
                {
                    Position = tcp.position, Rotation = tcp.rotation
                };
                check.Collision = IsInCollision(out string pair);
                check.CollisionDescription = pair;
                if (!check.Collision && checkSingularity) check.SingularityFailure = evaluator.GetSingularityRejection(q);
                if (check.SingularityFailure != null) CaptureDiagnostic(PlanDiagnosticCode.Singularity, check.SingularityFailure, q, new Pose(check.Position, check.Rotation),
                    (float)evaluator.LastSingularityRatio, (float)evaluator.SingularityThreshold, "Jacobian min/max singular-value ratio");
                if (!evaluation.IsActive) throw new OperationCanceledException("Cartesian state evaluation was stopped.");
                System.Threading.Interlocked.Increment(ref currentRrtSearchIterations);
                return check;
            }

            RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck[] CheckBatch(IReadOnlyList<float[]> states)
            {
                token.ThrowIfCancellationRequested();
                var checks = new RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck[states.Count];
                var missing = new List<int>();
                for (int i = 0; i < states.Count; i++)
                    if (cache.TryGetValue(Key(states[i]), out var check))
                    {
                        checks[i] = check;
                        if (searchReport != null && searchReport.Outcome == PlanReportOutcome.Running)
                        { System.Threading.Interlocked.Increment(ref searchReport.CacheHits); System.Threading.Interlocked.Increment(ref searchReport.ConfigurationQueries); }
                    }
                    else missing.Add(i);
                // A query has a small bounded number of FK/Jacobian/physics checks.
                // Update pumps these requests for the same 3 ms/frame budget as RRT.
                for (int offset = 0; offset < missing.Count; offset += 3)
                {
                    int first = offset, count = Math.Min(3, missing.Count - offset);
                    var answers = OnMain(() =>
                    {
                        try
                        {
                            var batch = new RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck[count];
                            for (int j = 0; j < count; j++) batch[j] = QueryScene(states[missing[first + j]]);
                            return batch;
                        }
                        finally { rig.SetJointAngles(restoreAngles); Physics.SyncTransforms(); }
                    });
                    for (int j = 0; j < count; j++)
                    {
                        int index = missing[first + j];
                        checks[index] = answers[j]; cache[Key(states[index])] = answers[j];
                    }
                }
                if (checks.Length > 0) lastCheck = checks[checks.Length - 1];
                return checks;
            }
            RobotBaseSixAxisPlannerAlgorithm.ConfigurationCheck Check(float[] q) => CheckBatch(new[] { q })[0];

            var world = new RobotBaseSixAxisPlannerAlgorithm.World
            {
                JointLimits = (Vector2[])sceneWorld.JointLimits.Clone(),
                ObstacleBounds = (Vector3[])sceneWorld.ObstacleBounds.Clone(),
                Workspace = sceneWorld.Workspace,
                LinearRoutingObstacles = obstacles,
                Clearance = sceneWorld.Clearance,
                CollisionResolutionDegrees = sceneWorld.CollisionResolutionDegrees,
                ArcStepMeters = sceneWorld.ArcStepMeters,
                LinearAutomaticWaypoints = sceneWorld.LinearAutomaticWaypoints,
                LinearForceLiftTraverseDescend = sceneWorld.LinearForceLiftTraverseDescend,
                LinearExploreSpatialCorridors = sceneWorld.LinearExploreSpatialCorridors,
                LinearOccupancyGrid = sceneWorld.LinearOccupancyGrid,
                LinearOccupancyError = sceneWorld.LinearOccupancyError,
                LinearGridMaximumNodes = sceneWorld.LinearGridMaximumNodes,
                LinearGridMaximumRouteAttempts = sceneWorld.LinearGridMaximumRouteAttempts,
                LinearMinimumLiftMeters = sceneWorld.LinearMinimumLiftMeters,
                LinearAllowAlternateGoalConfiguration = sceneWorld.LinearAllowAlternateGoalConfiguration,
                LinearWaypointMaximumEdges = sceneWorld.LinearWaypointMaximumEdges,
                LinearWaypointSearchSeconds = sceneWorld.LinearWaypointSearchSeconds,
                LinearOptimizePath = sceneWorld.LinearOptimizePath,
                LinearOptimizationSeconds = sceneWorld.LinearOptimizationSeconds,
                OptimizationShouldStop = optimizationShouldStop,
                LinearCharacteristicLengthMeters = sceneWorld.LinearCharacteristicLengthMeters,
                LinearMinimumSingularValueRatio = sceneWorld.LinearMinimumSingularValueRatio,
                CancellationToken = token,
                PlanningElapsedSeconds = () => computationClock.ElapsedSeconds,
                EvaluateConfiguration = Check,
                EvaluateConfigurationBatch = CheckBatch,
                GetTcpPosition = q => { ikSeed = (float[])q.Clone(); return Check(q).Position; },
                GetTcpRotation = q => Check(q).Rotation,
                IsConfigurationInCollision = q => Check(q).Collision,
                GetLastCollisionDescription = () => lastCheck?.CollisionDescription,
                GetLastCollisionTcpPosition = () => lastCheck == null ? Vector3.zero : lastCheck.Position,
                SolveIk = (position, rotation) =>
                {
                    float[] seed = (float[])ikSeed.Clone();
                    var solutions = OnMain(() =>
                    {
                        try
                        {
                            rig.SetJointAngles(seed); Physics.SyncTransforms();
                            return sceneWorld.SolveIk(position, rotation);
                        }
                        finally { rig.SetJointAngles(restoreAngles); Physics.SyncTransforms(); }
                    });
                    // The planner filters unreachable branches before batching checks.
                    return solutions;
                },
                MotionCost = (a, b) => ComputeWorkerMotionCost(a, b, wristPenalty),
                IsCartesianPointBlocked = (position, clearance) =>
                {
                    foreach (Bounds obstacle in obstacles)
                    {
                        Bounds expanded = obstacle;
                        expanded.Expand(2f * Math.Max(clearance, occupancyInflation));
                        if (expanded.Contains(position)) return true;
                    }
                    return false;
                },
                IsJointSegmentFree = (from, to) =>
                {
                    double distance = 0;
                    for (int j = 0; j < from.Length; j++) distance += (to[j] - from[j]) * (to[j] - from[j]);
                    int count = Math.Max(1, (int)Math.Ceiling(Math.Sqrt(distance) / sceneWorld.CollisionResolutionDegrees));
                    for (int i = 0; i <= count; i++)
                    {
                        var state = new float[from.Length];
                        for (int j = 0; j < state.Length; j++) state[j] = from[j] + (to[j] - from[j]) * (i / (float)count);
                        if (Check(state).Collision) return false;
                    }
                    return true;
                }
            };
            return world;
        }

        private IEnumerator TryBuildCartesianSegmentOverFrames(float[] startAngles, Vector3 startPosition,
            Quaternion startRotation, float[] endAngles, Vector3 endPosition, Quaternion endRotation,
            List<float[]> result, Action<bool> completed, string segmentLabel = "Current TCP -> Start",
            bool isApproachToStart = false, bool poseRetry = false)
        {
            result.Clear();
            Status = $"{segmentLabel}: searching with {GetPlannerAlgorithmLabel()} on a worker thread...";
            CancellationToken parentToken = gridSearchCancellation != null ? gridSearchCancellation.Token : CancellationToken.None;
            using var deadline = CancellationTokenSource.CreateLinkedTokenSource(parentToken);
            float wallLimit = Mathf.Max(1f, cartesianSegmentWallTimeSeconds);
            deadline.CancelAfter(TimeSpan.FromSeconds(wallLimit));
            CancellationToken token = deadline.Token;
            var segmentTimer = System.Diagnostics.Stopwatch.StartNew();
            float[] restoreAngles = rig.GetJointAnglesDegrees();
            var workerWorld = CreateCartesianWorkerWorld(restoreAngles, token,
                () => segmentTimer.Elapsed.TotalSeconds >= Math.Max(0, wallLimit - 1f));
            workerWorld.LinearIntermediatePositionRetry = poseRetry && intermediatePositionRetry;
            workerWorld.LinearIntermediateRotationRetry = poseRetry && intermediateRotationRetry;
            // Positioning at Start must not introduce a forced lift and descent
            // before the authored route begins. Necessary collision detours remain enabled.
            if (isApproachToStart) workerWorld.LinearForceLiftTraverseDescend = false;
            var kind = GetAlgorithmKind();
            System.Threading.Interlocked.Exchange(ref currentRrtSearchIterations, 0);
            System.Threading.Interlocked.Exchange(ref rrtSearchInProgress, 1);
            var search = Task.Run(() =>
            {
                try
                {
                    bool found = new RobotBaseSixAxisPlannerAlgorithm(workerWorld).TryPlan(kind,
                        startAngles, endAngles, startPosition, startRotation, endPosition, endRotation, out var plan);
                    return new RrtWorkerOutcome { Found = found, Result = plan };
                }
                catch (OperationCanceledException) { return new RrtWorkerOutcome { Cancelled = true }; }
                catch (Exception error) { return new RrtWorkerOutcome { Error = error }; }
            });
            try
            {
                while (!search.IsCompleted)
                {
                    if (token.IsCancellationRequested)
                    {
                        if (!parentToken.IsCancellationRequested)
                            lastCartesianFailure = $"Segment [{segmentLabel}] stopped at the {wallLimit:F1} s wall-time limit ({CurrentRrtSearchIterations} configuration checks). No partial path will execute.";
                        completed(false); yield break;
                    }
                    yield return null;
                }
                var outcome = search.GetAwaiter().GetResult();
                if (outcome.Error != null)
                {
                    Debug.LogException(outcome.Error, this);
                    lastCartesianFailure = $"Segment [{segmentLabel}]: {outcome.Error.Message}";
                    completed(false); yield break;
                }
                if (outcome.Cancelled || outcome.Result == null)
                {
                    if (token.IsCancellationRequested && !parentToken.IsCancellationRequested)
                        lastCartesianFailure = $"Segment [{segmentLabel}] stopped at the {wallLimit:F1} s wall-time limit ({CurrentRrtSearchIterations} configuration checks). No partial path will execute.";
                    completed(false); yield break;
                }
                var plan = outcome.Result;
                if (!string.IsNullOrEmpty(plan.RoutingDiagnostics))
                    Debug.Log($"MoveL routing diagnostics [{segmentLabel}] (mode: {linearRoutingMode}):\n{plan.RoutingDiagnostics}", this);
                cartesianArcAttempts += plan.RoutesTried;
                planningIterations += plan.Iterations;
                if (!outcome.Found)
                {
                    hasCartesianFailurePosition = plan.HasFailurePosition;
                    cartesianFailurePosition = plan.FailurePosition;
                    cartesianFailureLabel = segmentLabel + "\n" + plan.FailureReason;
                    lastCartesianFailure = $"Segment [{segmentLabel}]. Tried {plan.RoutesTried} Cartesian route(s). {plan.FailureReason} Clearance is {minimumObstacleClearance * 1000f:F0} mm.";
                    completed(false); yield break;
                }
                automaticLinearWaypoints.AddRange(plan.AutomaticWaypoints);
                result.AddRange(plan.JointPath);
                completed(result.Count > 1);
            }
            finally
            {
                System.Threading.Interlocked.Exchange(ref rrtSearchInProgress, 0);
                rig.SetJointAngles(restoreAngles); Physics.SyncTransforms();
            }
        }

        private IEnumerator TryBuildCartesianTailOverFrames(float[] fromAngles, Vector3 fromPosition, Quaternion fromRotation,
            List<Transform> poses, List<List<float[]>> poseSolutions, int stageIndex,
            List<float[]> accumulatedPath, Quaternion defaultRotation, Action<bool> completed, bool poseRetry = false)
        {
            if (stageIndex >= poses.Count) { completed(true); yield break; }
            Transform pose = poses[stageIndex];
            var candidates = poseSolutions[stageIndex];
            SortByDistance(candidates, fromAngles);
            int prefixCount = accumulatedPath.Count, automaticPrefix = automaticLinearWaypoints.Count;
            GetPlanningPose(pose, defaultRotation, out Vector3 position, out Quaternion rotation);
            string label = (stageIndex == 0 ? $"Start ({startPose.name})" : $"WP{stageIndex} ({poses[stageIndex - 1].name})") +
                " -> " + (stageIndex == poses.Count - 1 ? $"End Target ({pose.name})" : $"WP{stageIndex + 1} ({pose.name})");
            for (int i = 0; i < candidates.Count && (plannerAlgorithm != PlannerAlgorithm.CartesianLinear || i == 0); i++)
            {
                var segment = new List<float[]>();
                bool found = false;
                yield return StartCoroutine(TryBuildCartesianSegmentOverFrames(fromAngles, fromPosition, fromRotation,
                    candidates[i], position, rotation, segment, ok => found = ok, label, poseRetry: poseRetry));
                if (!found) continue;
                for (int j = 1; j < segment.Count; j++) accumulatedPath.Add(segment[j]);
                bool tailFound = false;
                yield return StartCoroutine(TryBuildCartesianTailOverFrames(segment[segment.Count - 1], position, rotation,
                    poses, poseSolutions, stageIndex + 1, accumulatedPath, defaultRotation, ok => tailFound = ok, poseRetry));
                if (tailFound) { completed(true); yield break; }
                accumulatedPath.RemoveRange(prefixCount, accumulatedPath.Count - prefixCount);
                automaticLinearWaypoints.RemoveRange(automaticPrefix, automaticLinearWaypoints.Count - automaticPrefix);
            }
            completed(false);
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
                // Wake as soon as Cancel is requested, without a polling interval
                // or waiting for the next Unity Update to service this query.
                completed.Wait(cancellationToken);
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
            var sweptSegmentCache = new Dictionary<ArmTrackSegmentKey, bool>();
            long queryContext = collisionContextRevision;
            float? queryE1 = plannedTrackState != null ? plannedTrackState.E1Millimeters : (float?)null;
            float copiedWristPenalty = wristMotionPenalty;
            var copiedReachPolicy = ReachPolicy; var copiedReachGeometry = stretchGeometry;
            float WorkerCost(float[] a, float[] b)
            {
                float cost = ComputeWorkerMotionCost(a, b, copiedWristPenalty);
                return cost * (1 + .5f * (copiedReachPolicy.Penalty(copiedReachGeometry.Measure(a), false) + copiedReachPolicy.Penalty(copiedReachGeometry.Measure(b), false)));
            }
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
                MotionCost = WorkerCost,
                RouteCost = WorkerCost,
                TcpTravelCost = (a, b) => InvokeRrtMainThread(() => GetCachedRrtTcpTravelDistance(a, b), cancellationToken),
                TcpPathCost = states => InvokeRrtMainThread(() => GetTcpPathTravelDistance(states), cancellationToken),
                IsJointSegmentFree = (a, b) =>
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var key = new ArmTrackSegmentKey(new ArmTrackState(a, queryE1), new ArmTrackState(b, queryE1), queryContext);
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
            var check = EvaluateCombinedState(AtPlanningStation(angles));
            if (check.InCollision) { lastCollisionPair = check.Description; lastCollisionTcpPosition = check.TcpPose.position; }
            return check.CollisionFree;
        }

        private bool IsCartesianPlanner => plannerAlgorithm == PlannerAlgorithm.CartesianArc || plannerAlgorithm == PlannerAlgorithm.CartesianLinear;

        private RobotBaseSixAxisPlannerAlgorithm.Kind GetAlgorithmKind() => plannerAlgorithm switch
        {
            PlannerAlgorithm.RRTConnectFast => RobotBaseSixAxisPlannerAlgorithm.Kind.RrtConnectFast,
            PlannerAlgorithm.RRTConnectOptimized => RobotBaseSixAxisPlannerAlgorithm.Kind.RrtConnectOptimized,
            PlannerAlgorithm.JointSpacePrm => RobotBaseSixAxisPlannerAlgorithm.Kind.JointSpacePrm,
            PlannerAlgorithm.InformedRrtStar => RobotBaseSixAxisPlannerAlgorithm.Kind.InformedRrtStar,
            PlannerAlgorithm.CartesianLinear => RobotBaseSixAxisPlannerAlgorithm.Kind.CartesianLinear,
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
            AssertCollisionContext();
            var key = new ArmTrackSegmentKey(AtPlanningStation(from), AtPlanningStation(to), collisionContextRevision);
            if (rrtTcpTravelCache.TryGetValue(key, out float distance)) return distance;
            distance = GetTcpTravelDistance(from, to);
            rrtTcpTravelCache[key] = distance;
            return distance;
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
            plannerAlgorithm == PlannerAlgorithm.CartesianLinear ? "Cartesian Linear (MoveL)" :
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

        /// <summary>Delegates Cartesian arc or strict linear planning to a plain C# planner.</summary>
        private bool TryBuildCartesianSegment(float[] startAngles, Vector3 startPosition, Quaternion startRotation,
            float[] endAngles, Vector3 endPosition, Quaternion endRotation, List<float[]> result,
            string segmentLabel = "Current TCP -> Start", bool isApproachToStart = false)
        {
            result.Clear();
            var segmentWorld = CreateAlgorithmWorld();
            if (isApproachToStart) segmentWorld.LinearForceLiftTraverseDescend = false;
            var segmentAlgorithm = new RobotBaseSixAxisPlannerAlgorithm(segmentWorld);
            bool found = segmentAlgorithm.TryPlan(GetAlgorithmKind(),
                    startAngles, endAngles, startPosition, startRotation, endPosition, endRotation,
                    out RobotBaseSixAxisPlannerAlgorithm.Result plan);
            if (!string.IsNullOrEmpty(plan.RoutingDiagnostics))
                Debug.Log($"MoveL routing diagnostics [{segmentLabel}] (mode: {linearRoutingMode}):\n{plan.RoutingDiagnostics}", this);
            if (!found)
            {
                cartesianArcAttempts += plan.RoutesTried;
                hasCartesianFailurePosition = plan.HasFailurePosition;
                cartesianFailurePosition = plan.FailurePosition;
                cartesianFailureLabel = segmentLabel + "\n" + plan.FailureReason;
                lastCartesianFailure = $"Segment [{segmentLabel}]. Tried {plan.RoutesTried} Cartesian route(s). {plan.FailureReason} Clearance is {minimumObstacleClearance * 1000f:F0} mm.";
                rig.SetJointAngles(startAngles);
                Physics.SyncTransforms();
                return false;
            }
            cartesianArcAttempts += plan.RoutesTried;
            planningIterations += plan.Iterations;
            automaticLinearWaypoints.AddRange(plan.AutomaticWaypoints);
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
            int automaticPrefix = automaticLinearWaypoints.Count;
            GetPlanningPose(pose, defaultRotation, out Vector3 targetPosition, out Quaternion targetRotation);
            string segmentLabel = (stageIndex == 0 ? $"Start ({startPose.name})" : $"WP{stageIndex} ({poses[stageIndex - 1].name})") +
                " -> " + (stageIndex == poses.Count - 1 ? $"End Target ({pose.name})" : $"WP{stageIndex + 1} ({pose.name})");
            for (int i = 0; i < candidates.Count && (plannerAlgorithm != PlannerAlgorithm.CartesianLinear || i == 0); i++)
            {
                var segment = new List<float[]>();
                if (!TryBuildCartesianSegment(fromAngles, fromPosition, fromRotation,
                        candidates[i], targetPosition, targetRotation, segment, segmentLabel)) continue;
                for (int p = 1; p < segment.Count; p++) accumulatedPath.Add(segment[p]);
                if (TryBuildCartesianTail(segment[segment.Count - 1], targetPosition, targetRotation,
                        poses, poseSolutions, stageIndex + 1, accumulatedPath, defaultRotation)) return true;
                accumulatedPath.RemoveRange(prefixCount, accumulatedPath.Count - prefixCount);
                automaticLinearWaypoints.RemoveRange(automaticPrefix, automaticLinearWaypoints.Count - automaticPrefix);
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
                Quaternion rotation = RequireMarkerOrientations ? waypoint.rotation : defaultRotation;
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
            if (IsBusy || IsRrtSearchInProgress) return false;
            BeginDiagnosticReport();
            if (!BeginTrackOperation(false)) return false;
            try { bool found = TryPlanLocked(); if (found) CompleteDiagnosticReport(PlanDiagnosticCode.None, Status, PlanReportOutcome.Complete); return found; }
            finally { ReleaseTrackLock(); }
        }

        private bool TryPlanLocked()
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
                return Fail("Collision geometry is missing. Assign robot colliders and colliders beneath Obstacles.", PlanDiagnosticCode.InvalidSetup);
            if (checkEndpointIkBeforePlanning && !CheckEndpointsForPlanning())
                return Fail("Endpoint IK check failed. " + StartIkSummary + ". " + EndIkSummary, PlanDiagnosticCode.EndpointValidation);
            if (ShouldScanLinearOccupancy)
            {
                Status = "Scanning occupancy before Cartesian Linear planning...";
                linearOccupancyScanner.ScanOnce();
                if (!linearOccupancyScanner.TryGetPlanningSnapshot(out _, out string scanError))
                    return Fail("Occupancy scan failed: " + scanError);
            }

            float[] originalAngles = rig.GetJointAnglesDegrees();
            try
            {
                Quaternion preferredRotation = GetCurrentTcp().rotation;
                Quaternion startRotation = RequireMarkerOrientations ? startPose.rotation : preferredRotation;
                Quaternion goalRotation = RequireMarkerOrientations ? endTarget.rotation : preferredRotation;
                InitializeGoalSelection(goalRotation);
                if (!inverseKinematics.SolveAll(startPose.position, startRotation, out List<float[]> starts))
                    return Fail("Start pose is unreachable by the analytic IK.", PlanDiagnosticCode.IkNoSolution);

                List<float[]> validStarts = FilterPoseSolutions(starts, startPose, startRotation, out PoseCheckReport startReport);
                rig.SetJointAngles(originalAngles);
                Physics.SyncTransforms();
                List<float[]> validGoals = SolveGoalPose(plannedGoalPosition, plannedGoalRotation, false, out PoseCheckReport goalReport);
                if (validStarts.Count > 0 && validGoals.Count == 0 && AllowAdaptiveGoal && !ExactIntermediateRetryEnabled)
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
                for (int s = 0; s < validStarts.Count && (plannerAlgorithm != PlannerAlgorithm.CartesianLinear || s == 0); s++)
                {
                    path.Clear();
                    if (IsCartesianPlanner)
                    {
                        automaticLinearWaypoints.Clear();
                        rig.SetJointAngles(originalAngles);
                        Physics.SyncTransforms();
                        Transform tcp = GetCurrentTcp();
                        var prefix = new List<float[]>();
                        if (!TryBuildCartesianSegment(originalAngles, tcp.position, tcp.rotation,
                                validStarts[s], startPose.position, startRotation, prefix, isApproachToStart: true))
                        {
                            lastCartesianFailure = $"Current TCP to Start: {lastCartesianFailure}";
                            continue;
                        }
                        path.AddRange(prefix);
                        var tailPoses = new List<Transform>(viaPoses) { endTarget };
                        var tailSolutions = new List<List<float[]>>(viaSolutions) { validGoals };
                        if (TryBuildCartesianTail(prefix[prefix.Count - 1], startPose.position, startRotation,
                                tailPoses, tailSolutions, 0, path, preferredRotation))
                        {
                            hasCartesianFailurePosition = false;
                            SetPlanSummary($"Plan found with {GetPlannerAlgorithmLabel()} after {cartesianArcAttempts} Cartesian route attempts in {FinishPlanningTimer()} ({path.Count} waypoints).");
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
                        rig.SetJointAngles(originalAngles);
                        Physics.SyncTransforms();
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

                if (IsCartesianPlanner)
                    return Fail($"{GetPlannerAlgorithmLabel()} failed. {lastCartesianFailure}");
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
            bool selectingStation = stationSearch != null || coordinatedSearch != null || (endpointReachabilityGateActive && CurrentMotionState != MotionState.Moving && CurrentMotionState != MotionState.Complete);
            activeStateEvaluation?.Dispose();
            endpointReachabilityGateActive = false;
            CancelCoordinatedSearch();
            RollbackStationSearch();
            LinearTrack?.HoldForStop();
            LastStoppedState = CaptureCombinedState();
            bool planning = selectingStation || CurrentMotionState == MotionState.Planning || IsRrtSearchInProgress;
            // Capture before coroutine disposal: a search's finally block may restore
            // its query seed. Stop must hold the pose visible when the user clicks.
            float[] heldAngles = rig != null && rig.IsReady ? rig.GetJointAnglesDegrees() : null;
            if (planning) planningIterations = Mathf.Max(planningIterations, CurrentRrtSearchIterations);
            CancelGridSearch();
            StopAllCoroutines();
            ReleaseTrackLock();
            unchecked { collisionContextRevision++; }
            rrtCollisionCache.Clear(); rrtTcpTravelCache.Clear();
            execution = null;
            if (inverseKinematics != null) inverseKinematics.SolveContinuously = false;
            if (heldAngles != null) { rig.SetJointAngles(heldAngles); Physics.SyncTransforms(); }
            LastStopWasPlanning = planning;
            if (planning)
            {
                ClearCoordinatedPlan();
                exactStationPlan = false;
                hasTrackContext = false;
                plannedTrackState = null;
                FinishPlanningTimer();
                path.Clear();
                motionProfile = null;
                estimatedTravelTimeSeconds = 0f;
                lastPlanSummary = string.Empty;
                ClearGridGizmoData();
                ClearPathVisualization();
            }
            Status = planning ? "Planning cancelled by user. Partial route discarded; no path will execute."
                : path.Count > 1 ? "Motion stopped by user. Robot held at its current pose; replay remains available."
                : "Motion stopped by user. Robot held at its current pose.";
            if (LastStoppedState != null && LastStoppedState.HasTrackAxis) Status += $" E1 held at {LastStoppedState.E1Millimeters:F2} mm.";
            CurrentMotionState = MotionState.Stopped;
            CompleteDiagnosticReport(planning ? PlanDiagnosticCode.Cancelled : PlanDiagnosticCode.MotionStopped, Status, planning ? PlanReportOutcome.Cancelled : PlanReportOutcome.Stopped);
        }

        private List<float[]> FilterPoseSolutions(List<float[]> candidates, Transform pose,
            Quaternion targetRotation, out PoseCheckReport report)
            => FilterPoseSolutions(candidates, pose.position, targetRotation, out report);

        private List<float[]> FilterPoseSolutions(List<float[]> candidates, Vector3 targetPosition,
            Quaternion targetRotation, out PoseCheckReport report, bool enforceAdaptiveBounds = false, bool includeClearance = true)
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
                if (IsInCollision(out string collisionPair, includeClearance: includeClearance))
                {
                    report.collisionCount++;
                    if (report.firstCollidingAngles == null)
                    {
                        report.firstCollisionPair = collisionPair;
                        report.firstCollidingAngles = (float[])candidates[i].Clone();
                    }
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
            if (plannedTrackState != null) summary += $" E1 locked at {plannedTrackState.E1Millimeters:F2} mm.";
            if (plannerAlgorithm == PlannerAlgorithm.CartesianLinear && automaticLinearWaypoints.Count > 0)
                summary += $" Automatically inserted {automaticLinearWaypoints.Count} MoveL waypoint(s).";
            lastPlanSummary = summary + (!adaptiveGoalUsed ? string.Empty :
                $" Adaptive goal: {adaptiveGoalPositionOffsetMillimeters:F2} mm position offset / {adaptiveGoalRotationOffsetDegrees:F2} degrees rotation " +
                $"({adaptiveGoalSamplesTried} samples; minimum clearance {minimumObstacleClearance * 1000f:F1} mm).");
        }

        private void GetPlanningPose(Transform pose, Quaternion defaultRotation, out Vector3 position, out Quaternion rotation)
        {
            position = pose == endTarget ? plannedGoalPosition : pose.position;
            rotation = pose == endTarget ? plannedGoalRotation : RequireMarkerOrientations ? pose.rotation : defaultRotation;
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

        private bool ShouldScanLinearOccupancy => plannerAlgorithm == PlannerAlgorithm.CartesianLinear &&
            automaticallyFindLinearWaypoints && linearRoutingMode == LinearRoutingMode.AutomaticFreeSpace &&
            linearOccupancyScanner != null && scanLinearOccupancyBeforePlanning;

        private RobotBaseSixAxisPlannerAlgorithm.World CreateAlgorithmWorld()
        {
            Bounds workspace = new Bounds(startPose != null ? startPose.position : transform.position, Vector3.zero);
            if (endTarget != null) workspace.Encapsulate(endTarget.position);
            if (intermediateWaypoints != null)
                foreach (Transform waypoint in intermediateWaypoints) if (waypoint != null) workspace.Encapsulate(waypoint.position);
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
                GetLastCollisionDescription = () => lastCollisionPair,
                GetLastCollisionTcpPosition = () => lastCollisionTcpPosition,
                LinearAutomaticWaypoints = automaticallyFindLinearWaypoints,
                LinearForceLiftTraverseDescend = linearRoutingMode == LinearRoutingMode.ForceLiftTraverseDescend,
                LinearMinimumLiftMeters = Mathf.Max(10f, linearMinimumLiftMillimeters) * 0.001f,
                LinearWaypointMaximumEdges = Mathf.Max(1, linearWaypointMaximumEdgeChecks),
                LinearWaypointSearchSeconds = Mathf.Max(0.1f, linearWaypointSearchSeconds),
                LinearOptimizePath = optimizeLinearPath,
                LinearOptimizationSeconds = Mathf.Max(0f, linearPathOptimizationSeconds),
                LinearRoutingObstacles = Array.ConvertAll(Array.FindAll(bounds, c => c != null && c.enabled && !c.isTrigger), c => c.bounds),
                LinearAllowAlternateGoalConfiguration = plannerAlgorithm == PlannerAlgorithm.CartesianLinear,
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
            world.GetJacobian = angles =>
            {
                // Revolute joint axes are invariant under their own rotation. World
                // origins/axes give the TCP Jacobian without finite-difference FK sweeps.
                if (!rig.SetJointAngles(angles)) throw new InvalidOperationException("Could not evaluate Jacobian state.");
                var origins = new Vector3[JointCount];
                var axes = new Vector3[JointCount];
                for (int j = 0; j < JointCount; j++)
                {
                    origins[j] = rig.Joints[j].position;
                    axes[j] = rig.Joints[j].TransformDirection(rig.LocalAxes[j]).normalized;
                }
                return CartesianLinearPlanner.GeometricJacobian(origins, axes, GetCurrentTcp().position,
                    world.LinearCharacteristicLengthMeters);
            };
            if (plannerAlgorithm == PlannerAlgorithm.CartesianLinear && automaticallyFindLinearWaypoints &&
                linearRoutingMode == LinearRoutingMode.AutomaticFreeSpace && linearOccupancyScanner != null)
            {
                if (linearOccupancyScanner.TryGetPlanningSnapshot(out var snapshot, out string error))
                    world.LinearOccupancyGrid = snapshot;
                else world.LinearOccupancyError = error;
                world.LinearGridMaximumNodes = Mathf.Max(1, linearGridMaximumNodes);
                world.LinearGridMaximumRouteAttempts = Mathf.Max(1, linearGridMaximumRouteAttempts);
            }
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
            bool collision = IsInCollision(out lastCollisionPair);
            if (collision) lastCollisionTcpPosition = GetCurrentTcp().position;
            return collision;
        }

        private readonly RobotColliderContactQuery contactQuery = new RobotColliderContactQuery();

        private bool IsInCollision(out string collisionPair, ISet<Collider> contacts = null, bool includeClearance = true)
        {
            collisionPair = null;
            if (LastPlanReport?.Outcome == PlanReportOutcome.Running) LastPlanReport.CollisionEvaluations++;
            if (ReachRejects(rig.GetJointAnglesDegrees(), false, out collisionPair))
            { CaptureDiagnostic(PlanDiagnosticCode.StretchLimit, collisionPair, rig.GetJointAnglesDegrees(), measured: MeasureStretchPercent(rig.GetJointAnglesDegrees()), required: maximumStretchPercent, units: "%"); return true; }
            float requiredClearance = includeClearance ? minimumObstacleClearance : 0f;
            var bodies = collisionRobotBodies ?? robotColliders;
            var track = LinearTrack;
            if (track != null && !track.TrackEnabled) track = null;
            for (int i = 0; i < bodies.Length; i++)
            {
                Collider robotCollider = bodies[i];
                if (robotCollider == null || !robotCollider.enabled || !robotCollider.gameObject.activeInHierarchy || robotCollider.isTrigger) continue;
                Bounds robotBounds = robotCollider.bounds;
                for (int j = 0; j < obstacleColliders.Length; j++)
                {
                    Collider obstacle = obstacleColliders[j];
                    if (obstacle == null || !obstacle.enabled || !obstacle.gameObject.activeInHierarchy || obstacle.isTrigger) continue;
                    if (obstacle == robotCollider || (track != null &&
                        ((track.IsCarriageCollider(robotCollider) && track.IsCarriageCollider(obstacle)) || track.AllowsContact(robotCollider, obstacle)))) continue;
                    if (!PlanningQueryUtilities.CouldViolateClearance(robotBounds, obstacle.bounds, requiredClearance)) continue;
                    bool overlapping = Physics.ComputePenetration(robotCollider, robotCollider.transform.position,
                            robotCollider.transform.rotation, obstacle, obstacle.transform.position,
                            obstacle.transform.rotation, out _, out _);
                    // Scene overlap queries test mesh surfaces even when penetration
                    // cannot produce a separation vector for a concave/backfacing mesh.
                    if (!overlapping && contactQuery.TryOverlap(robotCollider, obstacle, 0f, out bool surfaceContact)) overlapping = surfaceContact;
                    if (overlapping)
                    {
                        if (collisionPair == null) collisionPair = $"Robot link {DescribeCollisionObject(robotCollider)} / obstacle {DescribeCollisionObject(obstacle)} (penetration)";
                        CaptureDiagnostic(PlanDiagnosticCode.Collision, collisionPair, rig.GetJointAnglesDegrees(), robot: robotCollider, obstacle: obstacle, evidence: "Physics penetration / surface overlap; marker is TCP, not contact point.");
                        if (contacts != null) { contacts.Add(robotCollider); continue; }
                        if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                            SetGridFailurePosition((robotCollider.ClosestPoint(obstacle.bounds.center) +
                                obstacle.ClosestPoint(robotCollider.bounds.center)) * 0.5f);
                        return true;
                    }
                    bool concaveMesh = obstacle is MeshCollider mesh && !mesh.convex;
                    bool envelopeViolation = false;
                    if (requiredClearance > 0f && concaveMesh)
                    {
                        bool supported = contactQuery.TryOverlap(robotCollider, obstacle, requiredClearance, out bool nearSurface);
                        // Unsupported robot shapes retain conservative rejection rather
                        // than silently bypassing clearance against a concave obstacle.
                        envelopeViolation = !supported || nearSurface;
                    }
                    // ClosestPoint does not provide a usable distance for this concave
                    // mesh pairing. Use a conservative expanded primitive envelope.
                    float clearance = requiredClearance > 0f && !concaveMesh ? GetColliderSeparation(robotCollider, obstacle) : float.PositiveInfinity;
                    if (envelopeViolation || (requiredClearance > 0f && clearance < requiredClearance))
                    {
                        if (collisionPair == null) collisionPair = $"Robot link {DescribeCollisionObject(robotCollider)} / obstacle {DescribeCollisionObject(obstacle)} " +
                            (envelopeViolation ? $"(inside {requiredClearance * 1000f:F2} mm clearance envelope)" : $"(clearance {clearance * 1000f:F2} mm < {requiredClearance * 1000f:F2} mm)");
                        CaptureDiagnostic(PlanDiagnosticCode.Clearance, collisionPair, rig.GetJointAnglesDegrees(), measured: envelopeViolation ? (float?)null : clearance * 1000f, required: requiredClearance * 1000f, units: "mm", robot: robotCollider, obstacle: obstacle,
                            evidence: envelopeViolation ? "Conservative concave-mesh expanded envelope; exact separation unavailable." : "Collider separation below minimum clearance.");
                        if (contacts != null) { contacts.Add(robotCollider); continue; }
                        if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                            SetGridFailurePosition((robotCollider.ClosestPoint(obstacle.bounds.center) +
                                obstacle.ClosestPoint(robotCollider.bounds.center)) * 0.5f);
                        return true;
                    }
                }
            }

            if (!checkSelfCollision) return collisionPair != null;
            for (int a = 0; a < robotColliders.Length; a++)
            for (int b = a + 1; b < robotColliders.Length; b++)
            {
                Collider first = robotColliders[a];
                Collider second = robotColliders[b];
                if (first == null || second == null || !first.enabled || !second.enabled ||
                    !first.gameObject.activeInHierarchy || !second.gameObject.activeInHierarchy || first.isTrigger || second.isTrigger ||
                    AreSameOrAdjacentLink(first.transform, second.transform)) continue;
                if (!PlanningQueryUtilities.CouldViolateClearance(first.bounds, second.bounds, 0f)) continue;
                if (Physics.ComputePenetration(first, first.transform.position, first.transform.rotation,
                        second, second.transform.position, second.transform.rotation, out _, out _))
                {
                    if (collisionPair == null) collisionPair = $"Self collision: {DescribeCollisionObject(first)} / {DescribeCollisionObject(second)}";
                    CaptureDiagnostic(PlanDiagnosticCode.Collision, collisionPair, rig.GetJointAnglesDegrees(), robot: first, obstacle: second, evidence: "Self penetration");
                    if (contacts != null) { contacts.Add(first); contacts.Add(second); continue; }
                    if (plannerAlgorithm == PlannerAlgorithm.GridAStar)
                        SetGridFailurePosition((first.ClosestPoint(second.bounds.center) +
                            second.ClosestPoint(first.bounds.center)) * 0.5f);
                    return true;
                }
            }
            return collisionPair != null;
        }

        private sealed class CollisionPreviewHighlight
        {
            public Renderer Renderer;
            public Material[] Materials;
            public MaterialPropertyBlock Properties;
            public MaterialPropertyBlock[] MaterialProperties;
        }
        private readonly Dictionary<Renderer, CollisionPreviewHighlight> collisionPreviewHighlights = new Dictionary<Renderer, CollisionPreviewHighlight>();
        private Material collisionPreviewMaterial;

        private void HighlightCollisionPart(Collider contact)
        {
            if (contact == null) return;
            bool robotPart = contact.transform == rig.transform || contact.transform.IsChildOf(rig.transform);
            if (!robotPart) return;
            int joint = GetOwningJointIndex(contact.transform);
            var renderers = contact.GetComponentsInChildren<Renderer>(true);
            if (renderers.Length == 0)
            {
                Renderer parent = contact.GetComponentInParent<Renderer>();
                if (parent != null) renderers = new[] { parent };
            }
            foreach (var renderer in renderers)
            {
                // Joint hierarchies contain later links too. Tint only this link's geometry.
                if (renderer is LineRenderer || renderer is TrailRenderer || collisionPreviewHighlights.ContainsKey(renderer) ||
                    (renderer.transform != rig.transform && !renderer.transform.IsChildOf(rig.transform)) ||
                    (joint >= 0 && GetOwningJointIndex(renderer.transform) != joint)) continue;
                if (collisionPreviewMaterial == null)
                {
                    Shader shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
                    if (shader == null) return;
                    collisionPreviewMaterial = new Material(shader) { name = "IK collision preview red", hideFlags = HideFlags.HideAndDontSave };
                    Color red = new Color(1f, 0.03f, 0.02f, 1f);
                    if (collisionPreviewMaterial.HasProperty("_UnlitColor")) collisionPreviewMaterial.SetColor("_UnlitColor", red);
                    if (collisionPreviewMaterial.HasProperty("_Color")) collisionPreviewMaterial.SetColor("_Color", red);
                    if (collisionPreviewMaterial.HasProperty("_BaseColor")) collisionPreviewMaterial.SetColor("_BaseColor", red);
                }
                var materials = renderer.sharedMaterials;
                var saved = new CollisionPreviewHighlight
                {
                    Renderer = renderer, Materials = materials, Properties = new MaterialPropertyBlock(),
                    MaterialProperties = new MaterialPropertyBlock[materials.Length]
                };
                renderer.GetPropertyBlock(saved.Properties);
                var redMaterials = new Material[materials.Length];
                for (int i = 0; i < materials.Length; i++)
                {
                    saved.MaterialProperties[i] = new MaterialPropertyBlock();
                    renderer.GetPropertyBlock(saved.MaterialProperties[i], i);
                    renderer.SetPropertyBlock(null, i);
                    redMaterials[i] = collisionPreviewMaterial;
                }
                collisionPreviewHighlights.Add(renderer, saved);
                renderer.SetPropertyBlock(null);
                renderer.sharedMaterials = redMaterials;
            }
        }

        private void ClearCollisionPreviewHighlights()
        {
            foreach (var saved in collisionPreviewHighlights.Values)
            {
                if (saved.Renderer == null) continue;
                saved.Renderer.sharedMaterials = saved.Materials;
                saved.Renderer.SetPropertyBlock(saved.Properties.isEmpty ? null : saved.Properties);
                for (int i = 0; i < saved.MaterialProperties.Length; i++)
                    saved.Renderer.SetPropertyBlock(saved.MaterialProperties[i].isEmpty ? null : saved.MaterialProperties[i], i);
            }
            collisionPreviewHighlights.Clear();
        }

        private static string DescribeCollisionObject(Collider collider)
        {
            string path = collider.name;
            for (Transform parent = collider.transform.parent; parent != null; parent = parent.parent)
                path = parent.name + "/" + path;
            return path + " (" + collider.GetType().Name + ")";
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
                bool linear = plannerAlgorithm == PlannerAlgorithm.CartesianLinear;
                CartesianLinearPlanner evaluator = null;
                CartesianOccupancyGrid blendGrid = null;
                int[] cornerIndices = null;
                if (linear)
                {
                    var world = CreateAlgorithmWorld();
                    evaluator = new CartesianLinearPlanner(world);
                    var positions = new List<Vector3>(path.Count);
                    foreach (var angles in path)
                    {
                        rig.SetJointAngles(angles);
                        positions.Add(GetCurrentTcp().position);
                    }
                    // Identify TCP corners without dropping any validated execution samples.
                    cornerIndices = PathDisplayWaypoints.Select(positions, null);
                    blendGrid = world.LinearOccupancyGrid;
                    if (blendGrid != null)
                    {
                        var endpointCells = new HashSet<int>();
                        void Include(Vector3 position)
                        {
                            if (blendGrid.TryCell(position, out int id)) endpointCells.Add(id);
                        }
                        Include(positions[0]); Include(positions[positions.Count - 1]);
                        if (startPose != null) Include(startPose.position);
                        if (intermediateWaypoints != null)
                            foreach (var marker in intermediateWaypoints) if (marker != null) Include(marker.position);
                        blendGrid = blendGrid.WithValidatedEndpointCells(endpointCells);
                    }
                }
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
                    EnableCornerSmoothing = true,
                    PathAccuracyMillimeters = pathAccuracyMillimeters,
                    CollisionResolutionDegrees = linear ? Mathf.Min(0.25f, collisionResolutionDegrees) : collisionResolutionDegrees
                }, angles =>
                {
                    for (int j = 0; j < angles.Length; j++)
                        if (angles[j] < rig.AngleLimitsDegrees[j].x || angles[j] > rig.AngleLimitsDegrees[j].y) return false;
                    if (!rig.SetJointAngles(angles)) return false;
                    Physics.SyncTransforms();
                    if (IsInCollision()) return false;
                    if (blendGrid != null && !blendGrid.IsFree(GetCurrentTcp().position)) return false;
                    return evaluator == null || evaluator.GetSingularityRejection(angles) == null;
                }, cornerWaypointIndices: cornerIndices, isSegmentValid: linear ? (from, to) =>
                {
                    if (!IsSegmentFree(from, to)) return false;
                    if (blendGrid == null) return true;
                    rig.SetJointAngles(from); Vector3 a = GetCurrentTcp().position;
                    rig.SetJointAngles(to); Vector3 b = GetCurrentTcp().position;
                    return blendGrid.SegmentFree(a, b);
                } : (Func<float[], float[], bool>)null);
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
            UpdateStretchFeedback();
            if (!TrackContextMatches(out string initialTrackError)) { AbortTrackChange(initialTrackError); yield break; }
            // Rebuild before replay as well: Inspector limits and obstacles may have changed.
            DrawPlannedPath();
            MotionProfile profile = motionProfile;
            if (profile == null) { execution = null; yield break; }
            CurrentMotionState = MotionState.Moving;
            Status = "Executing planned motion.";
            float elapsed = 0f;
            while (elapsed < profile.DurationSeconds)
            {
                if (!TrackContextMatches(out string trackError)) { AbortTrackChange(trackError); yield break; }
                elapsed += Time.deltaTime;
                var nextAngles = profile.Evaluate(elapsed);
                if (ReachRejects(nextAngles, false, out string reachReason))
                { StopMotion(); Status = "Motion stopped: " + reachReason; CurrentMotionState = MotionState.Failed; yield break; }
                ObserveReach(nextAngles, false); rig.SetJointAngles(nextAngles);
                yield return null;
            }
            rig.SetJointAngles(profile.Evaluate(profile.DurationSeconds));
            Physics.SyncTransforms();
            float finalPositionError = Vector3.Distance(GetCurrentTcp().position, plannedGoalPosition);
            float finalRotationError = Quaternion.Angle(GetCurrentTcp().rotation, plannedGoalRotation);
            if (finalPositionError > positionClosureTolerance ||
                (RequireMarkerOrientations && finalRotationError > rotationClosureToleranceDegrees))
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
            if (coordinatedPath.Count > 1) { DrawCoordinatedPath(); return; }
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
            if (plannerAlgorithm == PlannerAlgorithm.CartesianLinear)
            {
                var required = new List<Vector3>(automaticLinearWaypoints);
                if (startPose != null) required.Add(startPose.position);
                if (intermediateWaypoints != null)
                    foreach (Transform marker in intermediateWaypoints) if (marker != null) required.Add(marker.position);
                var indices = PathDisplayWaypoints.Select(waypoints, waypointRotations, required);
                var displayPositions = new List<Vector3>(); var displayRotations = new List<Quaternion>();
                foreach (int index in indices) { displayPositions.Add(waypoints[index]); displayRotations.Add(waypointRotations[index]); }
                waypoints = displayPositions; waypointRotations = displayRotations;
            }
            gizmoWaypoints = waypoints.ToArray();
            gizmoWaypointRotations = waypointRotations.ToArray();

            if (pathRenderer == null)
            {
                Transform existing = transform.Find("Planned TCP Path");
                GameObject lineObject = existing != null ? existing.gameObject : new GameObject("Planned TCP Path");
                if (existing == null) lineObject.transform.SetParent(transform, false);
                pathRenderer = lineObject.GetComponent<LineRenderer>();
                if (pathRenderer == null) pathRenderer = lineObject.AddComponent<LineRenderer>();
            }
            ConfigurePathRenderer();
            pathRenderer.positionCount = points.Count;
            pathRenderer.SetPositions(points.ToArray());
            UpdateWaypointLabels(waypoints);
            Physics.SyncTransforms();
        }

        private void ConfigurePathRenderer()
        {
            pathRenderer.enabled = pathOverlayVisible && pathTransparencyPercent < 100;
            pathRenderer.forceRenderingOff = screenPathOverlayOwner != null;
            pathRenderer.useWorldSpace = true;
            pathRenderer.alignment = LineAlignment.View;
            pathRenderer.widthMultiplier = 0.008f;
            pathRenderer.startColor = pathRenderer.endColor = Color.cyan;
            pathRenderer.numCornerVertices = pathRenderer.numCapVertices = 6;
            pathRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            pathRenderer.receiveShadows = false;
            pathRenderer.allowOcclusionWhenDynamic = false;
            if (pathMaterial == null)
            {
                Shader shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
                if (shader == null) return;
                pathMaterial = new Material(shader) { name = "Opaque Planned TCP Path", hideFlags = HideFlags.DontSave };
                if (pathMaterial.HasProperty("_UnlitColor")) pathMaterial.SetColor("_UnlitColor", Color.cyan);
                if (pathMaterial.HasProperty("_Color")) pathMaterial.SetColor("_Color", Color.cyan);
                foreach (string property in new[] { "_SurfaceType", "_AlphaCutoffEnable", "_OpaqueCullMode", "_CullMode" })
                    if (pathMaterial.HasProperty(property)) pathMaterial.SetFloat(property, 0f);
                if (pathMaterial.HasProperty("_DoubleSidedEnable")) pathMaterial.SetFloat("_DoubleSidedEnable", 1f);
                if (shader.name == "HDRP/Unlit") UnityEngine.Rendering.HighDefinition.HDMaterial.ValidateMaterial(pathMaterial);
            }
            pathRenderer.sharedMaterial = pathMaterial;
        }

        private void UpdateEstimatedTravelTime()
        {
            BuildMotionProfile();
            estimatedTravelTimeSeconds = motionProfile != null ? motionProfile.DurationSeconds : 0f;
        }

        private void UpdateWaypointLabels(List<Vector3> waypoints)
        {
            if (!Application.isPlaying) return;
            UpdateWaypointMarkers(waypoints);
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

            // Recover the owned pool after a domain reload, then destroy excess
            // sample labels from older versions instead of leaving hundreds hidden.
            if (waypointLabels.Count == 0)
                foreach (TextMesh label in waypointLabelsRoot.GetComponentsInChildren<TextMesh>(true))
                    if (label.transform.parent == waypointLabelsRoot && label.name.StartsWith("Waypoint Label ")) waypointLabels.Add(label);
            waypointLabels.RemoveAll(label => label == null);
            while (waypointLabels.Count > waypoints.Count)
            {
                int last = waypointLabels.Count - 1;
                GameObject owned = waypointLabels[last].gameObject;
                waypointLabels.RemoveAt(last); owned.SetActive(false); Destroy(owned);
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

        private void UpdateWaypointMarkers(List<Vector3> waypoints)
        {
            if (waypointMarkersRoot == null)
            {
                waypointMarkersRoot = transform.Find("Planned TCP Waypoint Markers");
                if (waypointMarkersRoot == null)
                {
                    var owned = new GameObject("Planned TCP Waypoint Markers"); owned.transform.SetParent(transform, false); waypointMarkersRoot = owned.transform;
                }
            }
            if (waypointMarkers.Count == 0)
                foreach (var renderer in waypointMarkersRoot.GetComponentsInChildren<MeshRenderer>(true))
                    if (renderer.transform.parent == waypointMarkersRoot && renderer.name.StartsWith("Waypoint Marker ")) waypointMarkers.Add(renderer);
            waypointMarkers.RemoveAll(marker => marker == null);
            while (waypointMarkers.Count > waypoints.Count)
            {
                int last = waypointMarkers.Count - 1; var owned = waypointMarkers[last].gameObject;
                waypointMarkers.RemoveAt(last); owned.SetActive(false); Destroy(owned);
            }
            for (int i = 0; i < waypoints.Count; i++)
            {
                if (waypointMarkers.Count <= i)
                {
                    var owned = GameObject.CreatePrimitive(PrimitiveType.Sphere); owned.name = $"Waypoint Marker {i}"; owned.layer = 2;
                    if (owned.TryGetComponent<Collider>(out var collider)) { collider.enabled = false; Destroy(collider); }
                    owned.transform.SetParent(waypointMarkersRoot, false);
                    var renderer = owned.GetComponent<MeshRenderer>(); renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    renderer.receiveShadows = false; waypointMarkers.Add(renderer);
                }
                var marker = waypointMarkers[i]; marker.sharedMaterial = pathMaterial;
                marker.transform.position = waypoints[i]; marker.transform.localScale = Vector3.one * (i == 0 || i == waypoints.Count - 1 ? .024f : .018f);
                marker.gameObject.SetActive(showWaypointLabels);
            }
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
            foreach (var marker in waypointMarkers) if (marker != null) marker.gameObject.SetActive(active);
            for (int i = 0; i < waypointLabels.Count; i++)
            {
                if (waypointLabels[i] != null) waypointLabels[i].gameObject.SetActive(active);
            }
        }

        private bool ValidateSetup()
        {
            CorrectSwappedPoseReferences();
            if (rig == null || inverseKinematics == null || !rig.IsReady)
                return Fail("RobotBaseSixAxisRig and RobotBaseSixAxisAnalyticIK must be configured on this robot.", PlanDiagnosticCode.InvalidSetup);
            if (startPose == null || endTarget == null || obstaclesRoot == null)
                return Fail("Assign Start, End Target, and the Obstacles root in the inspector.", PlanDiagnosticCode.InvalidSetup);
            return PrepareStretchPolicy();
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
            if (rig != null && (robotColliders == null || Array.TrueForAll(robotColliders, collider => collider == null)))
            {
                robotColliders = Array.FindAll(rig.GetComponentsInChildren<Collider>(true), collider => !IsOwnedWaypointMarker(collider.transform));
                if (robotColliders.Length == 0 && generateRobotBoxCollidersIfMissing)
                {
                    GenerateMeshBoundBoxes();
                    robotColliders = Array.FindAll(rig.GetComponentsInChildren<Collider>(true), collider => !IsOwnedWaypointMarker(collider.transform));
                }
            }
            ApplyRobotBoxColliderScale();
            var track = LinearTrack;
            bool tracked = track != null && track.TrackEnabled;
            var bodies = new List<Collider>(robotColliders ?? Array.Empty<Collider>());
            if (tracked)
                foreach (var collider in track.CarriageColliders)
                    if (collider != null && !bodies.Contains(collider)) bodies.Add(collider);
            collisionRobotBodies = bodies.ToArray();
            var obstacles = new List<Collider>();
            var known = new HashSet<Collider>();
            if (obstaclesRoot != null)
                foreach (var collider in obstaclesRoot.GetComponentsInChildren<Collider>(true))
                    if (known.Add(collider)) obstacles.Add(collider);
            if (additionalObstacleColliders != null)
                foreach (var collider in additionalObstacleColliders)
                    if (collider != null && (rig == null ||
                        (collider.transform != rig.transform && !collider.transform.IsChildOf(rig.transform))) &&
                        known.Add(collider)) obstacles.Add(collider);
            if (tracked)
                foreach (var group in new[] { track.CarriageColliders, track.StationaryRailColliders })
                    foreach (var collider in group)
                        if (collider != null && known.Add(collider)) obstacles.Add(collider);
            obstacleColliders = obstacles.ToArray();
        }

        private Transform GetCurrentTcp() => rig.ToolFrame != null ? rig.ToolFrame : rig.Joints[5];

        private void GenerateMeshBoundBoxes()
        {
            MeshFilter[] meshes = rig.GetComponentsInChildren<MeshFilter>(true);
            for (int i = 0; i < meshes.Length; i++)
            {
                MeshFilter meshFilter = meshes[i];
                if (IsOwnedWaypointMarker(meshFilter.transform)) continue;
                if (meshFilter.sharedMesh == null || meshFilter.GetComponent<Collider>() != null) continue;
                BoxCollider box = meshFilter.gameObject.AddComponent<BoxCollider>();
                Bounds bounds = meshFilter.sharedMesh.bounds;
                box.center = bounds.center;
                box.size = bounds.size;
            }
        }
        private bool IsOwnedWaypointMarker(Transform item)
        {
            for (var parent = item; parent != null; parent = parent.parent)
                if (parent.name == "Planner Failed Position" || parent.name == "Planner Rejected Pose") return true;
            if ((failureHighlightRoot != null && item.IsChildOf(failureHighlightRoot)) || (failureGhostRoot != null && item.IsChildOf(failureGhostRoot))) return true;
            var owned = waypointMarkersRoot != null ? waypointMarkersRoot : transform.Find("Planned TCP Waypoint Markers");
            return owned != null && item.IsChildOf(owned);
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

        private bool Fail(string message, PlanDiagnosticCode code = PlanDiagnosticCode.NoCompleteRoute)
        {
            CurrentMotionState = MotionState.Failed;
            if (planningTimer != null && planningTimer.IsRunning)
                message += $" Planning time: {FinishPlanningTimer()}.";
            if (plannerAlgorithm == PlannerAlgorithm.GridAStar && hasGridFailurePosition)
                message += $" Failure location: {gizmoGridFailurePosition.x:F3}, {gizmoGridFailurePosition.y:F3}, {gizmoGridFailurePosition.z:F3} m.";
            if (IsCartesianPlanner && hasCartesianFailurePosition)
                message += $" Rejected TCP position: {cartesianFailurePosition.x:F3}, {cartesianFailurePosition.y:F3}, {cartesianFailurePosition.z:F3} m (world).";
            Status = message;
            // A station may fail while the outer automatic-station search continues.
            if (stationSearch == null) CompleteDiagnosticReport(code, message);
            Debug.LogWarning($"Motion planner: {message}", this);
#if UNITY_EDITOR
            UnityEditor.SceneView.RepaintAll();
#endif
            return false;
        }

        private void ClearGridGizmoData()
        {
            automaticLinearWaypoints.Clear();
            hasCartesianFailurePosition = false;
            cartesianFailureLabel = null;
#if UNITY_EDITOR
            cartesianDiagnosticScroll = Vector2.zero;
#endif
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
            return Mathf.Sqrt(sum) * (1 + .5f * (ReachPenalty(a) + ReachPenalty(b)));
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
            public float[] firstCollidingAngles;
        }

#if UNITY_EDITOR
        private void DrawCartesianFailureDiagnostics()
        {
            var scene = UnityEditor.SceneView.currentDrawingSceneView;
            if (scene == null || scene.camera == null) return;
            Vector3 anchor = UnityEditor.HandleUtility.WorldToGUIPointWithDepth(cartesianFailurePosition);
            if (anchor.z <= 0f) return;
            float viewportWidth = scene.camera.pixelWidth / UnityEditor.EditorGUIUtility.pixelsPerPoint;
            float viewportHeight = scene.camera.pixelHeight / UnityEditor.EditorGUIUtility.pixelsPerPoint;
            if (viewportWidth < 100f || viewportHeight < 100f) return;
            float width = Mathf.Min(520f, viewportWidth - 24f);
            float textWidth = width - 36f;
            var style = new GUIStyle(UnityEditor.EditorStyles.label)
            {
                wordWrap = true,
                richText = false,
                fontSize = 12,
                padding = new RectOffset(4, 4, 4, 4)
            };
            style.normal.textColor = Color.white;
            // Break between candidates and allow long hierarchy paths to wrap too.
            string diagnostic = (cartesianFailureLabel ?? string.Empty)
                .Replace("; ", ";\n").Replace("/", "/ ");
            var content = new GUIContent("MoveL rejected TCP position\n" + diagnostic);
            float contentHeight = style.CalcHeight(content, textWidth);
            float height = Mathf.Min(contentHeight + 16f, viewportHeight - 32f);
            var panel = new Rect(
                Mathf.Clamp(anchor.x + 18f, 12f, viewportWidth - width - 12f),
                Mathf.Clamp(anchor.y + 18f, 12f, viewportHeight - height - 12f), width, height);
            UnityEditor.Handles.BeginGUI();
            try
            {
                UnityEditor.EditorGUI.DrawRect(panel, new Color(0.08f, 0.08f, 0.08f, 0.94f));
                var scrollRect = new Rect(panel.x + 6f, panel.y + 6f, panel.width - 12f, panel.height - 12f);
                cartesianDiagnosticScroll = GUI.BeginScrollView(scrollRect, cartesianDiagnosticScroll,
                    new Rect(0f, 0f, textWidth, contentHeight));
                GUI.Label(new Rect(0f, 0f, textWidth, contentHeight), content, style);
                GUI.EndScrollView();
            }
            finally { UnityEditor.Handles.EndGUI(); }
        }
#endif
        // Show requested MoveL geometry even before a successful plan exists.
        private void DrawCartesianWaypointRoute()
        {
            if (plannerAlgorithm != PlannerAlgorithm.CartesianLinear || startPose == null || endTarget == null) return;
            Color previous = Gizmos.color;
            Gizmos.color = new Color(1f, 0.35f, 0.15f);
            Transform from = startPose;
            if (intermediateWaypoints != null)
                for (int i = 0; i < intermediateWaypoints.Length; i++)
                {
                    Transform point = intermediateWaypoints[i];
                    if (point == null) continue;
                    Gizmos.DrawLine(from.position, point.position);
                    Gizmos.DrawSphere(point.position, 0.015f);
                    Gizmos.DrawWireSphere(point.position, 0.025f);
#if UNITY_EDITOR
                    UnityEditor.Handles.Label(point.position + Vector3.up * 0.035f, $"WP{i + 1}: {point.name}");
#endif
                    from = point;
                }
            Gizmos.DrawLine(from.position, endTarget.position);
            Gizmos.color = previous;
        }
        private void OnDrawGizmos()
        {
            DrawCartesianWaypointRoute();
            if (plannerAlgorithm == PlannerAlgorithm.CartesianLinear && CurrentMotionState != MotionState.Failed)
                for (int i = 0; i < automaticLinearWaypoints.Count; i++)
                {
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawSphere(automaticLinearWaypoints[i], 0.018f);
#if UNITY_EDITOR
                    UnityEditor.Handles.Label(automaticLinearWaypoints[i] + Vector3.up * 0.04f, $"Auto WP{i + 1}");
#endif
                }
            if (hasCartesianFailurePosition && IsCartesianPlanner)
            {
                Color previousColor = Gizmos.color;
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(cartesianFailurePosition, 0.012f);
                Gizmos.DrawWireSphere(cartesianFailurePosition, 0.03f);
                Gizmos.DrawLine(cartesianFailurePosition - Vector3.right * 0.05f, cartesianFailurePosition + Vector3.right * 0.05f);
                Gizmos.DrawLine(cartesianFailurePosition - Vector3.up * 0.05f, cartesianFailurePosition + Vector3.up * 0.05f);
#if UNITY_EDITOR
                DrawCartesianFailureDiagnostics();
#endif
                Gizmos.color = previousColor;
            }
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
            // Path strokes are drawn once by the constant-pixel overlays below / in the runtime UI.
            for (int i = 0; i < gizmoWaypoints.Length; i++)
            {
                bool isStart = i == 0;
                bool isEnd = i == gizmoWaypoints.Length - 1;
                Gizmos.color = isStart ? Color.green : isEnd ? Color.red : new Color(1f, 0.65f, 0f);
                if (pathWaypointDots) Gizmos.DrawSphere(isStart || isEnd ? gizmoWaypoints[i] : RobotMotionControls.ClosestPathPoint(gizmoWaypoints[i], gizmoPath), isStart || isEnd ? 0.018f : 0.012f);
#if UNITY_EDITOR
                UnityEditor.Handles.color = Color.white;
                if (isStart || isEnd ? pathEndpointLabels : showWaypointLabels)
                    UnityEditor.Handles.Label(gizmoWaypoints[i] + Vector3.up * 0.025f,
                        isStart ? pathStartText : isEnd ? pathEndText : $"WP {i}");
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
#if UNITY_EDITOR
            // A solid, depth-independent diagnostic overlay stays visible while
            // orbiting the Scene camera, including portions behind scene geometry.
            if (pathOverlayVisible && pathTransparencyPercent < 100 && UnityEditor.SceneView.currentDrawingSceneView != null)
            {
                var previousDepth = UnityEditor.Handles.zTest;
                Color previousColor = UnityEditor.Handles.color;
                try
                {
                    UnityEditor.Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
                    if (pathOutlinePixels > 0)
                    {
                        UnityEditor.Handles.color = new Color(0.02f, 0.04f, 0.05f, PathStrokeColor.a);
                        UnityEditor.Handles.DrawAAPolyLine(pathWidthPixels + 2 * pathOutlinePixels, gizmoPath);
                    }
                    UnityEditor.Handles.color = PathStrokeColor;
                    UnityEditor.Handles.DrawAAPolyLine(pathWidthPixels, gizmoPath);
                }
                finally { UnityEditor.Handles.zTest = previousDepth; UnityEditor.Handles.color = previousColor; }
            }
#endif
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

    /// <summary>Primitive-vs-scene surface overlap, independent of planning clearance.
    /// Kept separate from ComputePenetration, which can ignore mesh backfaces.</summary>
    internal sealed class RobotColliderContactQuery
    {
        private Collider[] hits = new Collider[32];

        public bool TryOverlap(Collider robot, Collider obstacle, float margin, out bool overlapping)
        {
            overlapping = false;
            if (!(robot is BoxCollider) && !(robot is SphereCollider) && !(robot is CapsuleCollider)) return false;
            Vector3 rawScale = robot.transform.lossyScale;
            Vector3 scale = new Vector3(Mathf.Abs(rawScale.x), Mathf.Abs(rawScale.y), Mathf.Abs(rawScale.z));
            int mask = 1 << obstacle.gameObject.layer;
            while (true)
            {
                int count;
                if (robot is BoxCollider box)
                {
                    Vector3 half = Vector3.Scale(box.size * 0.5f, scale) + Vector3.one * margin;
                    count = Physics.OverlapBoxNonAlloc(box.transform.TransformPoint(box.center), half, hits,
                        box.transform.rotation, mask, QueryTriggerInteraction.Ignore);
                }
                else if (robot is SphereCollider sphere)
                {
                    float radius = sphere.radius * Mathf.Max(scale.x, Mathf.Max(scale.y, scale.z)) + margin;
                    count = Physics.OverlapSphereNonAlloc(sphere.transform.TransformPoint(sphere.center), radius, hits,
                        mask, QueryTriggerInteraction.Ignore);
                }
                else
                {
                    var capsule = (CapsuleCollider)robot;
                    int axis = capsule.direction;
                    float radiusScale = axis == 0 ? Mathf.Max(scale.y, scale.z) : axis == 1 ? Mathf.Max(scale.x, scale.z) : Mathf.Max(scale.x, scale.y);
                    float radius = capsule.radius * radiusScale;
                    float halfLine = Mathf.Max(0f, capsule.height * scale[axis] * 0.5f - radius);
                    Vector3 center = capsule.transform.TransformPoint(capsule.center);
                    Vector3 direction = capsule.transform.rotation * (axis == 0 ? Vector3.right : axis == 1 ? Vector3.up : Vector3.forward);
                    count = Physics.OverlapCapsuleNonAlloc(center - direction * halfLine, center + direction * halfLine,
                        radius + margin, hits, mask, QueryTriggerInteraction.Ignore);
                }
                for (int i = 0; i < count; i++)
                    if (hits[i] == obstacle) { overlapping = true; return true; }
                if (count < hits.Length) return true;
                // A full buffer is not proof that the obstacle is absent.
                Array.Resize(ref hits, hits.Length * 2);
            }
        }
    }
}














