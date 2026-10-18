using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Compatibility entry point and shared request/result types for six plain C# planners.
    /// Each search implementation lives in its own class under Planners/.
    /// </summary>
    public sealed class RobotBaseSixAxisPlannerAlgorithm
    {
        public enum Kind { RrtConnectFast, RrtConnectOptimized, CartesianArc, GridAStar, JointSpacePrm, InformedRrtStar }

        /// <summary>Scene/query services required by planning, independent of Unity lifecycle.</summary>
        public sealed class World
        {
            public Func<float[], float[], bool> IsJointSegmentFree;
            public Func<Vector3, Quaternion, List<float[]>> SolveIk;
            public Func<float[], Vector3> GetTcpPosition;
            public Func<float[], Quaternion> GetTcpRotation;
            public Func<float[], bool> IsConfigurationInCollision;
            public Func<float[]> RandomConfiguration;
            public Func<float[], float[], float> MotionCost;
            public Func<float[], float[], float> RouteCost;
            public Func<float[], float[], float> TcpTravelCost;
            public Func<IReadOnlyList<float[]>, float> TcpPathCost;
            public Action<int> IterationProgress;
            public bool PrioritizeShortestPath;
            // Cost in world meters per degree, compatible with TCP travel cost.
            public float Joint6TravelCostPerDegree;
            public CancellationToken CancellationToken;
            public Func<Vector3, float, bool> IsCartesianPointBlocked;
            public Vector2[] JointLimits;
            public Bounds Workspace;
            public Vector3[] ObstacleBounds;
            public Vector3 GridOrigin;
            public float GridCellSize;
            public float Clearance;
            public float ExtensionStepDegrees = 20f;
            public float CollisionResolutionDegrees = 4f;
            public int MaximumIterations = 4500;
            public int MaximumGridNodes = 20000;
            public int MaximumGridRouteAttempts = 8;
            public float GoalBias = 0.08f;
            public float ArcStepMeters = 0.04f;
            public System.Random Random = new System.Random(1);
        }

        public sealed class Result
        {
            public readonly List<float[]> JointPath = new List<float[]>();
            public int Iterations;
            public int RoutesTried;
            public Vector3 FailurePosition;
            public string FailureReason;
        }

        /// <summary>Immutable input snapshot for background-only occupancy search.</summary>
        public sealed class GridSearchSnapshot
        {
            public Bounds Workspace;
            public Bounds[] InflatedObstacles;
            public Vector3 StartPosition;
            public Vector3 GoalPosition;
            public float CellSize;
            public int MaximumNodes;
            public Vector3Int[] ForbiddenCells;
            public CancellationToken CancellationToken;
        }

        public sealed class GridSearchResult
        {
            public readonly List<Vector3> Points = new List<Vector3>();
            public int ExpandedNodes;
            public Vector3 FailurePosition;
            public string FailureReason;
            public bool Found => Points.Count > 1;
        }

        /// <summary>Runs occupancy search on copied value data; safe to call from a worker thread.</summary>
        public static GridSearchResult SearchGridSnapshot(GridSearchSnapshot snapshot) =>
            GridAStarPlanner.SearchSnapshot(snapshot);

        private readonly World world;

        public RobotBaseSixAxisPlannerAlgorithm(World world)
        {
            this.world = world ?? throw new ArgumentNullException(nameof(world));
        }

        /// <summary>Creates a standalone planner using this entry point's shared world callbacks.</summary>
        public RobotBaseSixAxisPlanner CreatePlanner(Kind kind)
        {
            switch (kind)
            {
                case Kind.RrtConnectFast: return new FastRrtConnectPlanner(world);
                case Kind.RrtConnectOptimized: return new OptimizedRrtConnectPlanner(world);
                case Kind.CartesianArc: return new CartesianArcPlanner(world);
                case Kind.GridAStar: return new GridAStarPlanner(world);
                case Kind.JointSpacePrm: return new JointSpacePrmPlanner(world);
                case Kind.InformedRrtStar: return new InformedRrtStarPlanner(world);
                default: throw new ArgumentOutOfRangeException(nameof(kind), kind, null);
            }
        }

        public bool TryPlan(Kind kind, float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, out Result result) =>
            CreatePlanner(kind).TryPlan(start, goal, startPosition, startRotation, goalPosition, goalRotation, out result);
    }
}
