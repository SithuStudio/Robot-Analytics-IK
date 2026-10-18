using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Shared planning context, validation, sampling, and route costs. No Unity lifecycle dependencies.</summary>
    public abstract class RobotBaseSixAxisPlanner
    {
        protected readonly World world;

        protected RobotBaseSixAxisPlanner(World world)
        {
            this.world = world ?? throw new ArgumentNullException(nameof(world));
        }

        /// <summary>Plans between joint states and their TCP poses using the supplied world callbacks.</summary>
        public bool TryPlan(float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, out Result result)
        {
            ValidateWorld();
            result = new Result();
            return Plan(start, goal, startPosition, startRotation, goalPosition, goalRotation, result);
        }

        protected abstract bool Plan(float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, Result result);

        private void ValidateWorld()
        {
            if (world.IsJointSegmentFree == null || world.SolveIk == null ||
                world.GetTcpPosition == null || world.GetTcpRotation == null ||
                world.IsConfigurationInCollision == null || world.MotionCost == null)
                throw new InvalidOperationException("Planning world is missing IK, FK, or collision query callbacks.");
        }

        protected float[] GetRandomConfiguration()
        {
            if (world.RandomConfiguration != null) return world.RandomConfiguration();
            var result = new float[world.JointLimits.Length];
            for (int i = 0; i < result.Length; i++) result[i] = Mathf.Lerp(world.JointLimits[i].x, world.JointLimits[i].y, (float)world.Random.NextDouble());
            return result;
        }

        protected float PathCost(List<float[]> path)
        {
            float total = 0f;
            if (world.PrioritizeShortestPath && world.TcpPathCost != null)
                total = world.TcpPathCost(path);
            else
                for (int i = 1; i < path.Count; i++)
                    total += world.PrioritizeShortestPath
                        ? world.TcpTravelCost != null ? world.TcpTravelCost(path[i - 1], path[i]) : JointDistance(path[i - 1], path[i])
                        : world.RouteCost != null ? world.RouteCost(path[i - 1], path[i]) : world.MotionCost(path[i - 1], path[i]);
            if (world.PrioritizeShortestPath && world.Joint6TravelCostPerDegree > 0f)
                for (int i = 1; i < path.Count; i++)
                    if (path[i].Length >= 6 && path[i - 1].Length >= 6)
                        total += world.Joint6TravelCostPerDegree * Math.Abs(path[i][5] - path[i - 1][5]);
            return total;
        }

        protected static float JointDistance(float[] a, float[] b)
        { double sum = 0.0; for (int i = 0; i < a.Length; i++) { double d = a[i] - b[i]; sum += d * d; } return (float)Math.Sqrt(sum); }
    }
}
