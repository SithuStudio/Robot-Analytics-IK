using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>RRT-Connect route ranking and collision-checked removal of unnecessary J6 motion.</summary>
    public sealed class OptimizedRrtConnectPlanner : RrtConnectPlannerBase
    {
        public OptimizedRrtConnectPlanner(World world) : base(world) { }

        protected override bool Plan(float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, Result result)
        {
            if (!TryPlanRrt(false, start, goal, result)) return false;
            if (world.Joint6TravelCostPerDegree > 0f)
            {
                List<float[]> reduced = ReduceJoint6Excursions(result.JointPath);
                if (!ReferenceEquals(reduced, result.JointPath))
                {
                    result.JointPath.Clear();
                    result.JointPath.AddRange(reduced);
                }
            }
            return true;
        }

        private List<float[]> ReduceJoint6Excursions(List<float[]> route)
        {
            if (route.Count < 3 || route[0].Length < 6) return route;
            var progress = new float[route.Count];
            for (int i = 1; i < route.Count; i++)
            {
                double squared = 0.0;
                for (int j = 0; j < 5; j++)
                {
                    double delta = route[i][j] - route[i - 1][j];
                    squared += delta * delta;
                }
                progress[i] = progress[i - 1] + (float)Math.Sqrt(squared);
            }
            float start = route[0][5], end = route[route.Count - 1][5];
            float bestCost = PathCost(route);
            // First try a steady roll between the required endpoint angles. If an
            // obstacle requires wrist roll, retain more of the original safe motion.
            float[] strengths = { 1f, 0.5f, 0.25f };
            for (int pass = 0; pass < strengths.Length; pass++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                var candidate = new List<float[]>(route.Count);
                for (int i = 0; i < route.Count; i++)
                {
                    var angles = (float[])route[i].Clone();
                    if (i > 0 && i < route.Count - 1)
                    {
                        float fraction = progress[progress.Length - 1] > 1e-6f
                            ? progress[i] / progress[progress.Length - 1] : i / (float)(route.Count - 1);
                        float desired = start + (end - start) * fraction;
                        angles[5] += (desired - angles[5]) * strengths[pass];
                    }
                    candidate.Add(angles);
                }
                bool safe = true;
                for (int i = 1; i < candidate.Count; i++)
                    if (!world.IsJointSegmentFree(candidate[i - 1], candidate[i])) { safe = false; break; }
                if (!safe) continue;
                float cost = PathCost(candidate);
                if (cost >= bestCost) continue;
                route = candidate;
                bestCost = cost;
                if (pass == 0) break;
            }
            return route;
        }
    }
}
