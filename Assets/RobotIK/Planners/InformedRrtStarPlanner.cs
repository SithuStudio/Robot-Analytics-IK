using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Joint-space RRT* parent selection, rewiring, and informed ellipsoid sampling.</summary>
    public sealed class InformedRrtStarPlanner : RobotBaseSixAxisPlanner
    {
        public InformedRrtStarPlanner(World world) : base(world) { }

        protected override bool Plan(float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, Result result)
        {
            if (world.IsJointSegmentFree(start, goal))
            {
                result.JointPath.Add((float[])start.Clone());
                result.JointPath.Add((float[])goal.Clone());
                return true;
            }

            var nodes = new List<StarNode> { new StarNode((float[])start.Clone(), -1, 0f) };
            int bestGoalParent = -1;
            float bestGoalCost = float.PositiveInfinity;
            for (int iteration = 0; iteration < Math.Max(1, world.MaximumIterations); iteration++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                result.Iterations = iteration + 1;
                world.IterationProgress?.Invoke(result.Iterations);
                float[] sample = bestGoalParent >= 0
                    ? SampleInformedConfiguration(start, goal, bestGoalCost)
                    : world.Random.NextDouble() < world.GoalBias
                        ? (float[])goal.Clone() : GetRandomConfiguration();
                int nearest = FindNearestStar(nodes, sample);
                float[] candidate = Steer(nodes[nearest].Angles, sample, world.ExtensionStepDegrees);
                if (JointDistance(nodes[nearest].Angles, candidate) < 0.001f ||
                    world.IsConfigurationInCollision(candidate) ||
                    !world.IsJointSegmentFree(nodes[nearest].Angles, candidate)) continue;

                int neighborhoodCount = Math.Min(nodes.Count,
                    Math.Max(6, Math.Min(12, (int)Math.Ceiling(2.0 * Math.Log(nodes.Count + 1.0)))));
                List<int> neighbors = FindNearestStarIndices(nodes, candidate, neighborhoodCount);
                int parent = nearest;
                float parentCost = nodes[nearest].Cost + JointDistance(nodes[nearest].Angles, candidate);
                for (int i = 0; i < neighbors.Count; i++)
                {
                    int neighbor = neighbors[i];
                    float cost = nodes[neighbor].Cost + JointDistance(nodes[neighbor].Angles, candidate);
                    if (cost >= parentCost || !world.IsJointSegmentFree(nodes[neighbor].Angles, candidate)) continue;
                    parent = neighbor;
                    parentCost = cost;
                }

                int added = nodes.Count;
                nodes.Add(new StarNode(candidate, parent, parentCost));
                nodes[parent].Children.Add(added);
                for (int i = 0; i < neighbors.Count; i++)
                {
                    int neighbor = neighbors[i];
                    if (neighbor == parent || IsAncestor(nodes, neighbor, added)) continue;
                    float rewiredCost = parentCost + JointDistance(candidate, nodes[neighbor].Angles);
                    if (rewiredCost + 1e-4f >= nodes[neighbor].Cost ||
                        !world.IsJointSegmentFree(candidate, nodes[neighbor].Angles)) continue;
                    int oldParent = nodes[neighbor].Parent;
                    if (oldParent >= 0) nodes[oldParent].Children.Remove(neighbor);
                    nodes[neighbor].Parent = added;
                    nodes[added].Children.Add(neighbor);
                    UpdateStarSubtreeCosts(nodes, neighbor, rewiredCost);
                }
                if (bestGoalParent >= 0)
                    bestGoalCost = nodes[bestGoalParent].Cost + JointDistance(nodes[bestGoalParent].Angles, goal);

                float toGoal = JointDistance(candidate, goal);
                if (toGoal <= world.ExtensionStepDegrees * 1.5f &&
                    nodes[added].Cost + toGoal < bestGoalCost && world.IsJointSegmentFree(candidate, goal))
                {
                    bestGoalParent = added;
                    bestGoalCost = nodes[added].Cost + toGoal;
                }
            }

            if (bestGoalParent < 0)
            {
                result.FailureReason = "Informed RRT* exhausted its iteration budget without finding a collision-free route. Increase Maximum Iterations or change the obstacle layout.";
                return false;
            }
            var path = new List<float[]>();
            for (int cursor = bestGoalParent; cursor >= 0; cursor = nodes[cursor].Parent)
                path.Add(nodes[cursor].Angles);
            path.Reverse();
            path.Add((float[])goal.Clone());
            result.JointPath.AddRange(path);
            return true;
        }

        private float[] SampleInformedConfiguration(float[] start, float[] goal, float bestCost)
        {
            int n = start.Length;
            float directCost = JointDistance(start, goal);
            if (bestCost <= directCost + 1e-4f) return GetRandomConfiguration();
            var sphere = new double[n];
            double norm = 0.0;
            for (int i = 0; i < n; i++)
            {
                double u1 = Math.Max(1e-12, world.Random.NextDouble());
                double u2 = world.Random.NextDouble();
                sphere[i] = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
                norm += sphere[i] * sphere[i];
            }
            norm = Math.Sqrt(norm);
            double radius = Math.Pow(world.Random.NextDouble(), 1.0 / n) / Math.Max(norm, 1e-12);
            for (int i = 0; i < n; i++) sphere[i] *= radius;
            double major = bestCost * 0.5;
            double minor = Math.Sqrt(Math.Max(0.0, bestCost * bestCost - directCost * directCost)) * 0.5;
            for (int i = 0; i < n; i++) sphere[i] *= i == 0 ? major : minor;

            var direction = new double[n];
            for (int i = 0; i < n; i++) direction[i] = goal[i] - start[i];
            for (int i = 0; i < n; i++) direction[i] /= Math.Max(directCost, 1e-12);
            var reflection = new double[n];
            reflection[0] = 1.0 - direction[0];
            for (int i = 1; i < n; i++) reflection[i] = -direction[i];
            double reflectionNorm = 0.0;
            for (int i = 0; i < n; i++) reflectionNorm += reflection[i] * reflection[i];
            var sample = new float[n];
            for (int i = 0; i < n; i++)
            {
                double rotated = sphere[i];
                if (reflectionNorm > 1e-12)
                {
                    double dot = 0.0;
                    for (int j = 0; j < n; j++) dot += reflection[j] * sphere[j];
                    rotated -= 2.0 * reflection[i] * dot / reflectionNorm;
                }
                double value = (start[i] + goal[i]) * 0.5 + rotated;
                Vector2 limits = world.JointLimits != null && i < world.JointLimits.Length
                    ? world.JointLimits[i] : new Vector2(-180f, 180f);
                sample[i] = (float)Math.Max(limits.x, Math.Min(limits.y, value));
            }
            return sample;
        }

        private static float[] Steer(float[] from, float[] to, float step)
        {
            float distance = JointDistance(from, to);
            float t = distance <= step || distance <= 1e-6f ? 1f : step / distance;
            var result = new float[from.Length];
            for (int i = 0; i < result.Length; i++) result[i] = from[i] + (to[i] - from[i]) * t;
            return result;
        }

        private static int FindNearestStar(List<StarNode> nodes, float[] target)
        {
            int nearest = 0;
            float best = JointDistance(nodes[0].Angles, target);
            for (int i = 1; i < nodes.Count; i++)
            {
                float distance = JointDistance(nodes[i].Angles, target);
                if (distance < best) { best = distance; nearest = i; }
            }
            return nearest;
        }

        private static List<int> FindNearestStarIndices(List<StarNode> nodes, float[] target, int count)
        {
            var nearest = new List<int>(count);
            var distances = new List<float>(count);
            for (int i = 0; i < nodes.Count; i++)
            {
                float distance = JointDistance(nodes[i].Angles, target);
                int insert = distances.Count;
                while (insert > 0 && distance < distances[insert - 1]) insert--;
                if (insert >= count) continue;
                nearest.Insert(insert, i);
                distances.Insert(insert, distance);
                if (nearest.Count > count) { nearest.RemoveAt(count); distances.RemoveAt(count); }
            }
            return nearest;
        }

        private static bool IsAncestor(List<StarNode> nodes, int possibleAncestor, int node)
        {
            for (int cursor = node; cursor >= 0; cursor = nodes[cursor].Parent)
                if (cursor == possibleAncestor) return true;
            return false;
        }

        private static void UpdateStarSubtreeCosts(List<StarNode> nodes, int root, float rootCost)
        {
            nodes[root].Cost = rootCost;
            var pending = new Queue<int>();
            pending.Enqueue(root);
            while (pending.Count > 0)
            {
                int parent = pending.Dequeue();
                List<int> children = nodes[parent].Children;
                for (int i = 0; i < children.Count; i++)
                {
                    int child = children[i];
                    nodes[child].Cost = nodes[parent].Cost + JointDistance(nodes[parent].Angles, nodes[child].Angles);
                    pending.Enqueue(child);
                }
            }
        }

        private sealed class StarNode
        {
            public readonly float[] Angles;
            public readonly List<int> Children = new List<int>();
            public int Parent;
            public float Cost;
            public StarNode(float[] angles, int parent, float cost)
            { Angles = angles; Parent = parent; Cost = cost; }
        }
    }
}
