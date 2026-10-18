using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Joint-space probabilistic roadmap construction, Dijkstra search, and safe shortcutting.</summary>
    public sealed class JointSpacePrmPlanner : RobotBaseSixAxisPlanner
    {
        public JointSpacePrmPlanner(World world) : base(world) { }

        protected override bool Plan(float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, Result result)
        {
            if (world.IsJointSegmentFree(start, goal))
            {
                result.JointPath.Add((float[])start.Clone());
                result.JointPath.Add((float[])goal.Clone());
                return true;
            }

            // A bounded roadmap keeps planning responsive; MaximumIterations is the
            // sampling budget, capped because every edge requires swept robot checks.
            int sampleBudget = Math.Min(Math.Max(1, world.MaximumIterations), 600);
            const int neighborCount = 8;
            var nodes = new List<float[]>(sampleBudget + 2) { (float[])start.Clone() };
            var edges = new List<List<RoadmapEdge>>(sampleBudget + 2) { new List<RoadmapEdge>() };
            for (int iteration = 0; iteration < sampleBudget; iteration++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                result.Iterations = iteration + 1;
                world.IterationProgress?.Invoke(result.Iterations);
                float[] sample = GetRandomConfiguration();
                if (world.IsConfigurationInCollision(sample)) continue;
                int newIndex = nodes.Count;
                nodes.Add(sample);
                edges.Add(new List<RoadmapEdge>());
                ConnectRoadmapNode(newIndex, nodes, edges, neighborCount);
            }

            int goalIndex = nodes.Count;
            nodes.Add((float[])goal.Clone());
            edges.Add(new List<RoadmapEdge>());
            ConnectRoadmapNode(goalIndex, nodes, edges, neighborCount);
            if (edges[goalIndex].Count == 0)
            {
                result.FailureReason = $"Joint-space PRM built a roadmap with {nodes.Count - 2} sampled configurations, but the goal did not connect to a collision-free neighbor. Increase Maximum Iterations or change the obstacle layout.";
                return false;
            }

            var costs = new float[nodes.Count];
            var parents = new int[nodes.Count];
            var visited = new bool[nodes.Count];
            for (int i = 0; i < costs.Length; i++) { costs[i] = float.PositiveInfinity; parents[i] = -1; }
            costs[0] = 0f;
            for (int step = 0; step < nodes.Count; step++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                int current = -1;
                float bestCost = float.PositiveInfinity;
                for (int i = 0; i < nodes.Count; i++)
                    if (!visited[i] && costs[i] < bestCost) { current = i; bestCost = costs[i]; }
                if (current < 0 || current == goalIndex) break;
                visited[current] = true;
                for (int e = 0; e < edges[current].Count; e++)
                {
                    RoadmapEdge edge = edges[current][e];
                    float candidate = bestCost + edge.Cost;
                    if (candidate >= costs[edge.To]) continue;
                    costs[edge.To] = candidate;
                    parents[edge.To] = current;
                }
            }

            if (parents[goalIndex] < 0)
            {
                result.FailureReason = $"Joint-space PRM expanded {nodes.Count} roadmap nodes, but the sampled graph did not connect Start to Goal. Increase Maximum Iterations or try another Random Seed.";
                return false;
            }
            var reverse = new List<int>();
            for (int cursor = goalIndex; cursor >= 0; cursor = parents[cursor])
            {
                reverse.Add(cursor);
                if (cursor == 0) break;
            }
            reverse.Reverse();
            var route = new List<float[]>(reverse.Count);
            for (int i = 0; i < reverse.Count; i++) route.Add(nodes[reverse[i]]);
            // Shortcut only through edges that pass the same full joint sweep check.
            for (int anchor = 0; anchor < route.Count - 2;)
            {
                int furthest = anchor + 1;
                for (int candidate = route.Count - 1; candidate > anchor + 1; candidate--)
                    if (world.IsJointSegmentFree(route[anchor], route[candidate])) { furthest = candidate; break; }
                if (furthest > anchor + 1) route.RemoveRange(anchor + 1, furthest - anchor - 1);
                else anchor++;
            }
            result.JointPath.AddRange(route);
            return true;
        }

        private void ConnectRoadmapNode(int nodeIndex, List<float[]> nodes,
            List<List<RoadmapEdge>> edges, int neighborCount)
        {
            List<int> nearest = FindNearestConfigurationIndices(nodes, nodes[nodeIndex], neighborCount, nodeIndex);
            for (int i = 0; i < nearest.Count; i++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                int other = nearest[i];
                if (!world.IsJointSegmentFree(nodes[other], nodes[nodeIndex])) continue;
                float cost = JointDistance(nodes[other], nodes[nodeIndex]);
                edges[other].Add(new RoadmapEdge(nodeIndex, cost));
                edges[nodeIndex].Add(new RoadmapEdge(other, cost));
            }
        }

        private static List<int> FindNearestConfigurationIndices(List<float[]> nodes,
            float[] target, int count, int exclusiveUpperBound)
        {
            var nearest = new List<int>(count);
            var distances = new List<float>(count);
            for (int i = 0; i < exclusiveUpperBound; i++)
            {
                float distance = JointDistance(nodes[i], target);
                int insert = distances.Count;
                while (insert > 0 && distance < distances[insert - 1]) insert--;
                if (insert >= count) continue;
                nearest.Insert(insert, i);
                distances.Insert(insert, distance);
                if (nearest.Count > count) { nearest.RemoveAt(count); distances.RemoveAt(count); }
            }
            return nearest;
        }

        private readonly struct RoadmapEdge
        {
            public readonly int To;
            public readonly float Cost;
            public RoadmapEdge(int to, float cost) { To = to; Cost = cost; }
        }
    }
}
