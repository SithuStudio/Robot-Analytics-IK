using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Shared bidirectional tree expansion for the two RRT-Connect variants.</summary>
    public abstract class RrtConnectPlannerBase : RobotBaseSixAxisPlanner
    {
        protected RrtConnectPlannerBase(World world) : base(world) { }

        protected bool TryPlanRrt(bool firstSolution, float[] start, float[] goal, Result result)
        {
            if (world.IsJointSegmentFree(start, goal))
            {
                result.JointPath.Add((float[])start.Clone());
                result.JointPath.Add((float[])goal.Clone());
                return true;
            }

            var treeA = new List<Node> { new Node((float[])start.Clone(), -1) };
            var treeB = new List<Node> { new Node((float[])goal.Clone(), -1) };
            bool aFromStart = true;
            int firstConnection = -1;
            int budget = Math.Max(1, world.MaximumIterations);
            // Find a feasible route first, then spend a bounded part of the budget
            // improving it. A larger maximum must not force a long refinement wait.
            int optimizationBudget = firstSolution ? 0 : Math.Min(128, Math.Max(16, budget / 4));
            List<float[]> best = null;
            float bestCost = float.PositiveInfinity;
            int blockedExtensions = 0;

            for (int iteration = 0; iteration < budget; iteration++)
            {
                world.CancellationToken.ThrowIfCancellationRequested();
                if (firstConnection >= 0 && iteration - firstConnection > optimizationBudget) break;
                result.Iterations++;
                world.IterationProgress?.Invoke(result.Iterations);
                float[] sample = world.Random != null && world.Random.NextDouble() < world.GoalBias
                    ? treeB[0].Angles : GetRandomConfiguration();
                // Wrist preference ranks solutions; before a first optimized route
                // exists, ordinary joint distance keeps exploration from starving.
                bool weightedNearest = firstSolution || best != null;
                int nearA = FindNearest(treeA, sample, weightedNearest);
                if (TryExtend(treeA, nearA, sample, out int addedA))
                {
                    float[] target = treeA[addedA].Angles;
                    int nearB = FindNearest(treeB, target, weightedNearest);
                    bool connected = false;
                    while (TryExtend(treeB, nearB, target, out int next))
                    {
                        nearB = next;
                        if (JointDistance(treeB[nearB].Angles, target) < 0.01f) { connected = true; break; }
                    }
                    if (connected)
                    {
                        List<float[]> candidate = JoinTrees(treeA, addedA, treeB, nearB, aFromStart);
                        float cost = PathCost(candidate);
                        if (cost < bestCost) { best = candidate; bestCost = cost; }
                        if (firstConnection < 0) firstConnection = iteration;
                    }
                    else blockedExtensions++;
                }
                else blockedExtensions++;
                SwapTrees(ref treeA, ref treeB);
                aFromStart = !aFromStart;
                if (best != null && firstSolution) break;
            }
            if (best == null)
            {
                int startNodes = aFromStart ? treeA.Count : treeB.Count;
                int goalNodes = aFromStart ? treeB.Count : treeA.Count;
                result.FailureReason = $"RRT-Connect exhausted its iteration budget without a connection. " +
                    $"Start tree: {startNodes} nodes; goal tree: {goalNodes} nodes; " +
                    $"{blockedExtensions} iterations were blocked by collision/clearance checks.";
                return false;
            }
            result.JointPath.AddRange(best);
            return true;
        }

        private bool TryExtend(List<Node> tree, int fromIndex, float[] target, out int added)
        {
            float[] from = tree[fromIndex].Angles;
            // Extension Step Degrees is a geometric joint-angle step. Using the
            // weighted ranking metric here shrinks it whenever wrist weights rise.
            float distance = JointDistance(from, target);
            added = fromIndex;
            if (distance < 0.001f) return true;
            float t = Math.Min(1f, world.ExtensionStepDegrees / distance);
            var next = new float[from.Length];
            for (int i = 0; i < next.Length; i++) next[i] = from[i] + (target[i] - from[i]) * t;
            if (!world.IsJointSegmentFree(from, next)) return false;
            added = tree.Count; tree.Add(new Node(next, fromIndex)); return true;
        }

        private int FindNearest(List<Node> tree, float[] target, bool weighted)
        {
            int best = 0;
            float distance = weighted ? SearchDistance(tree[0].Angles, target) : JointDistance(tree[0].Angles, target);
            for (int i = 1; i < tree.Count; i++)
            {
                float d = weighted ? SearchDistance(tree[i].Angles, target) : JointDistance(tree[i].Angles, target);
                if (d < distance) { best = i; distance = d; }
            }
            return best;
        }

        private float SearchDistance(float[] a, float[] b) => world.MotionCost(a, b);

        private static List<float[]> JoinTrees(List<Node> a, int ai, List<Node> b, int bi, bool aStartsAtStart)
        {
            List<float[]> first = Trace(a, ai), second = Trace(b, bi);
            var result = new List<float[]>(first.Count + second.Count - 1);
            if (aStartsAtStart) { first.Reverse(); result.AddRange(first); for (int i = 1; i < second.Count; i++) result.Add(second[i]); }
            else { second.Reverse(); result.AddRange(second); result.AddRange(first); }
            return result;
        }

        private static List<float[]> Trace(List<Node> tree, int index)
        { var result = new List<float[]>(); while (index >= 0) { result.Add(tree[index].Angles); index = tree[index].Parent; } return result; }

        private static void SwapTrees(ref List<Node> a, ref List<Node> b) { List<Node> tmp = a; a = b; b = tmp; }

        private sealed class Node
        { public readonly float[] Angles; public readonly int Parent; public Node(float[] angles, int parent) { Angles = angles; Parent = parent; } }
    }
}
