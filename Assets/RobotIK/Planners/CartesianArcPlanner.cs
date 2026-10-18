using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>Cartesian TCP arc sampling with IK and swept robot collision validation.</summary>
    public sealed class CartesianArcPlanner : RobotBaseSixAxisPlanner
    {
        public CartesianArcPlanner(World world) : base(world) { }

        protected override bool Plan(float[] start, float[] goal, Vector3 a, Quaternion ar,
            Vector3 b, Quaternion br, Result result)
        {
            Vector3 delta = b - a;
            Vector3 side = Vector3.Cross(delta.normalized, Vector3.up);
            if (side.sqrMagnitude < 1e-5f) side = Vector3.Cross(delta.normalized, Vector3.right);
            side.Normalize();
            Vector3[] directions = { Vector3.up, -Vector3.up, side, -side, Vector3.right, -Vector3.right,
                Vector3.forward, -Vector3.forward };
            float extent = 0.1f;
            if (world.ObstacleBounds != null)
                for (int i = 0; i < world.ObstacleBounds.Length; i++) extent = Mathf.Max(extent, world.ObstacleBounds[i].magnitude + world.Clearance);
            float[] heights = { 0f, 0.15f, 0.3f, extent * 2f, extent * 3f };
            int samples = Mathf.Clamp(Mathf.CeilToInt(delta.magnitude / Mathf.Max(0.005f, world.ArcStepMeters)), 12, 64);
            List<float[]> best = null;
            float bestCost = float.PositiveInfinity;
            for (int h = 0; h < heights.Length; h++)
            for (int d = 0; d < (h == 0 ? 1 : directions.Length); d++)
            {
                result.RoutesTried++;
                Vector3 control = (a + b) * 0.5f + directions[d] * heights[h];
                var candidate = new List<float[]> { (float[])start.Clone() };
                float[] previous = start;
                bool valid = true;
                for (int i = 1; i <= samples; i++)
                {
                    float t = i / (float)samples;
                    Vector3 p = i == samples ? b : (1f-t)*(1f-t)*a + 2f*(1f-t)*t*control + t*t*b;
                    Quaternion q = Quaternion.Slerp(ar, br, t);
                    List<float[]> solutions = i == samples ? new List<float[]> { (float[])goal.Clone() } : world.SolveIk(p, q);
                    if (solutions == null || solutions.Count == 0) { valid = false; break; }
                    solutions.Sort((x, y) => world.MotionCost(previous, x).CompareTo(world.MotionCost(previous, y)));
                    float[] selected = null;
                    for (int k = 0; k < solutions.Count; k++)
                        if (!world.IsConfigurationInCollision(solutions[k]) && world.IsJointSegmentFree(previous, solutions[k]))
                        { selected = solutions[k]; break; }
                    if (selected == null) { valid = false; break; }
                    candidate.Add((float[])selected.Clone());
                    previous = selected;
                }
                if (!valid) continue;
                float cost = PathCost(candidate);
                if (cost < bestCost) { best = candidate; bestCost = cost; }
            }
            if (best == null) { result.FailureReason = "No collision-free Cartesian TCP arc was found."; return false; }
            result.JointPath.AddRange(best);
            return true;
        }
    }
}
