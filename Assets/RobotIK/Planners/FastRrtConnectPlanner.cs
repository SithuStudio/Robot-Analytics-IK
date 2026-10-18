using System;
using System.Collections.Generic;
using UnityEngine;
using World = RobotIK.RobotBaseSixAxisPlannerAlgorithm.World;
using Result = RobotIK.RobotBaseSixAxisPlannerAlgorithm.Result;

namespace RobotIK
{
    /// <summary>RRT-Connect that stops at the first collision-free route.</summary>
    public sealed class FastRrtConnectPlanner : RrtConnectPlannerBase
    {
        public FastRrtConnectPlanner(World world) : base(world) { }

        protected override bool Plan(float[] start, float[] goal, Vector3 startPosition,
            Quaternion startRotation, Vector3 goalPosition, Quaternion goalRotation, Result result) =>
            TryPlanRrt(true, start, goal, result);
    }
}
