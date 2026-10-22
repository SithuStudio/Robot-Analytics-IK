using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using UnityEngine;

namespace RobotIK
{
    internal static class PlanningQueryUtilities
    {
        /// <summary>Keep a bounded request window open while a worker wakes and posts
        /// its next query. Exiting as soon as the queue is empty forces sequential
        /// request/response workloads to pay a complete Unity frame for every query.</summary>
        public static int Drain(ConcurrentQueue<Action> requests, double milliseconds, Func<bool> workerActive)
        {
            var timer = Stopwatch.StartNew();
            int executed = 0;
            while (timer.Elapsed.TotalMilliseconds < milliseconds)
            {
                if (requests.TryDequeue(out Action request)) { request(); executed++; }
                else if (executed > 0 && workerActive != null && workerActive()) Thread.Yield();
                else break;
            }
            return executed;
        }

        /// <summary>A conservative broad phase: if inflated AABBs do not overlap,
        /// their actual colliders cannot penetrate or violate the clearance margin.</summary>
        public static bool CouldViolateClearance(Bounds robot, Bounds obstacle, float clearance)
        {
            robot.Expand(2f * (Math.Max(0f, clearance) + 0.00001f));
            return robot.Intersects(obstacle);
        }
    }

    internal static class PathDisplayWaypoints
    {
        // Display simplification only. No execution or collision samples are removed.
        public static int[] Select(IReadOnlyList<Vector3> positions, IReadOnlyList<Quaternion> rotations,
            IReadOnlyList<Vector3> required = null)
        {
            if (positions.Count == 0) return Array.Empty<int>();
            var selected = new SortedSet<int> { 0, positions.Count - 1 };
            if (required != null)
                foreach (Vector3 marker in required)
                {
                    int nearest = 0;
                    for (int i = 1; i < positions.Count; i++)
                        if ((positions[i] - marker).sqrMagnitude < (positions[nearest] - marker).sqrMagnitude) nearest = i;
                    if ((positions[nearest] - marker).sqrMagnitude < 0.000004f) selected.Add(nearest);
                }
            var anchors = new List<int>(selected);
            var pending = new Stack<(int First, int Last)>();
            for (int i = 1; i < anchors.Count; i++) pending.Push((anchors[i - 1], anchors[i]));
            while (pending.Count > 0)
            {
                var span = pending.Pop();
                Vector3 delta = positions[span.Last] - positions[span.First];
                float worst = 1f; int split = -1;
                for (int i = span.First + 1; i < span.Last; i++)
                {
                    float t = delta.sqrMagnitude < 1e-10f ? (i - span.First) / (float)(span.Last - span.First) :
                        Mathf.Clamp01(Vector3.Dot(positions[i] - positions[span.First], delta) / delta.sqrMagnitude);
                    float error = Vector3.Distance(positions[i], Vector3.Lerp(positions[span.First], positions[span.Last], t)) / 0.001f;
                    if (rotations != null && rotations.Count == positions.Count)
                        error = Mathf.Max(error, Quaternion.Angle(rotations[i], Quaternion.Slerp(rotations[span.First], rotations[span.Last], t)) / 0.5f);
                    if (error > worst) { worst = error; split = i; }
                }
                if (split < 0) continue;
                selected.Add(split); pending.Push((span.First, split)); pending.Push((split, span.Last));
            }
            var result = new int[selected.Count]; selected.CopyTo(result); return result;
        }
    }
}
