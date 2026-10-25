using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Program waypoints over a densely validated trajectory. Dense samples
    /// remain authoritative for collision validation, interpolation and timing.</summary>
    public static class TrackTransferWaypoints
    {
        public static int[] SelectBeforeOptimization(IReadOnlyList<ArmTrackState> states, IReadOnlyList<Pose> poses, IReadOnlyList<int> phases, int target)
        {
            if (states == null || poses == null || phases == null || states.Count != poses.Count || states.Count != phases.Count) throw new ArgumentException("Waypoint inputs must align.");
            int count = states.Count; if (count == 0) return Array.Empty<int>();
            var keep = new SortedSet<int> { 0, count - 1 };
            for (int i = 1; i < count; i++) if (phases[i] != phases[i - 1]) { keep.Add(i - 1); keep.Add(i); }
            var boundaries = new List<int>(keep); var pending = new Stack<Vector2Int>();
            for (int i = 1; i < boundaries.Count; i++) pending.Push(new Vector2Int(boundaries[i - 1], boundaries[i]));
            while (pending.Count > 0)
            {
                var pair = pending.Pop(); int worst = -1; double largest = 1;
                for (int i = pair.x + 1; i < pair.y; i++)
                {
                    double numerator = 0, denominator = 0;
                    for (int axis = 0; axis < 7; axis++)
                    {
                        double a = Coordinate(states[pair.x], axis), d = Coordinate(states[pair.y], axis) - a;
                        numerator += (Coordinate(states[i], axis) - a) * d; denominator += d * d;
                    }
                    float t = denominator > 1e-20 ? Mathf.Clamp01((float)(numerator / denominator)) : (i - pair.x) / (float)(pair.y - pair.x);
                    double error = 0;
                    for (int axis = 0; axis < 7; axis++)
                        error = Math.Max(error, Math.Abs(Coordinate(states[i], axis) - (Coordinate(states[pair.x], axis) * (1 - t) + Coordinate(states[pair.y], axis) * t)) / (axis == 6 ? .001 : .0001));
                    // Retain tool-path corners and orientation changes as well as joint-space bends.
                    error = Math.Max(error, Vector3.Distance(poses[i].position, Vector3.Lerp(poses[pair.x].position, poses[pair.y].position, t)) / .00001);
                    error = Math.Max(error, Quaternion.Angle(poses[i].rotation, Quaternion.Slerp(poses[pair.x].rotation, poses[pair.y].rotation, t)) / .01);
                    if (error > largest) { largest = error; worst = i; }
                }
                if (worst >= 0) { keep.Add(worst); pending.Push(new Vector2Int(pair.x, worst)); pending.Push(new Vector2Int(worst, pair.y)); }
            }
            target = Mathf.Clamp(target, 2, count);
            // Split the largest remaining index gap until the requested density is met.
            while (keep.Count < target)
            {
                int before = -1, selected = -1, gap = 1;
                foreach (int index in keep) { if (before >= 0 && index - before > gap) { gap = index - before; selected = before + gap / 2; } before = index; }
                if (selected < 0) break; keep.Add(selected);
            }
            var output = new int[keep.Count]; keep.CopyTo(output); return output;
        }
        /// <summary>Preferred maximum, never padded. Try tighter fits first and relax
        /// only up to the configured transfer tolerances. Final MoveL is represented
        /// by Cartesian segments, with its dense IK trajectory retained unchanged.</summary>
        public static int[] Select(IReadOnlyList<ArmTrackState> states, IReadOnlyList<Pose> poses, IReadOnlyList<int> phases, int target,
            float positionMillimeters = 1, float rotationDegrees = .1f)
        {
            if (states == null || poses == null || phases == null || states.Count != poses.Count || states.Count != phases.Count ||
                !LinearTrackAxis.Finite(positionMillimeters) || !LinearTrackAxis.Finite(rotationDegrees) || positionMillimeters <= 0 || rotationDegrees <= 0)
                throw new ArgumentException("Invalid waypoint reduction inputs.");
            int[] selected = Array.Empty<int>();
            for (int level = 0; level < 5; level++)
            {
                float scale = 1f / (1 << (4 - level));
                var keep = new SortedSet<int>(); int count = states.Count;
                if (count == 0) return selected;
                keep.Add(0); keep.Add(count - 1);
                for (int i = 1; i < count; i++) if (phases[i] != phases[i - 1]) { keep.Add(i - 1); keep.Add(i); }
                var bounds = new List<int>(keep); var pending = new Stack<Vector2Int>();
                for (int i = 1; i < bounds.Count; i++) pending.Push(new Vector2Int(bounds[i - 1], bounds[i]));
                while (pending.Count > 0)
                {
                    var pair = pending.Pop(); int worst = -1; double largest = 1;
                    bool cartesian = phases[pair.x] == 4 && phases[pair.y] == 4;
                    for (int i = pair.x + 1; i < pair.y; i++)
                    {
                        var delta = poses[pair.y].position - poses[pair.x].position;
                        float t;
                        if (cartesian)
                        {
                            float angle = Quaternion.Angle(poses[pair.x].rotation, poses[pair.y].rotation);
                            t = delta.sqrMagnitude > 1e-12f ? Mathf.Clamp01(Vector3.Dot(poses[i].position - poses[pair.x].position, delta) / delta.sqrMagnitude) :
                                angle > .001f ? Mathf.Clamp01(Quaternion.Angle(poses[pair.x].rotation, poses[i].rotation) / angle) : (i - pair.x) / (float)(pair.y - pair.x);
                        }
                        else
                        {
                            double dot = 0, length = 0;
                            for (int axis = 0; axis < 7; axis++) { double d = Coordinate(states[pair.y], axis) - Coordinate(states[pair.x], axis); dot += (Coordinate(states[i], axis) - Coordinate(states[pair.x], axis)) * d; length += d * d; }
                            t = length > 1e-20 ? Mathf.Clamp01((float)(dot / length)) : (i - pair.x) / (float)(pair.y - pair.x);
                        }
                        double error = 0;
                        if (!cartesian) for (int axis = 0; axis < 7; axis++)
                            error = Math.Max(error, Math.Abs(Coordinate(states[i], axis) - (Coordinate(states[pair.x], axis) * (1 - t) + Coordinate(states[pair.y], axis) * t)) / (axis == 6 ? positionMillimeters * scale : .1 * scale));
                        else error = Math.Abs(states[i].E1Millimeters - states[pair.x].E1Millimeters) / .001;
                        error = Math.Max(error, Vector3.Distance(poses[i].position, Vector3.Lerp(poses[pair.x].position, poses[pair.y].position, t)) / (cartesian ? .00001 : positionMillimeters * scale * .001));
                        error = Math.Max(error, Quaternion.Angle(poses[i].rotation, Quaternion.Slerp(poses[pair.x].rotation, poses[pair.y].rotation, t)) / (cartesian ? .01 : rotationDegrees * scale));
                        if (error > largest) { largest = error; worst = i; }
                    }
                    if (worst >= 0) { keep.Add(worst); pending.Push(new Vector2Int(pair.x, worst)); pending.Push(new Vector2Int(worst, pair.y)); }
                }
                selected = new int[keep.Count]; keep.CopyTo(selected);
                if (selected.Length <= Math.Max(2, target)) break;
            }
            return selected;
        }
        private static double Coordinate(ArmTrackState state, int axis) => axis == 6 ? state.E1Millimeters : state[axis];
    }
}
