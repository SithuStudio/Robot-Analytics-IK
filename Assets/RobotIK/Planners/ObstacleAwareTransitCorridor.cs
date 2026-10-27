using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>TCP guide candidates only. A guide is never executable until arm/E1 IK,
    /// swept collision, clearance, singularity and branch checks validate every edge.</summary>
    public sealed class ObstacleAwareTransitCorridor
    {
        public string Name;
        public int BoundsIntersections;
        public float Length;
        public readonly List<Pose> Samples = new List<Pose>();
        public readonly List<Pose> ProgramAnchors = new List<Pose>();
        private readonly List<float> distances = new List<float>();
        private float travelBegin, travelEnd;
        public float TrackFraction(Pose pose)
        {
            float best = float.PositiveInfinity, at = 0;
            for (int i = 1; i < Samples.Count; i++)
            {
                var edge = Samples[i].position - Samples[i - 1].position;
                float t = edge.sqrMagnitude < 1e-12f ? 0 : Mathf.Clamp01(Vector3.Dot(pose.position - Samples[i - 1].position, edge) / edge.sqrMagnitude);
                float d = (pose.position - (Samples[i - 1].position + edge * t)).sqrMagnitude;
                if (d < best) { best = d; at = Mathf.Lerp(distances[i - 1], distances[i], t); }
            }
            return Mathf.Clamp01((at - travelBegin) / Mathf.Max(.0001f, travelEnd - travelBegin));
        }
        private static ObstacleAwareTransitCorridor Make(string name, Pose start, Pose end, Vector3 a, Vector3 b, float blend, float tilt)
        {
            var result = new ObstacleAwareTransitCorridor { Name = name };
            void Add(Vector3 p, bool anchor = false)
            {
                if (result.Samples.Count > 0 && (result.Samples[result.Samples.Count - 1].position - p).sqrMagnitude < 1e-12f) return;
                result.Samples.Add(new Pose(p, Quaternion.identity));
                if (anchor) result.ProgramAnchors.Add(new Pose(p, Quaternion.identity));
            }
            Add(start.position, true);
            void Corner(Vector3 previous, Vector3 corner, Vector3 next, bool first)
            {
                float trim = Mathf.Min(blend, Vector3.Distance(previous, corner) * .4f, Vector3.Distance(corner, next) * .4f);
                var entry = Vector3.MoveTowards(corner, previous, trim); var exit = Vector3.MoveTowards(corner, next, trim);
                Add(entry, true);
                if (first) result.travelBegin = result.Samples.Count - 1;
                for (int i = 1; i <= 8; i++) { float t = i / 8f; Add(Vector3.Lerp(Vector3.Lerp(entry, corner, t), Vector3.Lerp(corner, exit, t), t), i == 4 || i == 8); }
                if (!first) result.travelEnd = result.Samples.Count - 1;
            }
            Corner(start.position, a, b, true); Add((a + b) * .5f, true); Corner(a, b, end.position, false); Add(end.position, true);
            float length = 0; result.distances.Add(0);
            for (int i = 1; i < result.Samples.Count; i++) { length += Vector3.Distance(result.Samples[i - 1].position, result.Samples[i].position); result.distances.Add(length); }
            result.Length = length;
            result.travelBegin = result.distances[Mathf.Clamp((int)result.travelBegin, 0, result.distances.Count - 1)];
            result.travelEnd = result.distances[Mathf.Clamp((int)result.travelEnd, 0, result.distances.Count - 1)];
            Quaternion Rotation(Vector3 p)
            {
                float f = result.TrackFraction(new Pose(p, Quaternion.identity));
                return Quaternion.AngleAxis(tilt * Mathf.Sin(Mathf.PI * f), Vector3.up) * Quaternion.Slerp(start.rotation, end.rotation, f);
            }
            for (int i = 0; i < result.Samples.Count; i++) result.Samples[i] = new Pose(result.Samples[i].position, Rotation(result.Samples[i].position));
            for (int i = 0; i < result.ProgramAnchors.Count; i++) result.ProgramAnchors[i] = new Pose(result.ProgramAnchors[i].position, Rotation(result.ProgramAnchors[i].position));
            result.Samples[0] = start; result.Samples[result.Samples.Count - 1] = end;
            return result;
        }
        public static List<ObstacleAwareTransitCorridor> Build(Pose start, Pose end, IReadOnlyList<Bounds> obstacles, Vector3 trackDirection,
            float clearanceMeters, float dropMillimeters, float accuracyMillimeters, bool rotationRetry = false)
        {
            if (!CoordinatedMoveLPlanner.FinitePose(start) || !CoordinatedMoveLPlanner.FinitePose(end) ||
                !LinearTrackAxis.Finite(clearanceMeters) || clearanceMeters < 0 || !LinearTrackAxis.Finite(dropMillimeters) || dropMillimeters < 10 ||
                !LinearTrackAxis.Finite(accuracyMillimeters) || accuracyMillimeters < 0) throw new ArgumentException("Invalid obstacle-aware corridor inputs.");
            var forward = Vector3.ProjectOnPlane(trackDirection, Vector3.up).normalized;
            if (forward.sqrMagnitude < .5f) forward = Vector3.ProjectOnPlane(end.position - start.position, Vector3.up).normalized;
            if (forward.sqrMagnitude < .5f) forward = Vector3.forward;
            var side = Vector3.Cross(Vector3.up, forward).normalized;
            float margin = clearanceMeters + .05f, drop = dropMillimeters * .001f;
            var envelope = new Bounds(start.position, Vector3.zero); envelope.Encapsulate(end.position);
            envelope.Encapsulate(start.position - Vector3.up * drop); envelope.Encapsulate(end.position - Vector3.up * drop); envelope.Expand(2 * (margin + .2f));
            var relevant = new List<Bounds>();
            float high = Mathf.Max(start.position.y, end.position.y) + drop, low = Mathf.Min(start.position.y, end.position.y) - drop;
            float left = margin + .15f, right = left; var middle = (start.position + end.position) * .5f;
            foreach (var bound in obstacles ?? Array.Empty<Bounds>())
            {
                // Endpoint-containing bounds can enclose empty space (car shells).
                // They remain in full collision validation, not solid TCP guide geometry.
                if (!bound.Intersects(envelope) || bound.Contains(start.position) || bound.Contains(end.position)) continue;
                relevant.Add(bound); high = Mathf.Max(high, bound.max.y + margin); low = Mathf.Min(low, bound.min.y - margin);
                float center = Vector3.Dot(bound.center - middle, side);
                float radius = Mathf.Abs(side.x) * bound.extents.x + Mathf.Abs(side.y) * bound.extents.y + Mathf.Abs(side.z) * bound.extents.z;
                left = Mathf.Max(left, radius - center + margin); right = Mathf.Max(right, radius + center + margin);
            }
            var output = new List<ObstacleAwareTransitCorridor>();
            void Lane(string name, float height, float offset, float tilt = 0)
            {
                var a = new Vector3(start.position.x, height, start.position.z) + side * offset;
                var b = new Vector3(end.position.x, height, end.position.z) + side * offset;
                var candidate = Make(name, start, end, a, b, accuracyMillimeters * .001f, tilt);
                foreach (var obstacle in relevant)
                {
                    var inflated = obstacle; inflated.Expand(clearanceMeters * 2);
                    for (int i = 1; i < candidate.Samples.Count; i++)
                    {
                        var delta = candidate.Samples[i].position - candidate.Samples[i - 1].position;
                        if (inflated.Contains(candidate.Samples[i - 1].position) || delta.magnitude > 1e-8f && inflated.IntersectRay(new Ray(candidate.Samples[i - 1].position, delta.normalized), out float hit) && hit <= delta.magnitude)
                        { candidate.BoundsIntersections++; break; }
                    }
                }
                output.Add(candidate);
            }
            Lane("Lower corridor", Mathf.Min(start.position.y, end.position.y) - drop, 0);
            Lane("Outward lower left", Mathf.Min(start.position.y, end.position.y) - drop, -(margin + .15f));
            Lane("Outward lower right", Mathf.Min(start.position.y, end.position.y) - drop, margin + .15f);
            Lane("Above obstacles", high, 0); Lane("Below obstacles", low, 0);
            Lane("Track left detour", Mathf.Min(start.position.y, end.position.y), -left);
            Lane("Track right detour", Mathf.Min(start.position.y, end.position.y), right);
            Lane("Raised left detour", high, -left); Lane("Raised right detour", high, right);
            if (rotationRetry) { Lane("Above +5 degree intermediate retry", high, 0, 5); Lane("Above -5 degree intermediate retry", high, 0, -5); }
            // Bounds rank guides; even intersecting guides remain candidates. Bounds
            // cannot prove a mesh route invalid, nor can a clear TCP prove arm safety.
            int Priority(string name) => name == "Lower corridor" ? 0 : name == "Below obstacles" ? 1 : name.StartsWith("Outward lower") ? 2 : 3;
            output.Sort((a, b) =>
            {
                int compare = a.BoundsIntersections.CompareTo(b.BoundsIntersections);
                if (compare != 0) return compare;
                compare = Priority(a.Name).CompareTo(Priority(b.Name));
                return compare != 0 ? compare : a.Length.CompareTo(b.Length);
            });
            return output;
        }
        public static double Cost(IReadOnlyList<ArmTrackState> states, IReadOnlyList<Pose> poses, float peakStretch = 0, float knownMargin = float.NaN, float preferredStretch = .84f)
        {
            double tcp = 0, joints = 0, track = 0, turns = 0;
            for (int i = 1; i < states.Count; i++)
            {
                tcp += Vector3.Distance(poses[i - 1].position, poses[i].position);
                track += Math.Abs(states[i].E1Millimeters - states[i - 1].E1Millimeters) * .001;
                for (int j = 0; j < 6; j++) joints += Math.Abs(states[i][j] - states[i - 1][j]) * (j >= 3 ? 1.5 : 1);
                if (i > 1)
                {
                    var a = poses[i - 1].position - poses[i - 2].position; var b = poses[i].position - poses[i - 1].position;
                    if (a.sqrMagnitude > 1e-10f && b.sqrMagnitude > 1e-10f) { double turn = Vector3.Angle(a, b) / 180; turns += turn * turn; }
                }
            }
            double marginBonus = LinearTrackAxis.Finite(knownMargin) ? Math.Min(.1, Math.Max(0, knownMargin)) : 0;
            return tcp + .002 * joints + .1 * track + .1 * turns + Math.Max(0, peakStretch - preferredStretch) * 4 - marginBonus;
        }
    }
}
