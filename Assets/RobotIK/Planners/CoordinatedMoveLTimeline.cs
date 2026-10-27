using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Shared arm/track clock. Staged transfers optionally use a continuous,
    /// acceleration-limited E1 profile across each uninterrupted track run.</summary>
    public sealed class CoordinatedMoveLTimeline
    {
        private readonly ArmTrackState[] states;
        private readonly Pose[] poses;
        private readonly double[] times;
        private readonly TrackRun[] trackRuns;
        private sealed class TrackRun
        {
            public int First, Last;
            public double Start, Duration, Distance, Speed, Acceleration, Ramp, StartSpeed, EndSpeed, Deceleration;
            public double At(double elapsed)
            {
                elapsed = Math.Max(0, Math.Min(Duration, elapsed));
                if (elapsed < Ramp) return StartSpeed * elapsed + .5 * Acceleration * elapsed * elapsed;
                if (elapsed <= Duration - Deceleration) return StartSpeed * Ramp + .5 * Acceleration * Ramp * Ramp + Speed * (elapsed - Ramp);
                double remaining = Duration - elapsed;
                return Distance - EndSpeed * remaining - .5 * Acceleration * remaining * remaining;
            }
        }
        public float DurationSeconds => (float)times[times.Length - 1];
        public CoordinatedMoveLTimeline(IReadOnlyList<ArmTrackState> states, IReadOnlyList<Pose> poses, float tcpMetersPerSecond, float degreesPerSecond, float trackMillimetersPerSecond,
            float trackAccelerationMillimetersPerSecondSquared = 0)
        {
            if (states == null || poses == null || states.Count < 2 || states.Count != poses.Count || !LinearTrackAxis.Finite(tcpMetersPerSecond) ||
                !LinearTrackAxis.Finite(degreesPerSecond) || !LinearTrackAxis.Finite(trackMillimetersPerSecond) || !LinearTrackAxis.Finite(trackAccelerationMillimetersPerSecondSquared) ||
                trackAccelerationMillimetersPerSecondSquared < 0 || tcpMetersPerSecond <= 0 || degreesPerSecond <= 0 || trackMillimetersPerSecond <= 0)
                throw new ArgumentException("Invalid coordinated timeline inputs.");
            this.states = new ArmTrackState[states.Count]; this.poses = new Pose[poses.Count]; times = new double[states.Count];
            for (int i = 0; i < states.Count; i++)
            {
                if (states[i] == null || !states[i].HasTrackAxis || !CoordinatedMoveLPlanner.FinitePose(poses[i])) throw new ArgumentException("Timeline requires finite poses and complete arm/E1 states.");
                this.states[i] = states[i]; this.poses[i] = poses[i];
                if (i == 0) continue;
                double duration = Math.Max(.0001, Math.Max(Vector3.Distance(poses[i - 1].position, poses[i].position) / tcpMetersPerSecond, Quaternion.Angle(poses[i - 1].rotation, poses[i].rotation) / degreesPerSecond));
                duration = Math.Max(duration, Math.Abs((double)states[i].E1Millimeters - states[i - 1].E1Millimeters) / trackMillimetersPerSecond);
                for (int j = 0; j < 6; j++) duration = Math.Max(duration, Math.Abs((double)states[i][j] - states[i - 1][j]) / degreesPerSecond);
                times[i] = times[i - 1] + duration;
            }
            trackRuns = new TrackRun[states.Count];
            if (trackAccelerationMillimetersPerSecondSquared > 0) RetimeTrack(trackAccelerationMillimetersPerSecondSquared);
        }
        private void RetimeTrack(double acceleration)
        {
            var durations = new double[times.Length];
            for (int i = 1; i < times.Length; i++) durations[i] = times[i] - times[i - 1];
            double clock = 0;
            for (int edge = 1; edge < states.Length;)
            {
                double delta = (double)states[edge].E1Millimeters - states[edge - 1].E1Millimeters;
                if (Math.Abs(delta) <= .00001)
                { clock += durations[edge]; times[edge] = clock; edge++; continue; }
                int first = edge - 1, last = edge;
                while (last + 1 < states.Length)
                {
                    double step = (double)states[last + 1].E1Millimeters - states[last].E1Millimeters;
                    if (Math.Abs(step) <= .00001 || Math.Sign(step) != Math.Sign(delta)) break;
                    last++;
                }
                // Keep a local cap for each edge. The former minimum over the
                // entire run spread a near-stationary arm adjustment's tiny E1
                // speed across meters of otherwise unrestricted rail travel.
                var distances = new double[last - first];
                var caps = new double[distances.Length];
                var velocities = new double[distances.Length + 1];
                for (int i = first + 1; i <= last; i++)
                {
                    int at = i - first - 1;
                    distances[at] = Math.Abs((double)states[i].E1Millimeters - states[i - 1].E1Millimeters);
                    caps[at] = distances[at] / durations[i];
                    if (at > 0) velocities[at] = Math.Min(caps[at - 1], caps[at]);
                }
                // Stops remain at true holds/reversals. Interior samples retain a
                // shared boundary velocity, bounded by both adjacent edge caps.
                for (int i = 1; i < velocities.Length - 1; i++)
                    velocities[i] = Math.Min(velocities[i], Math.Sqrt(velocities[i - 1] * velocities[i - 1] + 2 * acceleration * distances[i - 1]));
                for (int i = velocities.Length - 2; i > 0; i--)
                    velocities[i] = Math.Min(velocities[i], Math.Sqrt(velocities[i + 1] * velocities[i + 1] + 2 * acceleration * distances[i]));
                for (int i = first + 1; i <= last; i++)
                {
                    int at = i - first - 1;
                    double distance = distances[at], v0 = velocities[at], v1 = velocities[at + 1];
                    double peak = Math.Min(caps[at], Math.Sqrt(acceleration * distance + .5 * (v0 * v0 + v1 * v1)));
                    double ramp = Math.Max(0, (peak - v0) / acceleration), decel = Math.Max(0, (peak - v1) / acceleration);
                    double cruise = Math.Max(0, distance - (v0 + peak) * ramp * .5 - (v1 + peak) * decel * .5);
                    var run = new TrackRun { First = i - 1, Last = i, Start = clock, Distance = distance, Speed = peak, Acceleration = acceleration,
                        Ramp = ramp, Deceleration = decel, StartSpeed = v0, EndSpeed = v1, Duration = ramp + cruise / peak + decel };
                    trackRuns[i] = run; clock += run.Duration; times[i] = clock;
                }
                edge = last + 1;
            }
        }
        public ArmTrackState Evaluate(float seconds, out Pose expected)
            => Evaluate(seconds, out expected, out _);
        public ArmTrackState Evaluate(float seconds, out Pose expected, out int upperIndex)
        {
            if (seconds <= 0) { upperIndex = 0; expected = poses[0]; return states[0]; }
            if (seconds >= DurationSeconds) { upperIndex = states.Length - 1; expected = poses[poses.Length - 1]; return states[states.Length - 1]; }
            int lo = 0, hi = times.Length - 1;
            while (hi - lo > 1) { int mid = (lo + hi) / 2; if (times[mid] <= seconds) lo = mid; else hi = mid; }
            float t = (float)((seconds - times[lo]) / (times[hi] - times[lo]));
            var run = trackRuns[hi];
            if (run != null)
            {
                double distance = run.At(seconds - run.Start);
                double before = Math.Abs((double)states[lo].E1Millimeters - states[run.First].E1Millimeters);
                t = Mathf.Clamp01((float)((distance - before) / Math.Abs((double)states[hi].E1Millimeters - states[lo].E1Millimeters)));
            }
            upperIndex = hi;
            expected = new Pose(Vector3.Lerp(poses[lo].position, poses[hi].position, t), Quaternion.Slerp(poses[lo].rotation, poses[hi].rotation, t));
            return CoordinatedMoveLPlanner.Interpolate(states[lo], states[hi], t);
        }
    }
}
