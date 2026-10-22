using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Reusable joint-path smoothing and synchronized TCP timing, independent of a rig or planner.
    /// Create once, then call Evaluate with elapsed seconds. Units are millimeters and degrees.
    /// FK and validation callbacks run synchronously while creating the profile. Call on Unity's
    /// main thread if callbacks use Transforms or Physics, and restore any rig pose they change.
    /// Evaluating a completed profile does not call the callbacks or modify its input path.
    /// </summary>
    public sealed class MotionProfile
    {
        /// <summary>Settings are copied on creation; changing them does not change a built profile.</summary>
        [Serializable]
        public sealed class Settings
        {
            public bool EnableCornerSmoothing = true;
            public float LinearSpeed = 500f;
            public float RotationSpeedDegree = 75f;
            public float TcpAccelerationMillimetersPerSecondSquared = 200f;
            public float TcpDecelerationMillimetersPerSecondSquared = 200f;
            public float TcpRotationAccelerationDegreesPerSecondSquared = 150f;
            public float TcpRotationDecelerationDegreesPerSecondSquared = 150f;
            public float PathAccuracyMillimeters = 20f;
            public float CollisionResolutionDegrees = 10f;

            internal Settings Copy() => (Settings)MemberwiseClone();
        }

        private readonly Settings settings;
        private readonly Func<float[], Pose> evaluateTcp;
        private readonly Func<float[], bool> isConfigurationValid;
        private readonly List<float[]> path = new List<float[]>();
        private readonly List<ExecutionGroup> executionGroups = new List<ExecutionGroup>();

        public float DurationSeconds => executionGroups.Count == 0 ? 0f : executionGroups[0].Profile.Duration;
        public int SmoothedPathCorners { get; private set; }
        public int RetainedSharpCorners { get; private set; }
        public int JointCount => path[0].Length;

        /// <summary>
        /// Builds a profile for any positive joint count. evaluateTcp returns the world TCP pose
        /// for joint angles in degrees. Optional validation returns true for safe configurations;
        /// without it, smoothing checks only the accuracy distance, not collisions.
        /// Start/end poses and the original joint path are preserved. Speed ramps occur only
        /// at start/end; angular speed can limit translation during simultaneous rotation.
        /// </summary>
        public static MotionProfile Create(IReadOnlyList<float[]> jointPath, Func<float[], Pose> evaluateTcp,
            Settings settings = null, Func<float[], bool> isConfigurationValid = null)
        {
            var profile = new MotionProfile(jointPath, evaluateTcp, settings ?? new Settings(), isConfigurationValid);
            profile.BuildExecutionGroups();
            return profile;
        }

        private MotionProfile(IReadOnlyList<float[]> jointPath, Func<float[], Pose> evaluateTcp,
            Settings settings, Func<float[], bool> isConfigurationValid)
        {
            if (jointPath == null) throw new ArgumentNullException(nameof(jointPath));
            if (evaluateTcp == null) throw new ArgumentNullException(nameof(evaluateTcp));
            if (jointPath.Count == 0 || jointPath[0] == null || jointPath[0].Length == 0)
                throw new ArgumentException("Provide at least one joint state with a positive joint count.", nameof(jointPath));
            foreach (float[] angles in jointPath)
            {
                if (angles == null || angles.Length != jointPath[0].Length)
                    throw new ArgumentException("All joint states must have the same joint count.", nameof(jointPath));
                foreach (float angle in angles)
                    if (float.IsNaN(angle) || float.IsInfinity(angle))
                        throw new ArgumentException("Joint angles must be finite.", nameof(jointPath));
                path.Add((float[])angles.Clone());
            }
            this.settings = settings.Copy();
            foreach (float value in new[]
            {
                settings.LinearSpeed, settings.RotationSpeedDegree,
                settings.TcpAccelerationMillimetersPerSecondSquared, settings.TcpDecelerationMillimetersPerSecondSquared,
                settings.TcpRotationAccelerationDegreesPerSecondSquared, settings.TcpRotationDecelerationDegreesPerSecondSquared,
                settings.PathAccuracyMillimeters, settings.CollisionResolutionDegrees
            })
                if (float.IsNaN(value) || float.IsInfinity(value) || value <= 0f)
                    throw new ArgumentException("Motion settings must be positive finite values.", nameof(settings));
            this.evaluateTcp = evaluateTcp;
            this.isConfigurationValid = isConfigurationValid;
        }

        /// <summary>Returns a new joint-angle array for elapsed time in seconds, clamped to the endpoints.</summary>
        public float[] Evaluate(float elapsedSeconds)
        {
            if (float.IsNaN(elapsedSeconds)) throw new ArgumentOutOfRangeException(nameof(elapsedSeconds));
            return executionGroups.Count == 0 ? (float[])path[0].Clone() :
                executionGroups[0].Evaluate(executionGroups[0].Profile.Evaluate(elapsedSeconds));
        }

        /// <summary>Returns joint angles at normalized geometric path progress, independent of timing.</summary>
        public float[] EvaluateProgress(float progress)
        {
            if (float.IsNaN(progress)) throw new ArgumentOutOfRangeException(nameof(progress));
            return executionGroups.Count == 0 ? (float[])path[0].Clone() : executionGroups[0].Evaluate(progress);
        }

        /// <summary>Samples the same smoothed geometry used by Evaluate, for path previews.</summary>
        public IEnumerable<float[]> SampleJointPath()
        {
            if (executionGroups.Count == 0) { yield return (float[])path[0].Clone(); yield break; }
            ExecutionGroup group = executionGroups[0];
            foreach (float progress in group.SampleProgress()) yield return group.Evaluate(progress);
        }

        private static float Distance(float[] a, float[] b)
        {
            float squared = 0f;
            for (int i = 0; i < a.Length; i++) squared += (a[i] - b[i]) * (a[i] - b[i]);
            return Mathf.Sqrt(squared);
        }

        private static float[] Interpolate(float[] a, float[] b, float t)
        {
            var result = new float[a.Length];
            for (int i = 0; i < result.Length; i++) result[i] = Mathf.Lerp(a[i], b[i], t);
            return result;
        }

        // Straight pieces and quadratic corner curves share a joint-distance
        // parameter so their tangents match without adding intermediate stops.
        private sealed class ExecutionPiece
        {
            public float[] From, Control, To;
            public float Length;
            public float[] Evaluate(float t) => Control == null ? Interpolate(From, To, t) :
                Interpolate(Interpolate(From, Control, t), Interpolate(Control, To, t), t);
        }

        private sealed class ExecutionGroup
        {
            public readonly List<ExecutionPiece> Pieces = new List<ExecutionPiece>();
            private readonly List<float> pieceEnds = new List<float>();
            public float Length;
            public TcpMotionProfile Profile;

            public void Add(ExecutionPiece piece)
            {
                if (piece.Length <= 0.00001f) return;
                Pieces.Add(piece);
                Length += piece.Length;
                pieceEnds.Add(Length);
            }

            public float[] Evaluate(float progress)
            {
                float position = Mathf.Clamp01(progress) * Length;
                int low = 0, high = Pieces.Count - 1;
                while (low < high)
                {
                    int middle = (low + high) / 2;
                    if (position <= pieceEnds[middle]) high = middle;
                    else low = middle + 1;
                }
                float offset = low == 0 ? 0f : pieceEnds[low - 1];
                return Pieces[low].Evaluate(Mathf.Clamp01((position - offset) / Pieces[low].Length));
            }

            public List<float> SampleProgress()
            {
                var samples = new List<float> { 0f };
                float offset = 0f;
                foreach (ExecutionPiece piece in Pieces)
                {
                    int count = Mathf.Max(8, Mathf.CeilToInt(piece.Length / 0.5f));
                    for (int i = 1; i <= count; i++)
                    {
                        float progress = (offset + piece.Length * (i / (float)count)) / Length;
                        if (progress > samples[samples.Count - 1]) samples.Add(progress);
                    }
                    offset += piece.Length;
                }
                samples[samples.Count - 1] = 1f;
                return samples;
            }
        }

        private void BuildExecutionGroups()
        {
            executionGroups.Clear();
            SmoothedPathCorners = RetainedSharpCorners = 0;
            var points = new List<float[]>();
            foreach (float[] waypoint in path)
                if (points.Count == 0 || Distance(points[points.Count - 1], waypoint) > 0.0001f)
                    points.Add(waypoint);
            if (points.Count < 2) return;
            var corners = new ExecutionPiece[points.Count];
            for (int i = 1; i < points.Count - 1; i++)
            {
                if (IsStraightPathCorner(points[i - 1], points[i], points[i + 1])) continue;
                if (!settings.EnableCornerSmoothing) { RetainedSharpCorners++; continue; }
                corners[i] = TrySmoothPathCorner(points[i - 1], points[i], points[i + 1]);
                if (corners[i] != null) SmoothedPathCorners++;
                else RetainedSharpCorners++;
            }
            var group = new ExecutionGroup();
            for (int i = 1; i < points.Count; i++)
            {
                float[] from = corners[i - 1] != null ? corners[i - 1].To : points[i - 1];
                float[] to = corners[i] != null ? corners[i].From : points[i];
                group.Add(new ExecutionPiece { From = from, To = to, Length = Distance(from, to) });
                if (corners[i] != null) group.Add(corners[i]);
            }
            if (group.Pieces.Count == 0) return;
            group.Profile = BuildTcpMotionProfile(group);
            executionGroups.Add(group);
        
        }

        private static bool IsStraightPathCorner(float[] before, float[] corner, float[] after)
        {
            float dot = 0f;
            for (int j = 0; j < corner.Length; j++)
                dot += (corner[j] - before[j]) * (after[j] - corner[j]);
            return dot / (Distance(before, corner) * Distance(corner, after)) > 0.999999f;
        }

        private ExecutionPiece TrySmoothPathCorner(float[] before, float[] corner, float[] after)
        {
            float incoming = Distance(before, corner), outgoing = Distance(corner, after);
            if (incoming < 0.001f || outgoing < 0.001f) return null;
            // A reversal has no nonzero common tangent; retain the original path.
            float dot = 0f;
            for (int j = 0; j < corner.Length; j++)
                dot += (corner[j] - before[j]) * (after[j] - corner[j]);
            if (dot / (incoming * outgoing) < -0.999f) return null;
            Vector3 center = evaluateTcp(corner).position;
            float accuracyMeters = Mathf.Max(1f, settings.PathAccuracyMillimeters) * 0.001f;
            float trim = 0.45f * Mathf.Min(incoming, outgoing);
            float rejectedTrim = trim;
            for (int attempt = 0; attempt < 16 && trim > 0.0001f; attempt++, trim *= 0.5f)
            {
                ExecutionPiece curve = CreateAccuracyCurve(before, corner, after, incoming, outgoing, trim);
                if (!IsAccuracyCurveValid(curve, center, accuracyMeters))
                { rejectedTrim = trim; continue; }
                // Use as much of the requested distance as safely fits, instead of
                // leaving the curve at an arbitrarily halved corner size.
                for (int refinement = 0; refinement < 8 && rejectedTrim - trim > 0.0001f; refinement++)
                {
                    float candidateTrim = 0.5f * (trim + rejectedTrim);
                    ExecutionPiece candidate = CreateAccuracyCurve(before, corner, after, incoming, outgoing, candidateTrim);
                    if (IsAccuracyCurveValid(candidate, center, accuracyMeters))
                    { trim = candidateTrim; curve = candidate; }
                    else rejectedTrim = candidateTrim;
                }
                return curve;
            }
            return null;
        }

        private static ExecutionPiece CreateAccuracyCurve(float[] before, float[] corner, float[] after,
            float incoming, float outgoing, float trim) => new ExecutionPiece
        {
            From = Interpolate(corner, before, trim / incoming),
            Control = corner,
            To = Interpolate(corner, after, trim / outgoing),
            Length = 2f * trim
        };

        private bool IsAccuracyCurveValid(ExecutionPiece curve, Vector3 center, float accuracyMeters)
        {
            int count = Mathf.Max(16, Mathf.CeilToInt(curve.Length / Mathf.Min(0.5f, Mathf.Max(0.01f, settings.CollisionResolutionDegrees))));
            for (int i = 0; i <= count; i++)
            {
                float[] angles = curve.Evaluate(i / (float)count);
                if (Vector3.Distance(evaluateTcp(angles).position, center) > accuracyMeters ||
                    (isConfigurationValid != null && !isConfigurationValid(angles))) return false;
            }
            return true;
        }

        // One synchronized feed profile for the complete route. Intermediate samples
        // map nominal travel time back to joint progress; they never trigger braking.
        private struct TcpMotionProfile
        {
            public float Duration, AccelerationTime, DecelerationTime, PeakSpeed;
            public float[] Progress;
            public double[] Travel;

            public float Evaluate(float time)
            {
                if (time <= 0f) return 0f;
                if (time >= Duration) return 1f;
                double distance;
                if (time < AccelerationTime)
                    distance = 0.5 * PeakSpeed * time * time / AccelerationTime;
                else if (time > Duration - DecelerationTime)
                {
                    double remaining = Duration - time;
                    distance = Travel[Travel.Length - 1] - 0.5 * PeakSpeed * remaining * remaining / DecelerationTime;
                }
                else distance = PeakSpeed * (time - 0.5 * AccelerationTime);
                int low = 0, high = Travel.Length - 1;
                while (high - low > 1)
                {
                    int middle = (low + high) / 2;
                    if (Travel[middle] <= distance) low = middle;
                    else high = middle;
                }
                float fraction = (float)((distance - Travel[low]) / (Travel[high] - Travel[low]));
                return Mathf.Lerp(Progress[low], Progress[high], Mathf.Clamp01(fraction));
            }
        }

        private TcpMotionProfile BuildTcpMotionProfile(ExecutionGroup group)
        {
            float[] samples = group.SampleProgress().ToArray();
            int intervals = samples.Length - 1;
            // Coordinate is nominal travel time at the requested TCP speeds, not
            // joint distance or a whole-route curvature-limited cruise speed.
            var travel = new double[samples.Length];
            var linearRates = new float[intervals];
            var angularRates = new float[intervals];
            float linearSpeed = Mathf.Max(1f, settings.LinearSpeed);
            float angularSpeed = Mathf.Max(1f, settings.RotationSpeedDegree);
            Vector3 previousPosition = Vector3.zero;
            Quaternion previousRotation = Quaternion.identity;
            for (int sample = 0; sample < samples.Length; sample++)
            {
                Pose tcp = evaluateTcp(group.Evaluate(samples[sample]));
                if (sample > 0)
                {
                    int i = sample - 1;
                    float translation = Vector3.Distance(tcp.position, previousPosition) * 1000f;
                    Quaternion delta = tcp.rotation * Quaternion.Inverse(previousRotation);
                    if (delta.w < 0f) delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);
                    float sine = new Vector3(delta.x, delta.y, delta.z).magnitude;
                    float rotation = 2f * Mathf.Atan2(sine, delta.w) * Mathf.Rad2Deg;
                    float seconds = Mathf.Max(0.000001f, Mathf.Max(translation / linearSpeed, rotation / angularSpeed));
                    travel[sample] = travel[i] + seconds;
                    linearRates[i] = translation / seconds;
                    angularRates[i] = rotation / seconds;
                }
                previousPosition = tcp.position;
                previousRotation = tcp.rotation;
            }
        
            return BuildCruiseMotionProfile(samples, travel, linearRates, angularRates);
        }

        private TcpMotionProfile BuildCruiseMotionProfile(float[] samples, double[] travel,
            float[] linearRates, float[] angularRates)
        {
            float maximumLinearRate = 0.0001f, maximumAngularRate = 0.0001f;
            for (int i = 0; i < linearRates.Length; i++)
            {
                maximumLinearRate = Mathf.Max(maximumLinearRate, linearRates[i]);
                maximumAngularRate = Mathf.Max(maximumAngularRate, angularRates[i]);
            }
            // These controls shape only the start/end speed envelope. There is no
            // corner-curvature penalty or braking event at intermediate waypoints.
            // Unrounded corners change direction abruptly; this is kinematic playback,
            // not a guarantee of bounded vector acceleration through a sharp corner.
            float acceleration = Mathf.Min(Mathf.Max(1f, settings.TcpAccelerationMillimetersPerSecondSquared) / maximumLinearRate,
                Mathf.Max(1f, settings.TcpRotationAccelerationDegreesPerSecondSquared) / maximumAngularRate);
            float deceleration = Mathf.Min(Mathf.Max(1f, settings.TcpDecelerationMillimetersPerSecondSquared) / maximumLinearRate,
                Mathf.Max(1f, settings.TcpRotationDecelerationDegreesPerSecondSquared) / maximumAngularRate);
            float length = (float)travel[travel.Length - 1];
            float peak = Mathf.Min(1f, Mathf.Sqrt(2f * length / (1f / acceleration + 1f / deceleration)));
            float accelerationTime = peak / acceleration;
            float decelerationTime = peak / deceleration;
            float cruiseDistance = Mathf.Max(0f, length - 0.5f * peak * (accelerationTime + decelerationTime));
            return new TcpMotionProfile
            {
                Duration = accelerationTime + cruiseDistance / peak + decelerationTime,
                AccelerationTime = accelerationTime, DecelerationTime = decelerationTime,
                PeakSpeed = peak, Progress = samples, Travel = travel
            };
        }



    }
}

