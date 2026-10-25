using System;
using UnityEngine;

namespace RobotIK
{
    [Serializable]
    public struct TrackPoseSample
    {
        public string Name;
        public float E1Millimeters;
        public Vector3 PositionMillimeters;
        [Tooltip("Absolute carriage orientation in TrackRoot coordinates; Unity Euler convention (Z, X, Y).")]
        public Vector3 RotationDegrees;

        public TrackPoseSample(string name, float e1, Vector3 position, Vector3 rotation)
        { Name = name; E1Millimeters = e1; PositionMillimeters = position; RotationDegrees = rotation; }
    }

    [Serializable]
    public struct TrackCorrectionChannels
    {
        public bool X, Y, Z, Rx, Ry, Rz;
        public static TrackCorrectionChannels PositionOnly => new TrackCorrectionChannels { X = true, Y = true, Z = true };
        public bool SameAs(TrackCorrectionChannels b) => X == b.X && Y == b.Y && Z == b.Z && Rx == b.Rx && Ry == b.Ry && Rz == b.Rz;
    }

    [Serializable]
    public sealed class TrackPoseProfile
    {
        public TrackPoseSample Home = new TrackPoseSample("Home", 0, Vector3.zero, Vector3.zero);
        public TrackPoseSample[] Intermediate = Array.Empty<TrackPoseSample>();
        public TrackPoseSample End = new TrackPoseSample("End", 1000, new Vector3(1000, 0, 0), Vector3.zero);
        public TrackCorrectionChannels Corrections = TrackCorrectionChannels.PositionOnly;

        public TrackPoseProfile Copy() => new TrackPoseProfile
        {
            Home = Home, End = End, Corrections = Corrections,
            Intermediate = Intermediate == null ? null : (TrackPoseSample[])Intermediate.Clone()
        };

        public bool SameAs(TrackPoseProfile b)
        {
            if (b == null || !Same(Home, b.Home) || !Same(End, b.End) || !Corrections.SameAs(b.Corrections)) return false;
            int count = Intermediate?.Length ?? 0;
            if (count != (b.Intermediate?.Length ?? 0)) return false;
            for (int i = 0; i < count; i++) if (!Same(Intermediate[i], b.Intermediate[i])) return false;
            return true;
        }

        private static bool Same(TrackPoseSample a, TrackPoseSample b) => a.Name == b.Name && a.E1Millimeters.Equals(b.E1Millimeters) &&
            a.PositionMillimeters.Equals(b.PositionMillimeters) && a.RotationDegrees.Equals(b.RotationDegrees);
    }

    /// <summary>
    /// Immutable, scene-independent evaluator for one E1 coordinate mapped to a calibrated
    /// carriage pose. Input positions are mm; output Pose uses Unity meters. No actuator motion.
    /// </summary>
    public sealed class LinearTrackAxis
    {
        private readonly TrackPoseProfile profile;
        private readonly TrackPoseSample[] samples;
        public float MinimumMillimeters => profile.Home.E1Millimeters;
        public float MaximumMillimeters => profile.End.E1Millimeters;

        public LinearTrackAxis(TrackPoseProfile source)
        {
            if (!ValidateProfile(source, out string error)) throw new ArgumentException(error, nameof(source));
            profile = source.Copy();
            int count = profile.Intermediate?.Length ?? 0;
            samples = new TrackPoseSample[count + 2];
            samples[0] = profile.Home;
            if (count > 0) Array.Copy(profile.Intermediate, 0, samples, 1, count);
            samples[samples.Length - 1] = profile.End;
        }

        public static bool ValidateProfile(TrackPoseProfile source, out string error)
        {
            error = null;
            if (source == null) { error = "Track pose profile is missing."; return false; }
            if (!ValidSample(source.Home) || !ValidSample(source.End))
            { error = "Track Home/End coordinates and poses must be finite."; return false; }
            if (!(source.End.E1Millimeters > source.Home.E1Millimeters) ||
                !Finite(source.End.E1Millimeters - source.Home.E1Millimeters))
            { error = "Track End E1 must be greater than Home E1."; return false; }
            float previous = source.Home.E1Millimeters;
            if (source.Intermediate != null)
                for (int i = 0; i < source.Intermediate.Length; i++)
                {
                    var sample = source.Intermediate[i];
                    if (!ValidSample(sample)) { error = $"Track calibration station {i + 1} contains non-finite data."; return false; }
                    if (!(sample.E1Millimeters > previous && sample.E1Millimeters < source.End.E1Millimeters))
                    { error = "Intermediate track E1 coordinates must be strictly ordered between Home and End; duplicates are invalid."; return false; }
                    previous = sample.E1Millimeters;
                }
            return true;
        }

        public bool TryEvaluate(float e1Millimeters, out Pose pose, out string error)
        {
            pose = default; error = null;
            if (!Finite(e1Millimeters) || e1Millimeters < MinimumMillimeters || e1Millimeters > MaximumMillimeters)
            { error = $"E1 must be within {MinimumMillimeters:F2}–{MaximumMillimeters:F2} mm."; return false; }
            int low = 0, high = samples.Length - 1;
            while (high - low > 1)
            {
                int mid = (low + high) / 2;
                if (samples[mid].E1Millimeters <= e1Millimeters) low = mid; else high = mid;
            }
            var a = samples[low]; var b = samples[high];
            float t = (e1Millimeters - a.E1Millimeters) / (b.E1Millimeters - a.E1Millimeters);
            float nominalT = (e1Millimeters - MinimumMillimeters) / (MaximumMillimeters - MinimumMillimeters);
            Vector3 nominal = Vector3.Lerp(profile.Home.PositionMillimeters, profile.End.PositionMillimeters, nominalT);
            Vector3 measured = Vector3.Lerp(a.PositionMillimeters, b.PositionMillimeters, t);
            var c = profile.Corrections;
            Vector3 p = new Vector3(c.X ? measured.x : nominal.x, c.Y ? measured.y : nominal.y, c.Z ? measured.z : nominal.z);
            Quaternion q = Quaternion.Slerp(Orientation(a), Orientation(b), t);
            if (!Finite(p) || !Finite(new Vector3(q.x, q.y, q.z)) || !Finite(q.w))
            { error = "Evaluated track pose is not finite."; return false; }
            pose = new Pose(p * 0.001f, q);
            return true;
        }

        private Quaternion Orientation(TrackPoseSample sample)
        {
            Quaternion home = Quaternion.Euler(profile.Home.RotationDegrees);
            var c = profile.Corrections;
            if (!c.Rx && !c.Ry && !c.Rz) return home;
            Quaternion measured = Quaternion.Euler(sample.RotationDegrees);
            if (c.Rx && c.Ry && c.Rz) return measured;
            // Partial orientation channels are corrections about TrackRoot axes,
            // relative to Home. Define the Euler convention once, then slerp poses.
            Vector3 delta = (measured * Quaternion.Inverse(home)).eulerAngles;
            return Quaternion.Euler(c.Rx ? Mathf.DeltaAngle(0, delta.x) : 0,
                c.Ry ? Mathf.DeltaAngle(0, delta.y) : 0,
                c.Rz ? Mathf.DeltaAngle(0, delta.z) : 0) * home;
        }

        private static bool ValidSample(TrackPoseSample s) => Finite(s.E1Millimeters) && Finite(s.PositionMillimeters) && Finite(s.RotationDegrees);
        internal static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        internal static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
    }
}
