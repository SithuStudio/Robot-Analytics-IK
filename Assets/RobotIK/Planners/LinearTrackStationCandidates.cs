using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Bounded deterministic sampling for a fixed-E1 search. Does not solve IK or move a scene.</summary>
    public static class LinearTrackStationCandidates
    {
        public static IReadOnlyList<float> Build(TrackPoseProfile profile, float current, float spacingMillimeters, int maximumCandidates)
        {
            if (!LinearTrackAxis.ValidateProfile(profile, out string error)) throw new ArgumentException(error, nameof(profile));
            if (!LinearTrackAxis.Finite(current) || !LinearTrackAxis.Finite(spacingMillimeters) || spacingMillimeters <= 0)
                throw new ArgumentException("Current E1 and positive station spacing must be finite.");
            int capacity = Math.Max(1, Math.Min(128, maximumCandidates));
            float min = profile.Home.E1Millimeters, max = profile.End.E1Millimeters;
            var values = new List<float>(capacity);
            void Add(float e1) { if (values.Count < capacity && e1 >= min && e1 <= max && !values.Contains(e1)) values.Add(e1); }
            Add(Mathf.Clamp(current, min, max)); Add(min); Add(max);
            if (profile.Intermediate != null)
            {
                var samples = (TrackPoseSample[])profile.Intermediate.Clone();
                Array.Sort(samples, (a, b) => Compare(a.E1Millimeters, b.E1Millimeters, current));
                foreach (var sample in samples) Add(sample.E1Millimeters);
            }
            double span = (double)max - min;
            double divisions = Math.Ceiling(span / spacingMillimeters);
            if (divisions <= capacity)
            {
                for (int i = 1; i < divisions; i++) Add((float)(min + i * span / divisions));
            }
            else
            {
                // Do not loop over a potentially enormous physical range at tiny spacing.
                int remaining = capacity - values.Count;
                for (int i = 1; i <= remaining; i++) Add((float)(min + i * span / (remaining + 1)));
            }
            values.Sort((a, b) => Compare(a, b, current));
            return values.AsReadOnly();
        }
        private static int Compare(float a, float b, float current)
        { int order = Math.Abs((double)a - current).CompareTo(Math.Abs((double)b - current)); return order != 0 ? order : a.CompareTo(b); }
    }
}
