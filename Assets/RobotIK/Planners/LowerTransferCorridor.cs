using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>World-down retract, level traverse and upward approach. Rounded
    /// shoulders are prescribed geometry, not an unvalidated display spline.</summary>
    public sealed class LowerTransferCorridor
    {
        public readonly List<Pose> Samples = new List<Pose>();
        public readonly List<Pose> ProgramAnchors = new List<Pose>();
        private readonly Pose start, arrival;
        private readonly Vector3 horizontal;
        private readonly float trackBegin, trackSpan;
        public float Fraction(Pose pose) => horizontal.sqrMagnitude < 1e-10f ? 0 : Mathf.Clamp01(Vector3.Dot(pose.position - start.position, horizontal) / horizontal.sqrMagnitude);
        public float TrackFraction(Pose pose) => trackSpan <= .0001f ? Fraction(pose) : Mathf.Clamp01((Fraction(pose) * horizontal.magnitude - trackBegin) / trackSpan);
        private Pose At(Vector3 position) => new Pose(position, Quaternion.Slerp(start.rotation, arrival.rotation, Fraction(new Pose(position, Quaternion.identity))));
        public LowerTransferCorridor(Pose start, Pose arrival, float dropMillimeters, float blendMillimeters)
        {
            if (!CoordinatedMoveLPlanner.FinitePose(start) || !CoordinatedMoveLPlanner.FinitePose(arrival) || !LinearTrackAxis.Finite(dropMillimeters) || dropMillimeters < 10 || !LinearTrackAxis.Finite(blendMillimeters) || blendMillimeters < 0) throw new ArgumentException("Invalid lower corridor geometry.");
            this.start = start; this.arrival = arrival; horizontal = arrival.position - start.position; horizontal.y = 0;
            float height = Mathf.Min(start.position.y, arrival.position.y) - dropMillimeters * .001f;
            var a = new Vector3(start.position.x, height, start.position.z); var b = new Vector3(arrival.position.x, height, arrival.position.z);
            trackBegin = Mathf.Min(blendMillimeters * .001f, Vector3.Distance(start.position, a) * .4f, horizontal.magnitude * .4f);
            float trackEndTrim = Mathf.Min(blendMillimeters * .001f, Vector3.Distance(arrival.position, b) * .4f, horizontal.magnitude * .4f);
            trackSpan = horizontal.magnitude - trackBegin - trackEndTrim;
            Samples.Add(start); ProgramAnchors.Add(start);
            void Corner(Vector3 previous, Vector3 corner, Vector3 next)
            {
                float trim = Mathf.Min(blendMillimeters * .001f, Vector3.Distance(previous, corner) * .4f, Vector3.Distance(corner, next) * .4f);
                if (trim < .0001f) { var sharp = At(corner); Samples.Add(sharp); ProgramAnchors.Add(sharp); return; }
                var before = corner + (previous - corner).normalized * trim; var after = corner + (next - corner).normalized * trim;
                var entry = At(before); Samples.Add(entry); ProgramAnchors.Add(entry);
                // Dense solver samples subsequently enforce IK closure and all collisions.
                for (int i = 1; i <= 8; i++)
                {
                    float t = i / 8f; var pose = At(Vector3.Lerp(Vector3.Lerp(before, corner, t), Vector3.Lerp(corner, after, t), t));
                    Samples.Add(pose); if (i == 4 || i == 8) ProgramAnchors.Add(pose);
                }
            }
            Corner(start.position, a, b);
            var middle = At((a + b) * .5f); Samples.Add(middle); ProgramAnchors.Add(middle);
            Corner(a, b, arrival.position);
            Samples.Add(arrival); ProgramAnchors.Add(arrival);
        }
    }
}
