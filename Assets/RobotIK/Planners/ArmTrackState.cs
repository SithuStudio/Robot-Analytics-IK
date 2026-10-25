using System;
using UnityEngine;

namespace RobotIK
{
    /// <summary>One immutable physical configuration. E1 is millimeters; arm coordinates are degrees.</summary>
    public sealed class ArmTrackState
    {
        public const int ArmJointCount = 6;
        private readonly float[] joints;
        public bool HasTrackAxis { get; }
        public float E1Millimeters { get; }
        public float this[int joint] => joints[joint];
        public float[] JointAnglesDegrees => (float[])joints.Clone();

        public ArmTrackState(float[] jointAnglesDegrees, float? e1Millimeters = null)
        {
            if (jointAnglesDegrees == null || jointAnglesDegrees.Length != ArmJointCount)
                throw new ArgumentException("A combined state requires exactly six arm coordinates.", nameof(jointAnglesDegrees));
            foreach (float value in jointAnglesDegrees)
                if (!Finite(value)) throw new ArgumentException("Arm coordinates must be finite.", nameof(jointAnglesDegrees));
            if (e1Millimeters.HasValue && !Finite(e1Millimeters.Value))
                throw new ArgumentException("E1 must be finite.", nameof(e1Millimeters));
            joints = (float[])jointAnglesDegrees.Clone();
            HasTrackAxis = e1Millimeters.HasValue; E1Millimeters = e1Millimeters ?? 0f;
        }
        private static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
    }

    /// <summary>Exact state plus scene/settings generation. No rounding or angle wrapping can alias nearby collisions.</summary>
    public readonly struct ArmTrackStateKey : IEquatable<ArmTrackStateKey>
    {
        private readonly float a, b, c, d, e, f, e1;
        private readonly bool tracked;
        private readonly long context;
        public ArmTrackStateKey(ArmTrackState state, long contextRevision)
        {
            if (state == null) throw new ArgumentNullException(nameof(state));
            a = state[0]; b = state[1]; c = state[2]; d = state[3]; e = state[4]; f = state[5];
            e1 = state.E1Millimeters; tracked = state.HasTrackAxis; context = contextRevision;
        }
        public bool Equals(ArmTrackStateKey x) => tracked == x.tracked && context == x.context && e1.Equals(x.e1) &&
            a.Equals(x.a) && b.Equals(x.b) && c.Equals(x.c) && d.Equals(x.d) && e.Equals(x.e) && f.Equals(x.f);
        public override bool Equals(object obj) => obj is ArmTrackStateKey x && Equals(x);
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = context.GetHashCode() * 31 + tracked.GetHashCode();
                hash = hash * 31 + e1.GetHashCode(); hash = hash * 31 + a.GetHashCode(); hash = hash * 31 + b.GetHashCode();
                hash = hash * 31 + c.GetHashCode(); hash = hash * 31 + d.GetHashCode(); hash = hash * 31 + e.GetHashCode();
                return hash * 31 + f.GetHashCode();
            }
        }
    }

    /// <summary>Symmetric exact endpoints, including E1 and the collision context.</summary>
    public readonly struct ArmTrackSegmentKey : IEquatable<ArmTrackSegmentKey>
    {
        private readonly ArmTrackStateKey first, second;
        public ArmTrackSegmentKey(ArmTrackState from, ArmTrackState to, long contextRevision)
        { first = new ArmTrackStateKey(from, contextRevision); second = new ArmTrackStateKey(to, contextRevision); }
        public bool Equals(ArmTrackSegmentKey x) => (first.Equals(x.first) && second.Equals(x.second)) || (first.Equals(x.second) && second.Equals(x.first));
        public override bool Equals(object obj) => obj is ArmTrackSegmentKey x && Equals(x);
        public override int GetHashCode() => first.GetHashCode() ^ second.GetHashCode();
    }

    public sealed class ArmTrackCollisionCheck
    {
        public bool IsValid { get; }
        public bool InCollision { get; }
        public bool CollisionFree => IsValid && !InCollision;
        public Pose TcpPose { get; }
        public string Description { get; }
        internal ArmTrackCollisionCheck(bool valid, bool collision, Pose tcpPose, string description)
        { IsValid = valid; InCollision = collision; TcpPose = tcpPose; Description = description; }
    }
}
