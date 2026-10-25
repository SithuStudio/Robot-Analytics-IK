using System;

namespace RobotIK
{
    /// <summary>Immutable, worker-safe calibrated geometry and posture preference.</summary>
    public readonly struct ArmStretchGeometry
    {
        public readonly double Length1, Length2, Cosine, Sine;
        public ArmStretchGeometry(double first, double second, double cosine, double sine)
        { Length1 = first; Length2 = second; Cosine = cosine; Sine = sine; }
        public bool Valid => Length1 > 0 && Length2 > 0 && !double.IsInfinity(Length1) && !double.IsInfinity(Length2) && !double.IsNaN(Cosine) && !double.IsNaN(Sine) && !double.IsInfinity(Cosine) && !double.IsInfinity(Sine);
        public float Measure(float[] q)
        {
            if (!Valid || q == null || q.Length != 6 || float.IsNaN(q[2]) || float.IsInfinity(q[2])) return float.NaN;
            foreach (float value in q) if (float.IsNaN(value) || float.IsInfinity(value)) return float.NaN;
            double angle = q[2] * Math.PI / 180;
            double square = Length1 * Length1 + Length2 * Length2 + 2 * Length1 * Length2 * (Cosine * Math.Cos(angle) + Sine * Math.Sin(angle));
            return (float)(Math.Sqrt(Math.Max(0, square)) / (Length1 + Length2));
        }
    }
    public readonly struct ArmStretchPolicy : IEquatable<ArmStretchPolicy>
    {
        public readonly bool Enabled, Strict, AllowE1, TransferOnly;
        public readonly float Limit, Weight;
        public ArmStretchPolicy(bool enabled, bool strict, bool allowE1, bool transferOnly, float percent, float weight)
        { Enabled = enabled; Strict = strict; AllowE1 = allowE1; TransferOnly = transferOnly; Limit = percent * .01f; Weight = weight; }
        public bool Applies(bool transfer) => Enabled && (!TransferOnly || transfer);
        public bool Allows(float ratio, bool transfer) => !Applies(transfer) || !Strict || !float.IsNaN(ratio) && !float.IsInfinity(ratio) && ratio <= Limit + .00001f;
        public float Penalty(float ratio, bool transfer)
        {
            if (!Applies(transfer) || Strict) return 0;
            if (float.IsNaN(ratio) || float.IsInfinity(ratio)) return 1000000;
            double excess = Math.Max(0, (ratio - Limit) / Math.Max(.01, 1 - Limit));
            return (float)(Weight * excess * excess);
        }
        public bool Equals(ArmStretchPolicy p) => Enabled == p.Enabled && Strict == p.Strict && AllowE1 == p.AllowE1 && TransferOnly == p.TransferOnly && Limit.Equals(p.Limit) && Weight.Equals(p.Weight);
    }
}
