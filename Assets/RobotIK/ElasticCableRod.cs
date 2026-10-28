using System;
using UnityEngine;

namespace RobotIK
{
    /// <summary>
    /// Discrete elastic rod: node positions, material quaternions and compliant stretch/shear,
    /// bend/twist constraints. No Unity object/physics dependency; collision projection is injected.
    /// The angular Jacobians use a small-increment approximation, not a certified beam analysis.
    /// </summary>
    public sealed partial class ElasticCableRod
    {
        public Vector3[] Positions { get; }
        public Quaternion[] Frames { get; }
        public Vector3[] PreviousPositions { get; }
        public Quaternion[] PreviousFrames { get; }
        public Vector3[] SubstepStartPositions { get; }
        public float[] RestLengths { get; }
        public float RestLength { get; private set; }
        public int EdgeCount => RestLengths.Length;
        public float BendRigidity = 2f, TwistRigidity = 0.5f, AxialRigidity = 20000f;
        public float Damping = 5f;
        public bool SelfCollision;
        public bool CoupledBending;
        /// <summary>Contact damping in inverse seconds, independent of the substep count.</summary>
        public float ContactDamping = 22.314355f;
        public float Radius { get; }
        public Action<ElasticCableRod> ProjectContacts;
        public Action<ElasticCableRod> PrepareContacts;

        private readonly Vector3[] velocities, oldPositions;
        private readonly Quaternion[] restRelative, frameTargets, frameStarts;
        private readonly Vector3[] pinTargets, pinStarts, shearLambda, bendLambda, contactNormals;
        private readonly bool[] pinned, framePinned, twistPinned;
        private readonly Quaternion[] twistTargets;
        private readonly float[] inverseMass, inverseBendInertia, inverseTwistInertia, distanceLambda, twistHistory;
        private readonly float[] torsionLower, torsionDiagonal, torsionUpper, torsionRight, torsionSolution;

        public ElasticCableRod(Vector3[] points, float radius, float massPerMeter)
        {
            if (points == null || points.Length < 3 || radius <= 0 || massPerMeter <= 0)
                throw new ArgumentException("A rod needs at least three points and positive radius/density.");
            int n = points.Length, e = n - 1;
            Positions = (Vector3[])points.Clone();
            PreviousPositions = (Vector3[])points.Clone();
            SubstepStartPositions = new Vector3[n];
            oldPositions = new Vector3[n]; velocities = new Vector3[n];
            pinTargets = new Vector3[n]; pinStarts = new Vector3[n]; pinned = new bool[n];
            contactNormals = new Vector3[n]; inverseMass = new float[n];
            Frames = new Quaternion[e]; PreviousFrames = new Quaternion[e];
            frameTargets = new Quaternion[e]; frameStarts = new Quaternion[e]; framePinned = new bool[e];
            twistPinned=new bool[e]; twistTargets=new Quaternion[e];
            torsionLower = new float[e]; torsionDiagonal = new float[e]; torsionUpper = new float[e];
            torsionRight = new float[e]; torsionSolution = new float[e];
            twistHistory = new float[e - 1];
            restRelative = new Quaternion[e - 1];
            RestLengths = new float[e]; shearLambda = new Vector3[e]; bendLambda = new Vector3[e - 1];
            distanceLambda = new float[e]; inverseBendInertia = new float[e]; inverseTwistInertia = new float[e];
            Radius = radius;
            float total = 0;
            Vector3 normal = Vector3.up, previous = Vector3.forward;
            for (int i = 0; i < e; i++)
            {
                float length = Vector3.Distance(points[i], points[i + 1]);
                if (length < 1e-5f || !Finite(length)) throw new ArgumentException("Rod contains degenerate edges.");
                RestLengths[i] = length; total += length;
                Vector3 tangent = (points[i + 1] - points[i]) / length;
                if (i > 0) normal = RodMath.FromTo(previous, tangent) * normal;
                normal = RodMath.Normal(tangent, normal);
                Frames[i] = RodMath.Frame(tangent, normal);
                PreviousFrames[i] = Frames[i];
                previous = tangent;
                float mass = massPerMeter * length;
                inverseMass[i] += mass * 0.5f; inverseMass[i + 1] += mass * 0.5f;
                inverseBendInertia[i] = 1f / (mass * (length * length / 12f + radius * radius / 4f));
                inverseTwistInertia[i] = 1f / (mass * radius * radius * 0.5f);
                if (i > 0) restRelative[i - 1] = RodMath.Inverse(Frames[i - 1]) * Frames[i];
            }
            for (int i = 0; i < n; i++) inverseMass[i] = 1f / inverseMass[i];
            RestLength = total;
            InitializeAcceleration();
        }

        public void SetPositionPin(int node, Vector3 target)
        { pinned[node] = true; pinTargets[node] = target; }
        public void SetFramePin(int edge, Quaternion target)
        { framePinned[edge] = true; frameTargets[edge] = RodMath.Normalize(target); }
        public void SetTwistPin(int edge,Quaternion target)
        { twistPinned[edge]=true; twistTargets[edge]=RodMath.Normalize(target); }
        private bool TwistHeld(int edge) => framePinned[edge] || twistPinned[edge];
        public bool HasContact(int node) => contactNormals[node].sqrMagnitude>0;
        public bool IsPinned(int node) => pinned[node];
        public void AddImpulse(int node, Vector3 impulse)
        { if (!pinned[node]) velocities[node] += impulse * inverseMass[node]; }
        public void ResetVelocities()
        { Array.Clear(velocities, 0, velocities.Length); }

        public void Step(float dt, int substeps, int iterations, Vector3 gravity)
        {
            if (!Finite(dt) || dt <= 0 || dt > 0.1f) throw new ArgumentOutOfRangeException(nameof(dt));
            substeps = Math.Max(1, Math.Min(16, substeps));
            iterations = Math.Max(4, Math.Min(96, iterations));
            Array.Copy(Positions, PreviousPositions, Positions.Length);
            Array.Copy(Frames, PreviousFrames, Frames.Length);
            Array.Copy(Positions, pinStarts, Positions.Length);
            Array.Copy(Frames, frameStarts, Frames.Length);
            SelfCollisionPairsTested = 0;
            float h = dt / substeps, decay = (float)Math.Exp(-Math.Max(0, Damping) * h);
            for (int sub = 0; sub < substeps; sub++)
            {
                Array.Copy(Positions, oldPositions, Positions.Length);
                Array.Copy(Positions, SubstepStartPositions, Positions.Length);
                Array.Clear(shearLambda, 0, shearLambda.Length);
                Array.Clear(bendLambda, 0, bendLambda.Length);
                Array.Clear(distanceLambda, 0, distanceLambda.Length);
                Array.Clear(contactNormals, 0, contactNormals.Length);
                Array.Clear(curvatureLambda, 0, curvatureLambda.Length);
                for (int i = 0; i < Positions.Length; i++)
                {
                    if (pinned[i]) continue;
                    velocities[i] = (velocities[i] + gravity * h) * decay;
                    Positions[i] += velocities[i] * h;
                }
                float fraction = (sub + 1f) / substeps;
                ApplyPins(fraction);
                PrepareContacts?.Invoke(this);
                for (int iteration = 0; iteration < iterations; iteration++)
                {
                    bool reverse = (iteration & 1) != 0;
                    if (CoupledBending)
                    {
                        SolveCoupledCurvature(h);
                        ProjectMountDirections(h);
                    }
                    else
                    {
                        for (int k = 0; k < EdgeCount - 1; k++) Bend(reverse ? EdgeCount - 2 - k : k, h);
                        for (int k = 0; k < EdgeCount; k++) Shear(reverse ? EdgeCount - 1 - k : k, h);
                    }
                    for (int k = 0; k < EdgeCount; k++) Distance(reverse ? EdgeCount - 1 - k : k, h);
                    if (CoupledBending) AlignMaterialFrames();
                    // Torsion is overdamped; it does not need a global solve on every
                    // local constraint sweep. Keep winding updates between passes.
                    if ((iteration & 3) == 3 || iteration == iterations - 1) EquilibrateTorsion();
                    if ((iteration & 3) == 3 || iteration == iterations - 1)
                    {
                        if (SelfCollision) ResolveSelfContacts();
                        ProjectContacts?.Invoke(this);
                    }
                    ApplyPins(fraction);
                    UpdateTwistHistory();
                }
                for (int i = 0; i < Positions.Length; i++)
                {
                    velocities[i] = pinned[i] ? Vector3.zero : (Positions[i] - oldPositions[i]) / h;
                    if (contactNormals[i].sqrMagnitude > 0)
                    {
                        Vector3 normal = contactNormals[i].normalized;
                        float into = Vector3.Dot(velocities[i], normal);
                        if (into < 0) velocities[i] -= normal * into;
                        velocities[i] *= (float)Math.Exp(-Math.Max(0, ContactDamping) * h);
                    }
                }
            }
        }

        private void ApplyPins(float fraction)
        {
            for (int i = 0; i < Positions.Length; i++)
                if (pinned[i]) Positions[i] = Vector3.LerpUnclamped(pinStarts[i], pinTargets[i], fraction);
            for (int i = 0; i < EdgeCount; i++)
                if (framePinned[i]) Frames[i] = RodMath.Slerp(frameStarts[i], frameTargets[i], fraction);
                else if(twistPinned[i])
                {
                    Vector3 tangent=Frames[i]*Vector3.forward;
                    Frames[i]=RodMath.Frame(tangent,RodMath.Normal(tangent,twistTargets[i]*Vector3.up));
                }
        }
        private float Mass(int i) => pinned[i] ? 0 : inverseMass[i];
        private Vector3 Inertia(int i, Vector3 world)
        {
            if (framePinned[i]) return Vector3.zero;
            Vector3 local = RodMath.Inverse(Frames[i]) * world;
            local.x *= inverseBendInertia[i]; local.y *= inverseBendInertia[i]; local.z *= twistPinned[i]?0:inverseTwistInertia[i];
            return Frames[i] * local;
        }
        private void Rotate(int i, Vector3 increment, bool limit = true)
        {
            if (!framePinned[i]) Frames[i] = RodMath.Normalize(RodMath.Exp(limit ? ClampRotation(increment) : increment) * Frames[i]);
        }
        private static Vector3 ClampRotation(Vector3 value)
        {
            float n = value.magnitude;
            return n > 0.35f ? value * (0.35f / n) : value;
        }
        private static float LimitMultiplier(float dl, float gradientWeight)
        {
            float angle = Math.Abs(dl) * gradientWeight;
            return angle > 0.35f ? dl * (0.35f / angle) : dl;
        }
        private void Shear(int edge, float h)
        {
            float length = RestLengths[edge];
            float alpha = length / Math.Max(1f, AxialRigidity) / (h * h);
            float w0 = Mass(edge), w1 = Mass(edge + 1);
            for (int axis = 0; axis < 3; axis++)
            {
                Vector3 direction = axis == 0 ? Vector3.right : axis == 1 ? Vector3.up : Vector3.forward;
                Vector3 tangent = Frames[edge] * Vector3.forward;
                Vector3 c = Positions[edge + 1] - Positions[edge] - tangent * length;
                Vector3 gradient = Vector3.Cross(direction, tangent) * length;
                Vector3 weighted = Inertia(edge, gradient);
                float denominator = w0 + w1 + Vector3.Dot(gradient, weighted) + alpha;
                if (denominator < 1e-12f) continue;
                float dl = (-c[axis] - alpha * shearLambda[edge][axis]) / denominator;
                dl = LimitMultiplier(dl, weighted.magnitude);
                Vector3 lambda = shearLambda[edge]; lambda[axis] += dl; shearLambda[edge] = lambda;
                Positions[edge] -= direction * (w0 * dl);
                Positions[edge + 1] += direction * (w1 * dl);
                Rotate(edge, weighted * dl);
            }
        }
        private void Distance(int edge, float h)
        {
            Vector3 delta = Positions[edge + 1] - Positions[edge];
            float length = delta.magnitude, w0 = Mass(edge), w1 = Mass(edge + 1);
            if (length < 1e-9f || w0 + w1 == 0) return;
            float alpha = RestLengths[edge] / Math.Max(1f, AxialRigidity) / (h * h);
            float dl = (-(length - RestLengths[edge]) - alpha * distanceLambda[edge]) / (w0 + w1 + alpha);
            distanceLambda[edge] += dl;
            Vector3 correction = delta / length * dl;
            Positions[edge] -= correction * w0;
            Positions[edge + 1] += correction * w1;
        }
        private void Bend(int edge, float h)
        {
            float length = (RestLengths[edge] + RestLengths[edge + 1]) * 0.5f;
            for (int axis = 0; axis < 2; axis++)
            {
                Quaternion relative = RodMath.Inverse(Frames[edge]) * Frames[edge + 1] * RodMath.Inverse(restRelative[edge]);
                Vector3 error = RodMath.Log(relative);
                Vector3 direction = Frames[edge] * (axis == 0 ? Vector3.right : Vector3.up);
                Vector3 wa = Inertia(edge, direction), wb = Inertia(edge + 1, direction);
                float alpha = length / Math.Max(0.00001f, BendRigidity) / (h * h);
                float denominator = Vector3.Dot(direction, wa + wb) + alpha;
                if (denominator < 1e-12f) continue;
                float dl = (-error[axis] - alpha * bendLambda[edge][axis]) / denominator;
                dl = LimitMultiplier(dl, Math.Max(wa.magnitude, wb.magnitude));
                Vector3 lambda = bendLambda[edge]; lambda[axis] += dl; bendLambda[edge] = lambda;
                Rotate(edge, -wa * dl); Rotate(edge + 1, wb * dl);
            }
        }
        private float ReadTwist(int edge)
        {
            Quaternion relative = RodMath.Inverse(Frames[edge]) * Frames[edge+1] * RodMath.Inverse(restRelative[edge]);
            float raw = 2f * (float)Math.Atan2(relative.z, relative.w), turn = 2f * (float)Math.PI;
            return raw + turn * (float)Math.Round((twistHistory[edge] - raw) / turn);
        }
        // The light cross sections are overdamped. Solve their coupled torsional energy
        // globally (a tridiagonal system), avoiding localized wrist wind-up from incomplete
        // pairwise projection. Centerline translation/bending retains dynamic motion.
        private void EquilibrateTorsion()
        {
            bool anyHeld = false;
            for (int i = 0; i < EdgeCount; i++) anyHeld |= TwistHeld(i);
            for (int i = 0; i < EdgeCount-1; i++) twistHistory[i] = ReadTwist(i);
            for (int i = 0; i < EdgeCount; i++)
            {
                bool held = TwistHeld(i) || (!anyHeld && i == 0);
                float left = i>0 ? TwistRigidity / ((RestLengths[i-1]+RestLengths[i])*0.5f) : 0;
                float right = i<EdgeCount-1 ? TwistRigidity / ((RestLengths[i]+RestLengths[i+1])*0.5f) : 0;
                torsionDiagonal[i] = held ? 1 : left+right+1e-5f;
                torsionLower[i] = held || i==0 || TwistHeld(i-1) || (!anyHeld && i==1) ? 0 : -left;
                torsionUpper[i] = held || i==EdgeCount-1 || TwistHeld(i+1) ? 0 : -right;
                torsionRight[i] = held ? 0 : (i<EdgeCount-1 ? right*twistHistory[i] : 0) - (i>0 ? left*twistHistory[i-1] : 0);
            }
            for (int i=1;i<EdgeCount;i++)
            {
                float factor = torsionLower[i] / torsionDiagonal[i-1];
                torsionDiagonal[i] -= factor*torsionUpper[i-1];
                torsionRight[i] -= factor*torsionRight[i-1];
            }
            for (int i=EdgeCount-1;i>=0;i--)
                torsionSolution[i] = (torsionRight[i] - (i<EdgeCount-1 ? torsionUpper[i]*torsionSolution[i+1] : 0)) / Math.Max(1e-9f,torsionDiagonal[i]);
            for (int i=0;i<EdgeCount;i++)
                Rotate(i,Frames[i]*Vector3.forward*Math.Max(-0.5f,Math.Min(0.5f,torsionSolution[i])),false);
        }
        public float MaximumRelativeStretch()
        {
            float result = 0;
            for (int i = 0; i < EdgeCount; i++) result = Math.Max(result,
                Math.Abs(Vector3.Distance(Positions[i], Positions[i + 1]) / RestLengths[i] - 1f));
            return result;
        }
        private void UpdateTwistHistory()
        {
            float turn = 2f * (float)Math.PI;
            for (int i = 0; i < EdgeCount - 1; i++)
            {
                Quaternion relative = RodMath.Inverse(Frames[i]) * Frames[i+1] * RodMath.Inverse(restRelative[i]);
                float raw = 2f * (float)Math.Atan2(relative.z, relative.w);
                twistHistory[i] = raw + turn * (float)Math.Round((twistHistory[i] - raw) / turn);
            }
        }
        public float TotalTwistRadians()
        {
            float result = 0;
            for (int i = 0; i < EdgeCount - 1; i++) result += twistHistory[i];
            return result;
        }
        public void ResolveNodeContact(int node, Vector3 normal, float depth)
        {
            if (pinned[node] || depth <= 0) return;
            Positions[node] += normal * depth;
            contactNormals[node] += normal;
        }
        public void ResolveEdgeContact(int edge, float t, Vector3 normal, float depth)
        {
            t = Math.Max(0, Math.Min(1, t));
            float a = 1f - t, b = t, w0 = Mass(edge), w1 = Mass(edge + 1);
            float denominator = w0 * a * a + w1 * b * b;
            if (denominator < 1e-9f || depth <= 0) return;
            Positions[edge] += normal * (depth * w0 * a / denominator);
            Positions[edge + 1] += normal * (depth * w1 * b / denominator);
            contactNormals[edge] += normal; contactNormals[edge + 1] += normal;
        }
        private void ResolveSelfContacts()
        {
            PrepareSelfCollisionBounds();
            for (int first = 0; first < EdgeCount; first++) for (int second = first+1; second < EdgeCount; second++)
            {
                int i=selfOrder[first], j=selfOrder[second];
                if (Axis(selfMin[j],selfAxis)>Axis(selfMax[i],selfAxis)) break;
                if (Math.Abs(i-j)<3 || !BoundsOverlap(selfMin[i],selfMax[i],selfMin[j],selfMax[j])) continue;
                SelfCollisionPairsTested++;
                RodMath.ClosestSegments(Positions[i], Positions[i + 1], Positions[j], Positions[j + 1], out float s, out float t);
                Vector3 delta = Vector3.Lerp(Positions[i], Positions[i + 1], s) - Vector3.Lerp(Positions[j], Positions[j + 1], t);
                float distance = delta.magnitude;
                if (distance >= 2 * Radius) continue;
                Vector3 normal = distance > 1e-6f ? delta / distance : RodMath.Normal((Positions[i + 1] - Positions[i]).normalized, Vector3.up);
                float a = 1-s, b = s, c = 1-t, d = t;
                float denominator = Mass(i)*a*a + Mass(i+1)*b*b + Mass(j)*c*c + Mass(j+1)*d*d;
                if (denominator < 1e-9f) continue;
                float correction = (2*Radius - distance) / denominator;
                Positions[i] += normal * (correction * Mass(i) * a);
                Positions[i+1] += normal * (correction * Mass(i+1) * b);
                Positions[j] -= normal * (correction * Mass(j) * c);
                Positions[j+1] -= normal * (correction * Mass(j+1) * d);
            }
        }
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }

    /// <summary>Managed quaternion math, also executable by the standalone numerical checks.</summary>
    public static class RodMath
    {
        public static Quaternion Normalize(Quaternion q)
        {
            float n = (float)Math.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
            return n < 1e-10f ? Quaternion.identity : new Quaternion(q.x/n, q.y/n, q.z/n, q.w/n);
        }
        public static Quaternion Inverse(Quaternion q) => new Quaternion(-q.x, -q.y, -q.z, q.w);
        public static Quaternion Exp(Vector3 rotation)
        {
            float angle = rotation.magnitude;
            if (angle < 1e-8f) return Normalize(new Quaternion(rotation.x*0.5f, rotation.y*0.5f, rotation.z*0.5f, 1));
            float s = (float)Math.Sin(angle*0.5) / angle;
            return new Quaternion(rotation.x*s, rotation.y*s, rotation.z*s, (float)Math.Cos(angle*0.5));
        }
        public static Vector3 Log(Quaternion q)
        {
            q = Normalize(q);
            if (q.w < 0) q = new Quaternion(-q.x, -q.y, -q.z, -q.w);
            float n = (float)Math.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
            float scale = n < 1e-8f ? 2f : 2f * (float)Math.Atan2(n, q.w) / n;
            return new Vector3(q.x*scale, q.y*scale, q.z*scale);
        }
        public static Quaternion Slerp(Quaternion a, Quaternion b, float t)
        { return Normalize(Exp(Log(b * Inverse(a)) * t) * a); }
        public static Vector3 Normal(Vector3 tangent, Vector3 preferred)
        {
            Vector3 n = preferred - tangent * Vector3.Dot(tangent, preferred);
            if (n.sqrMagnitude < 1e-8f)
            {
                Vector3 basis = Math.Abs(tangent.y) < 0.9f ? Vector3.up : Vector3.right;
                n = basis - tangent * Vector3.Dot(tangent, basis);
            }
            return n.normalized;
        }
        public static Quaternion FromTo(Vector3 a, Vector3 b)
        {
            float dot = Vector3.Dot(a,b);
            if (dot < -0.99999f) return Exp(Normal(a, Vector3.up) * (float)Math.PI);
            Vector3 cross = Vector3.Cross(a,b);
            return Normalize(new Quaternion(cross.x, cross.y, cross.z, 1+dot));
        }
        public static Quaternion Frame(Vector3 tangent, Vector3 normal)
        {
            Quaternion q = FromTo(Vector3.forward, tangent);
            Vector3 up = q * Vector3.up;
            float angle = (float)Math.Atan2(Vector3.Dot(tangent, Vector3.Cross(up,normal)), Vector3.Dot(up,normal));
            return Normalize(Exp(tangent*angle) * q);
        }
        public static void ClosestSegments(Vector3 p, Vector3 p1, Vector3 q, Vector3 q1, out float s, out float t)
        {
            Vector3 a = p1-p, b = q1-q, r = p-q;
            float aa = Vector3.Dot(a,a), bb = Vector3.Dot(b,b), ab = Vector3.Dot(a,b), ar = Vector3.Dot(a,r), br = Vector3.Dot(b,r);
            float determinant = aa*bb-ab*ab;
            s = determinant > 1e-12f ? Math.Max(0, Math.Min(1, (ab*br-bb*ar)/determinant)) : 0;
            t = bb > 1e-12f ? (ab*s+br)/bb : 0;
            if (t < 0) { t=0; s=aa > 1e-12f ? Math.Max(0,Math.Min(1,-ar/aa)) : 0; }
            else if (t > 1) { t=1; s=aa > 1e-12f ? Math.Max(0,Math.Min(1,(ab-ar)/aa)) : 0; }
        }
    }
}

