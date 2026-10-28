using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class ElasticCableRod
    {
        /// <summary>
        /// Feed material at node zero. Positive adds cable, negative consumes it.
        /// Only the inlet edge changes length; downstream material, velocities and
        /// intrinsic curvature survive remeshing. The caller reapplies attachment pins.
        /// No scene objects, collision queries or renderer dependencies.
        /// </summary>
        public ElasticCableRod FeedAtStart(float delta, float segmentLength, float massPerMeter)
            => FeedBoundary(delta,segmentLength,massPerMeter,true);

        public ElasticCableRod FeedAtEnd(float delta, float segmentLength, float massPerMeter)
            => FeedBoundary(delta,segmentLength,massPerMeter,false);

        private ElasticCableRod FeedBoundary(float delta, float segmentLength, float massPerMeter, bool atStart)
        {
            if (!Finite(delta) || !Finite(segmentLength) || segmentLength <= 1e-4f ||
                !Finite(massPerMeter) || massPerMeter <= 0)
                throw new ArgumentOutOfRangeException(nameof(delta));
            if (Math.Abs(delta) < 1e-8f) return this;
            if (Math.Abs(delta) > segmentLength * 0.25f)
                throw new ArgumentOutOfRangeException(nameof(delta), "Feed in increments below a quarter element.");

            int boundary=atStart?0:EdgeCount-1;
            float inletLength=RestLengths[boundary]+delta;
            if (inletLength < segmentLength*0.1f)
                throw new ArgumentOutOfRangeException(nameof(delta), "Minimum deployed inlet length reached.");
            bool remove=inletLength<segmentLength*0.5f && EdgeCount>3;
            bool insert=inletLength>segmentLength*1.5f;
            if (!remove && !insert)
            {
                // Normal feed steps allocate nothing and leave all downstream state intact.
                RestLength+=delta;
                RestLengths[boundary]=inletLength;
                float mass=massPerMeter*inletLength;
                int endNode=atStart?0:EdgeCount, innerNode=atStart?1:EdgeCount-1;
                inverseMass[endNode]=2f/mass;
                inverseMass[innerNode]=2f/(mass+massPerMeter*RestLengths[atStart?1:EdgeCount-2]);
                inverseBendInertia[boundary]=1f/(mass*(inletLength*inletLength/12f+Radius*Radius/4f));
                inverseTwistInertia[boundary]=1f/(mass*Radius*Radius*0.5f);
                return this;
            }

            var lengths = new List<float>(RestLengths);
            lengths[boundary] += delta;
            if (lengths[boundary] < segmentLength * 0.5f && lengths.Count > 3)
            {
                lengths[atStart?1:boundary-1] += lengths[boundary];
                lengths.RemoveAt(boundary);
            }
            boundary=atStart?0:lengths.Count-1;
            if (lengths[boundary] > segmentLength * 1.5f)
            {
                float first = lengths[boundary] * 0.5f;
                lengths[boundary] -= first;
                lengths.Insert(atStart?0:lengths.Count, first);
            }
            if (lengths[atStart?0:lengths.Count-1] < segmentLength * 0.1f)
                throw new ArgumentOutOfRangeException(nameof(delta), "Minimum deployed inlet length reached.");

            // Construct with nondegenerate rest geometry, then transfer dynamic state.
            var seed = new Vector3[lengths.Count + 1];
            for (int i = 0; i < lengths.Count; i++) seed[i+1] = seed[i] + Vector3.forward * lengths[i];
            var next = new ElasticCableRod(seed, Radius, massPerMeter)
            {
                BendRigidity = BendRigidity, TwistRigidity = TwistRigidity,
                AxialRigidity = AxialRigidity, Damping = Damping,
                SelfCollision = SelfCollision, ProjectContacts = ProjectContacts,
                PrepareContacts = PrepareContacts, CoupledBending = CoupledBending, ContactDamping = ContactDamping
            };
            var offsets = new float[EdgeCount + 1];
            for (int i = 0; i < EdgeCount; i++) offsets[i+1] = offsets[i] + RestLengths[i];
            var sourceEdges = new int[lengths.Count];
            float at = 0;
            for (int i = 0; i < next.Positions.Length; i++)
            {
                float source = Math.Max(0, Math.Min(RestLength, at - (atStart?delta:0)));
                int edge = FindEdge(offsets, source);
                float t = Math.Max(0, Math.Min(1, (source-offsets[edge])/RestLengths[edge]));
                next.Positions[i] = Vector3.LerpUnclamped(Positions[edge], Positions[edge+1], t);
                next.PreviousPositions[i] = Vector3.LerpUnclamped(PreviousPositions[edge], PreviousPositions[edge+1], t);
                next.velocities[i] = Vector3.LerpUnclamped(velocities[edge], velocities[edge+1], t);
                if (i == lengths.Count) continue;
                int frame = FindEdge(offsets, Math.Max(0, at + lengths[i]*0.5f - (atStart?delta:0)));
                sourceEdges[i] = frame;
                next.Frames[i] = Frames[frame];
                next.PreviousFrames[i] = PreviousFrames[frame];
                at += lengths[i];
            }
            next.Positions[0] = Positions[0];
            next.PreviousPositions[0] = PreviousPositions[0];
            next.Positions[next.EdgeCount] = Positions[EdgeCount];
            next.PreviousPositions[next.EdgeCount] = PreviousPositions[EdgeCount];
            for (int i = 0; i < lengths.Count-1; i++)
            {
                Quaternion intrinsic = Quaternion.identity;
                float winding = 0;
                for (int j = sourceEdges[i]; j < sourceEdges[i+1]; j++)
                {
                    intrinsic = intrinsic * restRelative[j];
                    winding += twistHistory[j];
                }
                next.restRelative[i] = RodMath.Normalize(intrinsic);
                next.twistHistory[i] = winding;
            }
            return next;
        }

        public struct BoundaryMaterial
        {
            public Quaternion frame;
            public Vector3 velocity, intrinsicCurvature;
            public float twistPerMeter;
        }

        public BoundaryMaterial SampleBoundaryMaterial(bool atStart)
        {
            int edge=atStart?0:EdgeCount-1, joint=atStart?0:EdgeCount-2;
            float length=(RestLengths[joint]+RestLengths[joint+1])*0.5f;
            return new BoundaryMaterial { frame=Frames[edge], velocity=velocities[atStart?1:EdgeCount-1],
                intrinsicCurvature=RodMath.Log(restRelative[joint])/length, twistPerMeter=twistHistory[joint]/length };
        }

        /// <summary>Mix incoming material metrics at a feeding guide, preserving downstream history.</summary>
        public void ReceiveBoundaryMaterial(bool atStart,float length,BoundaryMaterial sample)
        {
            int edge=atStart?0:EdgeCount-1, joint=atStart?0:EdgeCount-2;
            float blend=Math.Min(1,length/RestLengths[edge]);
            Quaternion transported=RodMath.FromTo(sample.frame*Vector3.forward,Frames[edge]*Vector3.forward)*sample.frame;
            Frames[edge]=RodMath.Slerp(Frames[edge],transported,blend);
            int node=atStart?1:EdgeCount-1;
            velocities[node]=Vector3.LerpUnclamped(velocities[node],sample.velocity,blend);
            float jointLength=(RestLengths[joint]+RestLengths[joint+1])*0.5f;
            restRelative[joint]=RodMath.Slerp(restRelative[joint],RodMath.Exp(sample.intrinsicCurvature*jointLength),blend);
            twistHistory[joint]=twistHistory[joint]*(1-blend)+sample.twistPerMeter*jointLength*blend;
        }

        private static int FindEdge(float[] offsets, float distance)
        {
            int low = 0, high = offsets.Length-2;
            while (low < high)
            {
                int middle = (low+high+1)/2;
                if (offsets[middle] <= distance) low=middle; else high=middle-1;
            }
            return low;
        }
    }
}
