using System;
using UnityEngine;

namespace RobotIK
{
    public sealed partial class ElasticCableRod
    {
        private Vector3[] curvatureLambda, curvatureRight, curvatureSolution;
        private float[] curvatureA, curvatureB, curvatureC;
        private double[] curvatureDiagonal, curvatureLower, curvatureLower2;
        private int[] selfOrder;
        private Vector3[] selfMin, selfMax;
        private int selfAxis;
        public int SelfCollisionPairsTested { get; private set; }

        private void InitializeAcceleration()
        {
            int count=EdgeCount-1;
            curvatureLambda=new Vector3[count]; curvatureRight=new Vector3[count]; curvatureSolution=new Vector3[count];
            curvatureA=new float[count]; curvatureB=new float[count]; curvatureC=new float[count];
            curvatureDiagonal=new double[count]; curvatureLower=new double[count]; curvatureLower2=new double[count];
            selfOrder=new int[EdgeCount]; selfMin=new Vector3[EdgeCount]; selfMax=new Vector3[EdgeCount];
            for(int i=0;i<EdgeCount;i++) selfOrder[i]=i;
        }

        /// <summary>
        /// Solve all centerline curvature constraints together. The mass-weighted
        /// constraint matrix has five diagonals, so an LDL factorization costs O(n).
        /// Material-frame curvature follows the authored rod, not world-space targets.
        /// </summary>
        private void SolveCoupledCurvature(float h)
        {
            int count=EdgeCount-1;
            for(int row=0;row<count;row++)
            {
                float a=1f/RestLengths[row], c=1f/RestLengths[row+1], b=-a-c;
                curvatureA[row]=a; curvatureB[row]=b; curvatureC[row]=c;
                float alpha=(RestLengths[row]+RestLengths[row+1])*0.5f/Math.Max(0.00001f,BendRigidity)/(h*h);
                Vector3 rest=Frames[row]*(restRelative[row]*Vector3.forward-Vector3.forward);
                Vector3 error=(Positions[row]-Positions[row+1])*a+(Positions[row+2]-Positions[row+1])*c-rest;
                Vector3 right=-error-curvatureLambda[row]*alpha;
                double diagonal=Mass(row)*a*a+Mass(row+1)*b*b+Mass(row+2)*c*c+alpha;
                double lower=0, lower2=0;
                if(row>1) lower2=Mass(row)*a*curvatureC[row-2]/curvatureDiagonal[row-2];
                if(row>0)
                {
                    double shared=Mass(row)*a*curvatureB[row-1]+Mass(row+1)*b*curvatureC[row-1];
                    lower=(shared-(row>1?lower2*curvatureDiagonal[row-2]*curvatureLower[row-1]:0))/curvatureDiagonal[row-1];
                    diagonal-=lower*lower*curvatureDiagonal[row-1];
                    right-=curvatureRight[row-1]*(float)lower;
                }
                if(row>1)
                {
                    diagonal-=lower2*lower2*curvatureDiagonal[row-2];
                    right-=curvatureRight[row-2]*(float)lower2;
                }
                curvatureLower[row]=lower; curvatureLower2[row]=lower2;
                curvatureDiagonal[row]=Math.Max(1e-9,diagonal);
                curvatureRight[row]=right;
            }
            for(int row=count-1;row>=0;row--)
            {
                Vector3 value=curvatureRight[row]/(float)curvatureDiagonal[row];
                if(row+1<count) value-=curvatureSolution[row+1]*(float)curvatureLower[row+1];
                if(row+2<count) value-=curvatureSolution[row+2]*(float)curvatureLower2[row+2];
                curvatureSolution[row]=value;
            }
            for(int node=0;node<Positions.Length;node++)
            {
                Vector3 correction=Vector3.zero;
                if(node<count) correction+=curvatureSolution[node]*curvatureA[node];
                if(node>0 && node-1<count) correction+=curvatureSolution[node-1]*curvatureB[node-1];
                if(node>1) correction+=curvatureSolution[node-2]*curvatureC[node-2];
                Positions[node]+=correction*(Mass(node)*0.8f);
            }
            for(int row=0;row<count;row++) curvatureLambda[row]+=curvatureSolution[row]*0.8f;
        }

        private void ProjectMountDirections(float h)
        {
            for(int edge=0;edge<EdgeCount;edge++)
            {
                if(!framePinned[edge]) continue;
                float a=Mass(edge), b=Mass(edge+1);
                float alpha=RestLengths[edge]/Math.Max(1f,AxialRigidity)/(h*h);
                if(a+b<1e-12f) continue;
                Vector3 error=Positions[edge+1]-Positions[edge]-Frames[edge]*Vector3.forward*RestLengths[edge];
                Vector3 correction=error/(a+b+alpha);
                Positions[edge]+=correction*a; Positions[edge+1]-=correction*b;
            }
        }

        private void AlignMaterialFrames()
        {
            for(int edge=0;edge<EdgeCount;edge++)
            {
                if(framePinned[edge]) continue;
                Vector3 tangent=Positions[edge+1]-Positions[edge];
                if(tangent.sqrMagnitude<1e-12f) continue;
                Frames[edge]=RodMath.Normalize(RodMath.FromTo(Frames[edge]*Vector3.forward,tangent.normalized)*Frames[edge]);
            }
        }

        /// <summary>Adjust rest metrics after arc-length resampling without changing the authored frames.</summary>
        public void SetRestLength(float length, float massPerMeter)
        {
            if(!Finite(length) || !Finite(massPerMeter) || length<=0 || massPerMeter<=0)
                throw new ArgumentOutOfRangeException(nameof(length));
            float scale=length/RestLength;
            Array.Clear(inverseMass,0,inverseMass.Length);
            for(int i=0;i<EdgeCount;i++)
            {
                RestLengths[i]*=scale;
                float mass=massPerMeter*RestLengths[i];
                inverseMass[i]+=mass*0.5f; inverseMass[i+1]+=mass*0.5f;
                inverseBendInertia[i]=1f/(mass*(RestLengths[i]*RestLengths[i]/12f+Radius*Radius/4f));
                inverseTwistInertia[i]=1f/(mass*Radius*Radius*0.5f);
            }
            for(int i=0;i<inverseMass.Length;i++) inverseMass[i]=1f/inverseMass[i];
            RestLength=length;
        }

        private static float Axis(Vector3 value,int axis) => axis==0?value.x:axis==1?value.y:value.z;
        private static bool BoundsOverlap(Vector3 min,Vector3 max,Vector3 otherMin,Vector3 otherMax) =>
            min.x<=otherMax.x && max.x>=otherMin.x && min.y<=otherMax.y && max.y>=otherMin.y && min.z<=otherMax.z && max.z>=otherMin.z;

        private void PrepareSelfCollisionBounds()
        {
            Vector3 allMin=Positions[0], allMax=Positions[0], padding=Vector3.one*Radius;
            for(int i=0;i<EdgeCount;i++)
            {
                selfMin[i]=Vector3.Min(Positions[i],Positions[i+1])-padding;
                selfMax[i]=Vector3.Max(Positions[i],Positions[i+1])+padding;
                allMin=Vector3.Min(allMin,selfMin[i]); allMax=Vector3.Max(allMax,selfMax[i]);
            }
            Vector3 size=allMax-allMin;
            selfAxis=size.x>size.y?(size.x>size.z?0:2):(size.y>size.z?1:2);
            // Reuse the preceding order. Mostly coherent cable motion makes insertion
            // sorting cheap; overlapping boxes alone reach the segment narrowphase.
            for(int i=1;i<EdgeCount;i++)
            {
                int edge=selfOrder[i], j=i-1;
                while(j>=0 && Axis(selfMin[selfOrder[j]],selfAxis)>Axis(selfMin[edge],selfAxis))
                { selfOrder[j+1]=selfOrder[j]; j--; }
                selfOrder[j+1]=edge;
            }
        }
    }
}
