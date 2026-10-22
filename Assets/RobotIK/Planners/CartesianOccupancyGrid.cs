using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Immutable scan data. No Transform, Physics or other scene access.</summary>
    public sealed class CartesianOccupancyGrid
    {
        private readonly bool[,,] occupied;
        private readonly Vector3 origin, stepX, stepY, stepZ, reciprocalX, reciprocalY, reciprocalZ, extent;
        public int CountX { get; }
        public int CountY { get; }
        public int CountZ { get; }
        public int Count => CountX * CountY * CountZ;

        public CartesianOccupancyGrid(bool[,,] cells, Vector3 origin, Vector3 stepX,
            Vector3 stepY, Vector3 stepZ, Vector3 extentInCells)
        {
            if (cells == null) throw new ArgumentNullException(nameof(cells));
            float determinant = Vector3.Dot(stepX, Vector3.Cross(stepY,stepZ));
            if (Math.Abs(determinant) < 1e-12f) throw new ArgumentException("Scan transform has a zero scale.");
            occupied = (bool[,,])cells.Clone();
            CountX=cells.GetLength(0); CountY=cells.GetLength(1); CountZ=cells.GetLength(2);
            this.origin=origin; this.stepX=stepX; this.stepY=stepY; this.stepZ=stepZ; extent=extentInCells;
            reciprocalX=Vector3.Cross(stepY,stepZ)/determinant;
            reciprocalY=Vector3.Cross(stepZ,stepX)/determinant;
            reciprocalZ=Vector3.Cross(stepX,stepY)/determinant;
        }
        private Vector3 Coordinates(Vector3 p)
        {
            p-=origin;
            return new Vector3(Vector3.Dot(p,reciprocalX),Vector3.Dot(p,reciprocalY),Vector3.Dot(p,reciprocalZ));
        }
        public int Index(int x,int y,int z) => (x*CountY+y)*CountZ+z;
        public void Cell(int index,out int x,out int y,out int z)
        { z=index%CountZ; y=(index/CountZ)%CountY; x=index/(CountY*CountZ); }
        public bool IsFree(int x,int y,int z, ISet<int> forbidden=null) =>
            x>=0 && x<CountX && y>=0 && y<CountY && z>=0 && z<CountZ &&
            !occupied[x,y,z] && (forbidden==null || !forbidden.Contains(Index(x,y,z)));
        public bool TryCell(Vector3 p,out int index)
        {
            Vector3 c=Coordinates(p); index=-1;
            if(c.x<0 || c.y<0 || c.z<0 || c.x>=extent.x || c.y>=extent.y || c.z>=extent.z) return false;
            int x=(int)Math.Floor(c.x), y=(int)Math.Floor(c.y), z=(int)Math.Floor(c.z);
            if(x>=CountX || y>=CountY || z>=CountZ) return false;
            index=Index(x,y,z); return true;
        }
        public bool IsFree(Vector3 p)
        {
            if(!TryCell(p,out int index)) return false;
            Cell(index,out int x,out int y,out int z); return IsFree(x,y,z);
        }
        public Vector3 Center(int index)
        {
            Cell(index,out int x,out int y,out int z);
            // The final cell can be smaller than a full voxel.
            return origin + stepX*((x+Math.Min(x+1f,extent.x))*.5f) +
                stepY*((y+Math.Min(y+1f,extent.y))*.5f) + stepZ*((z+Math.Min(z+1f,extent.z))*.5f);
        }

        // Guidance cost only. SegmentFree and full MoveL validation remain the
        // acceptance checks; retry penalties never turn a free cell into an obstacle.
        internal float SegmentCost(Vector3 a,Vector3 b,IReadOnlyDictionary<int,float> penalties)
        {
            float length=Vector3.Distance(a,b);
            if(penalties.Count==0) return length;
            Vector3 delta=Coordinates(b)-Coordinates(a);
            int samples=Math.Max(1,(int)Math.Ceiling(Math.Max(Math.Abs(delta.x),Math.Max(Math.Abs(delta.y),Math.Abs(delta.z)))*4));
            float cost=0;
            for(int i=0;i<samples;i++)
            {
                if(!TryCell(Vector3.Lerp(a,b,(i+.5f)/samples),out int id)) return float.PositiveInfinity;
                cost+=1+(penalties.TryGetValue(id,out float penalty)?penalty:0);
            }
            return length*cost/samples;
        }

        /// <summary>Voxel traversal checks every crossed cell, including diagonal ties.</summary>
        public bool SegmentFree(Vector3 a,Vector3 b,ISet<int> forbidden=null)
        {
            if(!TryCell(a,out int first) || !TryCell(b,out int last)) return false;
            Cell(first,out int x,out int y,out int z); Cell(last,out int ex,out int ey,out int ez);
            if(!IsFree(x,y,z,forbidden) || !IsFree(ex,ey,ez,forbidden)) return false;
            Vector3 p=Coordinates(a), d=Coordinates(b)-p;
            int[] cell={x,y,z}, end={ex,ey,ez}, sign={Math.Sign(d.x),Math.Sign(d.y),Math.Sign(d.z)};
            double[] next=new double[3], stride=new double[3];
            for(int k=0;k<3;k++)
            {
                stride[k]=sign[k]==0?double.PositiveInfinity:Math.Abs(1.0/d[k]);
                next[k]=sign[k]==0?double.PositiveInfinity:((sign[k]>0?cell[k]+1:cell[k])-p[k])/d[k];
            }
            for(int steps=0;steps<CountX+CountY+CountZ+3;steps++)
            {
                if(cell[0]==end[0]&&cell[1]==end[1]&&cell[2]==end[2]) return true;
                double t=Math.Min(next[0],Math.Min(next[1],next[2]));
                int mask=0;
                for(int k=0;k<3;k++) if(next[k]<=t+1e-7) mask|=1<<k;
                for(int subset=1;subset<8;subset++)
                    if((subset&mask)==subset && !IsFree(cell[0]+((subset&1)!=0?sign[0]:0),
                        cell[1]+((subset&2)!=0?sign[1]:0),cell[2]+((subset&4)!=0?sign[2]:0),forbidden)) return false;
                for(int k=0;k<3;k++) if((mask&(1<<k))!=0) { cell[k]+=sign[k]; next[k]+=stride[k]; }
            }
            return false;
        }
    }
}
