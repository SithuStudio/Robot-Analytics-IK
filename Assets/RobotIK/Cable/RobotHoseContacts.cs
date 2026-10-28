using System;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Profiling;

namespace RobotIK
{
    /// <summary>
    /// One-way rod contact with Unity colliders. Broadphase candidates are cached
    /// per substep and refreshed whenever constraint motion leaves their envelope.
    /// </summary>
    internal sealed class RobotHoseContacts : IDisposable
    {
        private readonly float radius;
        private readonly int layer;
        private readonly RobotHosePhysicsSettings settings;
        private readonly GameObject root;
        private readonly CapsuleCollider capsule;
        private readonly SphereCollider sphere;
        private Collider[] candidates = new Collider[128];
        private Bounds[] candidateBounds = new Bounds[128];
        private RaycastHit[] sweeps = new RaycastHit[32];
        private Vector3[] sweepPositions;
        private int candidateCount;
        private Bounds envelope;
        private ElasticCableRod cachedRod;
        private static readonly ProfilerMarker ContactMarker=new ProfilerMarker("RobotHose.Contacts");
        public int BroadphaseQueries { get; private set; }
        public int SweepQueries { get; private set; }
        public int PenetrationQueries { get; private set; }

        public RobotHoseContacts(float radius, int layer, Scene scene, RobotHosePhysicsSettings settings)
        {
            this.radius=radius; this.layer=layer; this.settings=settings;
            root=new GameObject("Hose contact probes") { hideFlags=HideFlags.HideAndDontSave, layer=layer };
            SceneManager.MoveGameObjectToScene(root,scene);
            root.transform.position=Vector3.one*1000000;
            capsule=root.AddComponent<CapsuleCollider>(); capsule.direction=2; capsule.radius=radius; capsule.isTrigger=true;
            sphere=root.AddComponent<SphereCollider>(); sphere.radius=radius; sphere.isTrigger=true;
        }

        public void ResetCounters() { BroadphaseQueries=0; SweepQueries=0; PenetrationQueries=0; }

        private bool Blocks(Collider other)
        {
            if(!other || other.isTrigger || other.gameObject==root || Physics.GetIgnoreLayerCollision(layer,other.gameObject.layer)) return false;
            if(settings.ignoredColliders!=null)
                foreach(var ignored in settings.ignoredColliders) if(other==ignored) return false;
            return true;
        }

        public void BeginSubstep(ElasticCableRod rod)
        {
            cachedRod=rod;
            if(sweepPositions==null || sweepPositions.Length<rod.Positions.Length)
                sweepPositions=new Vector3[Mathf.NextPowerOfTwo(rod.Positions.Length)];
            Array.Copy(rod.SubstepStartPositions,sweepPositions,rod.Positions.Length);
            RefreshCandidates(rod);
        }

        private void RefreshCandidates(ElasticCableRod rod)
        {
            envelope=new Bounds(rod.Positions[0],Vector3.zero);
            foreach(var point in rod.Positions) envelope.Encapsulate(point);
            foreach(var point in rod.SubstepStartPositions) envelope.Encapsulate(point);
            envelope.Expand(radius*8);
            int count;
            // Grow only on saturation, outside the normal allocation-free path.
            // No fixed-capacity truncation of dense environments.
            while(true)
            {
                BroadphaseQueries++;
                count=Physics.OverlapBoxNonAlloc(envelope.center,envelope.extents,candidates,Quaternion.identity,settings.collisionLayers,QueryTriggerInteraction.Ignore);
                if(count<candidates.Length) break;
                Array.Resize(ref candidates,candidates.Length*2);
                Array.Resize(ref candidateBounds,candidates.Length);
            }
            candidateCount=0;
            for(int i=0;i<count;i++)
            {
                Collider other=candidates[i]; if(!Blocks(other)) continue;
                candidates[candidateCount]=other;
                candidateBounds[candidateCount]=other.bounds;
                candidateCount++;
            }
        }

        private void EnsureEnvelope(ElasticCableRod rod)
        {
            Bounds interior=envelope; interior.Expand(-radius*2);
            foreach(var point in rod.Positions)
                if(!interior.Contains(point)) { RefreshCandidates(rod); return; }
        }

        private static Bounds EdgeBounds(Vector3 a,Vector3 b,float padding)
        {
            var bounds=new Bounds((a+b)*0.5f,Vector3.zero);
            bounds.SetMinMax(Vector3.Min(a,b)-Vector3.one*padding,Vector3.Max(a,b)+Vector3.one*padding);
            return bounds;
        }

        public void Project(ElasticCableRod rod)
        {
            using(ContactMarker.Auto()) ProjectContacts(rod);
        }

        private void ProjectContacts(ElasticCableRod rod)
        {
            if(cachedRod!=rod) BeginSubstep(rod);
            EnsureEnvelope(rod);
            float skin=Mathf.Min(0.001f,radius*0.05f);
            for(int i=0;i<rod.Positions.Length;i++)
            {
                if(rod.IsPinned(i)) continue;
                Vector3 from=sweepPositions[i], delta=rod.Positions[i]-from;
                float length=delta.magnitude;
                if(length<radius*0.25f) continue;
                // Re-sweep only new motion, rather than the same substep trajectory
                // on every collision iteration. Sweeps still use Unity's broadphase
                // so fast motion is not limited to the local candidate envelope.
                int count;
                while(true)
                {
                    SweepQueries++;
                    count=Physics.SphereCastNonAlloc(from,radius,delta/length,sweeps,length,settings.collisionLayers,QueryTriggerInteraction.Ignore);
                    if(count<sweeps.Length) break;
                    Array.Resize(ref sweeps,sweeps.Length*2);
                }
                float nearest=float.PositiveInfinity; Vector3 normal=Vector3.zero;
                for(int j=0;j<count;j++) if(Blocks(sweeps[j].collider) && sweeps[j].distance<nearest)
                { nearest=sweeps[j].distance; normal=sweeps[j].normal; }
                if(!float.IsPositiveInfinity(nearest))
                {
                    Vector3 stopped=from+delta/length*Mathf.Max(0,nearest-skin);
                    rod.ResolveNodeContact(i,normal,Mathf.Max(0,Vector3.Dot(stopped-rod.Positions[i],normal)));
                }
                sweepPositions[i]=rod.Positions[i];
            }
            EnsureEnvelope(rod);
            for(int i=0;i<rod.EdgeCount;i++)
            {
                for(int j=0;j<candidateCount;j++)
                {
                    Collider other=candidates[j]; if(!other) continue;
                    Vector3 a=rod.Positions[i], b=rod.Positions[i+1], delta=b-a;
                    if(!candidateBounds[j].Intersects(EdgeBounds(a,b,radius+skin))) continue;
                    float length=delta.magnitude; if(length<1e-7f) break;
                    capsule.height=length+2*radius;
                    Vector3 middle=(a+b)*0.5f;
                    PenetrationQueries++;
                    if(!Physics.ComputePenetration(capsule,middle,RodMath.FromTo(Vector3.forward,delta/length),
                        other,other.transform.position,other.transform.rotation,out Vector3 normal,out float depth)) continue;
                    float t=Mathf.Clamp01(Vector3.Dot(other.ClosestPoint(middle)-a,delta)/(length*length));
                    rod.ResolveEdgeContact(i,t,normal,depth+skin);
                }
            }
            // Earlier penetration corrections can leave the envelope. Refresh before
            // node cleanup to include any newly reached geometry.
            EnsureEnvelope(rod);
            for(int i=0;i<rod.Positions.Length;i++)
            {
                if(rod.IsPinned(i)) continue;
                for(int j=0;j<candidateCount;j++)
                {
                    Collider other=candidates[j]; if(!other) continue;
                    var bounds=new Bounds(rod.Positions[i],Vector3.one*(2*(radius+skin)));
                    if(!candidateBounds[j].Intersects(bounds)) continue;
                    PenetrationQueries++;
                    if(Physics.ComputePenetration(sphere,rod.Positions[i],Quaternion.identity,other,other.transform.position,other.transform.rotation,
                        out Vector3 normal,out float depth)) rod.ResolveNodeContact(i,normal,depth+skin);
                }
            }
        }
        public void Dispose()
        {
            if(!root) return;
            root.SetActive(false);
            if(Application.isPlaying) UnityEngine.Object.Destroy(root); else UnityEngine.Object.DestroyImmediate(root);
        }
    }
}
