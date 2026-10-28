using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace RobotIK.Editor
{
    public static class RobotHosePhysicsChecks
    {
        [MenuItem("Tools/Robot IK/Validate Robot Hose Physics",true)]
        private static bool CanRun() => !EditorApplication.isPlayingOrWillChangePlaymode &&
            !string.IsNullOrEmpty(SceneManager.GetActiveScene().path);

        private static void Check(bool value,string message)
        { if(!value) throw new Exception(message); }

        [MenuItem("Tools/Robot IK/Validate Robot Hose Physics")]
        public static void Run()
        {
            var original=SceneManager.GetActiveScene();
            var scene=EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Additive);
            SceneManager.SetActiveScene(scene);
            try
            {
                FeedChecks();
                OptimizationChecks();
                IntegrationChecks(scene);
                ContactCacheChecks(scene);
                RobotHoseReelChecks.Run(scene);
                Debug.Log("ROBOT_HOSE_PHYSICS_CHECKS_PASSED: inlet remeshing, winding, length conservation, contact, clamps, stall, renderer and cleanup.");
            }
            finally
            {
                SceneManager.SetActiveScene(original);
                EditorSceneManager.CloseScene(scene,true);
            }
        }
        public static void RunBatch()
        {
            try { Debug.Log(ElasticCableRodChecks.RunNumerical()); Run(); EditorApplication.Exit(0); }
            catch(Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        private static void FeedChecks()
        {
            var points=new Vector3[21];
            for(int i=0;i<points.Length;i++) points[i]=new Vector3(0,0,i*0.05f);
            var rod=new ElasticCableRod(points,0.015f,0.3f);
            rod.SetPositionPin(0,points[0]); rod.SetPositionPin(20,points[20]);
            rod.SetFramePin(0,rod.Frames[0]); Quaternion end=rod.Frames[19];
            for(int i=1;i<=120;i++)
            {
                rod.SetFramePin(19,RodMath.Exp(Vector3.forward*(4*Mathf.PI*i/120))*end);
                rod.Step(0.02f,4,24,Vector3.zero);
            }
            float turns=rod.TotalTwistRadians();
            var added=rod.FeedAtStart(0.005f,0.05f,0.3f);
            Check(Mathf.Abs(added.TotalTwistRadians()-turns)<0.001f,"Feed lost multi-turn winding without remeshing");
            Check(Vector3.Distance(added.Positions[10],rod.Positions[10])<1e-5f,"Feed moved downstream material");

            rod=new ElasticCableRod(points,0.015f,0.3f);
            for(int cycle=0;cycle<5;cycle++)
            {
                for(int i=0;i<80;i++) rod=rod.FeedAtStart(-0.005f,0.05f,0.3f);
                Check(Mathf.Abs(rod.RestLength-0.6f)<0.0001f,"Retraction length accounting");
                Check(rod.EdgeCount<20,"Retraction did not remove inlet nodes");
                for(int i=0;i<80;i++) rod=rod.FeedAtStart(0.005f,0.05f,0.3f);
                Check(Mathf.Abs(rod.RestLength-1)<0.0001f,"Repeated feed length drift");
                Check(Vector3.Distance(rod.Positions[rod.EdgeCount],points[20])<1e-5f,"Feed changed end attachment");
                foreach(float length in rod.RestLengths) Check(length>0.0001f,"Degenerate feed element");
            }
        }

        private static void IntegrationChecks(Scene scene)
        {
            var start=new GameObject("Start").transform;
            var clamp=new GameObject("Clamp").transform;
            var end=new GameObject("End").transform;
            start.position=new Vector3(0,1,0); clamp.position=new Vector3(1,1,0); end.position=new Vector3(1.5f,1,0);
            var first=new Vector3[21]; var second=new Vector3[11];
            for(int i=0;i<first.Length;i++) first[i]=Vector3.Lerp(start.position,clamp.position,i/20f)+Vector3.down*(0.3f*Mathf.Sin(Mathf.PI*i/20));
            for(int i=0;i<second.Length;i++) second[i]=Vector3.Lerp(clamp.position,end.position,i/10f);
            float initial=ElasticCableRouting.Length(first)+ElasticCableRouting.Length(second);
            var settings=new RobotHosePhysicsSettings { cableLength=initial+0.2f,initialDeployedLength=initial,
                targetDeployedLength=initial-0.08f,feedSpeed=0.04f,bendRigidity=0.001f,targetSegmentLength=0 };
            var floor=GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.transform.position=new Vector3(0.5f,0.55f,0); floor.transform.localScale=new Vector3(3,0.1f,3);
            Physics.SyncTransforms();
            using(var simulation=new RobotHoseSimulation(new[]{start,clamp,end},new[]{first,second},0.015f,settings,0,scene))
            {
                var positions=new List<Vector3>(); var frames=new List<Quaternion>(); var distances=new List<float>();
                for(int i=0;i<160;i++) simulation.Step(0.02f);
                simulation.GetRenderSamples(positions,frames,distances,1);
                Check(Vector3.Distance(positions[0],start.position)<1e-5f,"Start detached");
                Check(Vector3.Distance(positions[positions.Count-1],end.position)<1e-5f,"End detached");
                Check(positions.Exists(p=>Vector3.Distance(p,clamp.position)<1e-5f),"Midpoint clamp detached");
                Check(simulation.DeployedLength<initial-0.01f,"Retraction made no progress");
                Check(Mathf.Abs(simulation.DeployedLength+simulation.StoredLength-settings.cableLength)<1e-5f,"Total cable not conserved");
                foreach(var p in positions) Check(p.y>=0.612f,"Cable penetrated floor: "+p);
                settings.targetDeployedLength=0.1f;
                settings.feedSpeed=0.5f;
                for(int i=0;i<160;i++) simulation.Step(0.02f);
                Check(simulation.RetractionBlocked,"Impossible length did not stall");
                settings.targetDeployedLength=settings.cableLength;
                for(int i=0;i<160;i++) simulation.Step(0.02f);
                Check(Mathf.Abs(simulation.DeployedLength-settings.cableLength)<0.001f,"Extension did not reach available total");
                Check(simulation.BroadphaseQueries<40,"Collision broadphase repeated at every element/iteration");
                for(int i=0;i<20;i++) simulation.Step(0.02f);
                long allocationStart=GC.GetAllocatedBytesForCurrentThread();
                for(int i=0;i<20;i++) simulation.Step(0.02f);
                long allocated=GC.GetAllocatedBytesForCurrentThread()-allocationStart;
                Check(allocated==0,"Steady cable simulation allocated managed memory: "+allocated);
                Debug.Log("HOSE_CONTACT_COUNTS: elements="+simulation.SegmentCount+", broadphase="+simulation.BroadphaseQueries+
                    ", sweeps="+simulation.SweepQueries+", penetration="+simulation.PenetrationQueries);

                var owner=new GameObject("Renderer"); owner.SetActive(false);
                var hose=owner.AddComponent<RobotHose>(); hose.startPoint=start; hose.endPoint=end; hose.midPoints.Add(clamp);
                owner.SetActive(true);
                typeof(RobotHose).GetField("simulation",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(hose,simulation);
                typeof(RobotHose).GetMethod("UpdateSimulatedHose",BindingFlags.Instance|BindingFlags.NonPublic).Invoke(hose,null);
                var mesh=owner.GetComponent<MeshFilter>().sharedMesh;
                Check(mesh.vertexCount>0 && mesh.triangles.Length>0,"Physics tube mesh missing");
                Check(mesh.vertexCount>simulation.SegmentCount*(hose.radialSegments+1),"Rendering resolution was not independent of physics");
                foreach(var p in mesh.vertices) Check(!float.IsNaN(p.x) && !float.IsInfinity(p.x),"Nonfinite mesh");
                UnityEngine.Object.DestroyImmediate(owner);
            }
            foreach(var collider in UnityEngine.Object.FindObjectsByType<CapsuleCollider>(FindObjectsSortMode.None))
                Check(collider.gameObject.name!="Hose contact probes","Contact probe leaked");
        }

        private static void OptimizationChecks()
        {
            var points=ElasticCableRouting.Build(new[]{new Vector3(-0.6f,1,0),new Vector3(0.6f,1,0)},1.6f,47,Vector3.up);
            var legacy=StiffFixture(points,false);
            var coupled=StiffFixture(points,true);
            var timer=System.Diagnostics.Stopwatch.StartNew();
            for(int i=0;i<100;i++) legacy.Step(0.02f,2,10,Physics.gravity);
            double legacyMs=timer.Elapsed.TotalMilliseconds;
            timer.Restart();
            for(int i=0;i<100;i++) coupled.Step(0.02f,2,10,Physics.gravity);
            double coupledMs=timer.Elapsed.TotalMilliseconds;
            float legacyDrift=Vector3.Distance(points[points.Length/2],legacy.Positions[points.Length/2]);
            float coupledDrift=Vector3.Distance(points[points.Length/2],coupled.Positions[points.Length/2]);
            Debug.Log("HOSE_LOW_BUDGET_BEND: old drift="+legacyDrift+", coupled drift="+coupledDrift+
                ", strain="+coupled.MaximumRelativeStretch()+", old ms="+legacyMs+", coupled ms="+coupledMs);
            Check(!float.IsNaN(coupledDrift) && coupledDrift<0.02f,"Coupled bending failed to retain stiff rest shape at 2/10");
            Check(coupledDrift<legacyDrift,"Coupled bending did not improve low-iteration stiffness");

            // A sparse straight cable should never enter the self-contact narrowphase.
            var line=new Vector3[95];
            for(int i=0;i<line.Length;i++) line[i]=new Vector3(i*0.05f,4,0);
            var sparse=new ElasticCableRod(line,0.015f,0.2f) { SelfCollision=true };
            sparse.Step(0.02f,2,10,Vector3.zero);
            Check(sparse.SelfCollisionPairsTested==0,"Self-collision broadphase tested distant elements");

            var crossing=new[]{new Vector3(-0.3f,0,0),new Vector3(-0.2f,0,0),new Vector3(-0.1f,0,0),
                new Vector3(0.1f,0,0),new Vector3(0.2f,0,0),new Vector3(0.3f,0,0),new Vector3(0.3f,0.2f,0.01f),
                new Vector3(0.2f,0.2f,0.01f),new Vector3(0.1f,0.2f,0.01f),new Vector3(0,0.1f,0.01f),new Vector3(0,-0.1f,0.01f)};
            var crossed=new ElasticCableRod(crossing,0.02f,0.2f);
            var resolve=typeof(ElasticCableRod).GetMethod("ResolveSelfContacts",BindingFlags.Instance|BindingFlags.NonPublic);
            for(int i=0;i<4;i++) resolve.Invoke(crossed,null);
            RodMath.ClosestSegments(crossed.Positions[2],crossed.Positions[3],crossed.Positions[9],crossed.Positions[10],out float s,out float t);
            float separation=Vector3.Distance(Vector3.Lerp(crossed.Positions[2],crossed.Positions[3],s),Vector3.Lerp(crossed.Positions[9],crossed.Positions[10],t));
            Check(crossed.SelfCollisionPairsTested>0 && separation>0.039f,"Self broadphase missed crossing cable edges");

            var moving=StiffFixture(points,true);
            for(int i=0;i<80;i++)
            {
                float angle=0.25f*i/80;
                Quaternion rotation=RodMath.Exp(Vector3.up*angle);
                moving.SetPositionPin(0,rotation*points[0]); moving.SetPositionPin(moving.EdgeCount,rotation*points[points.Length-1]);
                moving.SetFramePin(0,rotation*coupled.Frames[0]); moving.SetFramePin(moving.EdgeCount-1,rotation*coupled.Frames[coupled.EdgeCount-1]);
                moving.Step(0.02f,2,10,Vector3.zero);
            }
            Check(moving.MaximumRelativeStretch()<0.05f,"Coupled rod stretched during attachment motion");

            // A 30 mm bend is allowed by contact even with a stiff rod. The final
            // collision pass must keep the centerline outside the floor.
            var stiffContact=StiffFixture(points,true);
            float floorY=points[points.Length/2].y+0.03f;
            stiffContact.ProjectContacts=r=>{
                for(int i=1;i<r.Positions.Length-1;i++)
                    if(Mathf.Abs(r.Positions[i].x)<0.15f && r.Positions[i].y<floorY)
                        r.ResolveNodeContact(i,Vector3.up,floorY-r.Positions[i].y);
            };
            for(int i=0;i<30;i++) stiffContact.Step(0.02f,2,10,Vector3.zero);
            for(int i=1;i<stiffContact.Positions.Length-1;i++)
                if(Mathf.Abs(stiffContact.Positions[i].x)<0.15f)
                    Check(stiffContact.Positions[i].y>=floorY-0.001f,"Stiff curvature solver overrode contact");

            var twistPoints=new Vector3[21];
            for(int i=0;i<twistPoints.Length;i++) twistPoints[i]=Vector3.forward*(i*0.05f);
            var twist=StiffFixture(twistPoints,true);
            Quaternion initialEnd=twist.Frames[twist.EdgeCount-1];
            for(int i=1;i<=120;i++)
            {
                twist.SetFramePin(twist.EdgeCount-1,RodMath.Exp(Vector3.forward*(4*Mathf.PI*i/120))*initialEnd);
                twist.Step(0.02f,2,10,Vector3.zero);
            }
            Check(Mathf.Abs(twist.TotalTwistRadians()-4*Mathf.PI)<0.15f,"Optimized torsion lost winding at 2/10");
            for(int i=0;i<5;i++) twist=twist.FeedAtStart(0.01f,0.05f,0.2f);
            Check(twist.CoupledBending && Mathf.Abs(twist.TotalTwistRadians()-4*Mathf.PI)<0.15f,"Remeshing lost optimized rod settings or twist history");
        }

        private static ElasticCableRod StiffFixture(Vector3[] points,bool coupled)
        {
            var rod=new ElasticCableRod(points,0.015f,0.2f) { CoupledBending=coupled,
                BendRigidity=100000,AxialRigidity=10000000,TwistRigidity=100,Damping=5 };
            rod.SetPositionPin(0,points[0]); rod.SetPositionPin(rod.EdgeCount,points[points.Length-1]);
            rod.SetFramePin(0,rod.Frames[0]); rod.SetFramePin(rod.EdgeCount-1,rod.Frames[rod.EdgeCount-1]);
            return rod;
        }

        private static void ContactCacheChecks(Scene scene)
        {
            var start=new GameObject("Cache start").transform;
            var end=new GameObject("Cache end").transform;
            start.position=new Vector3(100,2,0); end.position=new Vector3(101,2,0);
            var route=new Vector3[5];
            for(int i=0;i<route.Length;i++) route[i]=Vector3.Lerp(start.position,end.position,i/4f);
            var floor=GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.transform.position=new Vector3(100.5f,0.5f,0); floor.transform.localScale=new Vector3(2,0.2f,2);
            Physics.SyncTransforms();
            using(var simulation=new RobotHoseSimulation(new[]{start,end},new[]{route},0.015f,new RobotHosePhysicsSettings(),0,scene))
            {
                var spans=(Array)typeof(RobotHoseSimulation).GetField("spans",BindingFlags.Instance|BindingFlags.NonPublic).GetValue(simulation);
                var span=spans.GetValue(0);
                var rod=(ElasticCableRod)span.GetType().GetField("rod").GetValue(span);
                Array.Copy(rod.Positions,rod.SubstepStartPositions,rod.Positions.Length);
                rod.PrepareContacts(rod);
                int middle=rod.Positions.Length/2;
                rod.Positions[middle]=new Vector3(100.5f,0.59f,0);
                rod.ProjectContacts(rod);
                Check(simulation.BroadphaseQueries>=2,"Collision cache did not refresh outside its envelope");
                Check(rod.Positions[middle].y>0.615f,"Refreshed cache missed newly reached floor");

                floor.transform.position+=Vector3.right*20;
                Physics.SyncTransforms();
                rod.PrepareContacts(rod);
                int before=simulation.PenetrationQueries;
                rod.ProjectContacts(rod);
                Check(simulation.PenetrationQueries==before,"Moved collider remained in the next substep's cache");
            }
        }
    }
}
