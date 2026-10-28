using System;
using System.Reflection;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace RobotIK.Editor
{
    public static class RobotHoseReelChecks
    {
        private static void Check(bool value,string message) { if(!value) throw new Exception(message); }
        private static ElasticCableRod Rod(RobotHoseSimulation simulation,int index)
        {
            var spans=(Array)typeof(RobotHoseSimulation).GetField("spans",BindingFlags.Instance|BindingFlags.NonPublic).GetValue(simulation);
            var span=spans.GetValue(index);
            return (ElasticCableRod)span.GetType().GetField("rod").GetValue(span);
        }
        private static Vector3[] Arc(Vector3 start,Vector3 end)
        {
            var points=new Vector3[21];
            for(int i=0;i<points.Length;i++) points[i]=Vector3.Lerp(start,end,i/20f)+Vector3.up*(0.12f*Mathf.Sin(Mathf.PI*i/20));
            return points;
        }
        private static Transform Anchor(Vector3 position)
        {
            var value=new GameObject("Reel test anchor").transform; value.position=position; return value;
        }
        private static RobotHosePhysicsSettings Settings(float initial,float total)
        {
            return new RobotHosePhysicsSettings { cableLength=total,initialDeployedLength=initial,
                bendRigidity=0.002f,damping=5,useGravity=false,stallStrain=0.08f,
                reel=new RobotHoseReelSettings { automatic=true,desiredSlack=0.06f,slackTolerance=0.01f,
                    extensionSpeed=0.3f,retractionSpeed=0.15f,guideSlideSpeed=0.3f,acceleration=1,retractionDelay=0.1f } };
        }
        private static void Advance(RobotHoseSimulation simulation,int ticks)
        {
            for(int i=0;i<ticks;i++)
            {
                simulation.Step(0.02f);
                Check(simulation.DeployedLength<=simulation.TotalLength+0.0002f,"Reel exceeded total material budget");
                Check(Mathf.Abs(simulation.DeployedLength+simulation.StoredLength-simulation.TotalLength)<0.0002f,"Reel material conservation");
            }
        }
        public static void Run(Scene scene)
        {
            EndFeedChecks(); PolicyChecks(); AutomaticMotion(scene); GuideMotion(scene); MixedClamps(scene); GuideTwist(scene); ContactRoute(scene);
            Debug.Log("ROBOT_HOSE_REEL_CHECKS_PASSED: automatic extension/recovery, total limit, two-end remeshing, guide transfer, fixed clamps and material conservation.");
        }
        private static void EndFeedChecks()
        {
            var points=new Vector3[21]; for(int i=0;i<points.Length;i++) points[i]=Vector3.forward*(i*0.05f);
            var rod=new ElasticCableRod(points,0.015f,0.2f) { CoupledBending=true };
            for(int cycle=0;cycle<3;cycle++)
            {
                for(int i=0;i<70;i++) rod=rod.FeedAtEnd(-0.005f,0.05f,0.2f);
                Check(Mathf.Abs(rod.RestLength-0.65f)<0.0001f,"Outlet consumption accounting");
                for(int i=0;i<70;i++) rod=rod.FeedAtEnd(0.005f,0.05f,0.2f);
                Check(Mathf.Abs(rod.RestLength-1)<0.0001f && rod.CoupledBending,"Outlet remeshing reset metrics");
                Check(Vector3.Distance(rod.Positions[0],points[0])<1e-6f && Vector3.Distance(rod.Positions[rod.EdgeCount],points[20])<1e-6f,"Outlet remeshing displaced attachments");
            }
        }
        private static void PolicyChecks()
        {
            var settings=new RobotHoseReelSettings { automatic=true,retractionDelay=0.1f };
            var policy=new RobotHoseReelController(new[]{true,false},new[]{1f,1f,1f},settings);
            var demand=new[]{new RobotHoseSpanDemand { length=1,requiredLength=0.8f },
                new RobotHoseSpanDemand { length=1,requiredLength=0.8f },new RobotHoseSpanDemand { length=1,requiredLength=0.8f }};
            float delta=0;
            for(int i=0;i<20;i++) delta=policy.PlanReel(4,3,0,0.1f,demand,0.02f);
            Check(delta<0 && policy.ReelSpanCount==2,"Automatic recovery or clamp partition failed");
            var flows=new float[2]; policy.PlanGuides(demand,-0.02f,flows);
            Check(flows[0]<0 && flows[1]==0,"Guide did not fund recovery or crossed fixed clamp");
            demand[1].requiredLength=1.4f;
            Check(policy.PlanReel(4,3,0,0.1f,demand,0.02f)>=0,"Required reach did not request payout");
            policy.PlanReel(3,3,0,0.1f,demand,0.02f);
            Check(policy.State==RobotHoseReelState.TotalLengthReached,"Exhausted reel did not report total limit");
            demand[2].requiredLength=1.4f; policy.PlanGuides(demand,0,flows); policy.FinishStep();
            Check(policy.State==RobotHoseReelState.FixedSpanOverextended,"Fixed downstream group not reported");
        }
        private static void AutomaticMotion(Scene scene)
        {
            var start=Anchor(new Vector3(200,5,0)); var end=Anchor(new Vector3(201,5,0));
            var points=Arc(start.position,end.position); float initial=ElasticCableRouting.Length(points);
            var settings=Settings(initial,initial+0.5f); settings.bendRigidity=2;
            using(var simulation=new RobotHoseSimulation(new[]{start,end},new[]{points},0.015f,settings,0,scene))
            {
                end.position+=Vector3.right*0.3f; Advance(simulation,200);
                float extended=simulation.DeployedLength;
                Check(extended>initial+0.08f,"Moving robot did not automatically extend cable");
                end.position-=Vector3.right*0.55f; Advance(simulation,250);
                Check(simulation.DeployedLength<extended-0.05f,"Moving robot closer did not recover cable");
                end.position+=Vector3.right*3; Advance(simulation,300);
                Check(simulation.ReelState==RobotHoseReelState.TotalLengthReached,"Impossible robot reach did not report exhausted cable");
                AssertFinite(simulation);
            }
        }
        private static void GuideMotion(Scene scene)
        {
            var start=Anchor(new Vector3(210,5,0)); var guide=Anchor(new Vector3(210.8f,5,0)); var end=Anchor(new Vector3(211.6f,5,0));
            var first=Arc(start.position,guide.position); var second=Arc(guide.position,end.position);
            float initial=ElasticCableRouting.Length(first)+ElasticCableRouting.Length(second);
            var settings=Settings(initial,initial+0.6f);
            using(var simulation=new RobotHoseSimulation(new[]{start,guide,end},new[]{first,second},0.015f,settings,0,scene,new[]{RobotHoseMidpointMode.SlidingGuide}))
            {
                float originalSecond=Rod(simulation,1).RestLength;
                end.position+=Vector3.right*0.3f; Advance(simulation,250);
                Check(Rod(simulation,1).RestLength>originalSecond+0.1f,"Sliding guide did not supply downstream span");
                Check(Vector3.Distance(Rod(simulation,0).Positions[Rod(simulation,0).EdgeCount],guide.position)<1e-5f &&
                    Vector3.Distance(Rod(simulation,1).Positions[0],guide.position)<1e-5f,"Sliding guide lost spatial attachment");
                float extended=simulation.DeployedLength;
                end.position-=Vector3.right*0.55f; Advance(simulation,300);
                Check(simulation.DeployedLength<extended-0.05f,"Cable did not recover through sliding guide");
                Check(Rod(simulation,1).RestLength<originalSecond+0.1f,"Downstream material did not return through guide");
                settings.reel.automatic=false; settings.targetDeployedLength=simulation.DeployedLength;
                AssertFinite(simulation);
                settings.reel.guideSlideSpeed=0;
                Advance(simulation,10);
                long allocation=GC.GetAllocatedBytesForCurrentThread(); Advance(simulation,20);
                Check(GC.GetAllocatedBytesForCurrentThread()==allocation,"Steady reel/guide controller allocated memory");
                Debug.Log("HOSE_GUIDE_RESULT: downstream="+Rod(simulation,1).RestLength+", deployed="+simulation.DeployedLength+", state="+simulation.ReelState);
            }
        }
        private static void MixedClamps(Scene scene)
        {
            var anchors=new[]{Anchor(new Vector3(220,5,0)),Anchor(new Vector3(220.8f,5,0)),
                Anchor(new Vector3(221.6f,5,0)),Anchor(new Vector3(222.4f,5,0))};
            var routes=new[]{Arc(anchors[0].position,anchors[1].position),Arc(anchors[1].position,anchors[2].position),Arc(anchors[2].position,anchors[3].position)};
            float initial=0; foreach(var route in routes) initial+=ElasticCableRouting.Length(route);
            using(var simulation=new RobotHoseSimulation(anchors,routes,0.015f,Settings(initial,initial+0.3f),0,scene,
                new[]{RobotHoseMidpointMode.SlidingGuide,RobotHoseMidpointMode.FixedClamp}))
            {
                float fixedLength=Rod(simulation,2).RestLength;
                anchors[3].position+=Vector3.right*0.4f; Advance(simulation,100);
                Check(Mathf.Abs(Rod(simulation,2).RestLength-fixedLength)<1e-6f,"Material crossed fixed clamp");
                Check(simulation.ReelConnectedSpanCount==2 && simulation.ReelState==RobotHoseReelState.FixedSpanOverextended,"Mixed guide/clamp partition failed");
                AssertFinite(simulation);
            }
        }
        private static void GuideTwist(Scene scene)
        {
            var anchors=new[]{Anchor(new Vector3(230,5,0)),Anchor(new Vector3(230.8f,5,0)),Anchor(new Vector3(231.6f,5,0))};
            var routes=new Vector3[2][];
            for(int i=0;i<2;i++)
            {
                routes[i]=new Vector3[17];
                for(int j=0;j<17;j++) routes[i][j]=Vector3.Lerp(anchors[i].position,anchors[i+1].position,j/16f);
            }
            var settings=new RobotHosePhysicsSettings { useGravity=false,reel=new RobotHoseReelSettings { guideSlideSpeed=0 } };
            using(var simulation=new RobotHoseSimulation(anchors,routes,0.015f,settings,0,scene,new[]{RobotHoseMidpointMode.SlidingGuide}))
            {
                for(int i=1;i<=240;i++)
                {
                    anchors[2].rotation=RodMath.Exp(Vector3.right*(4*Mathf.PI*i/240));
                    simulation.Step(0.02f);
                }
                Advance(simulation,100);
                float twist=Rod(simulation,0).TotalTwistRadians()+Rod(simulation,1).TotalTwistRadians();
                Check(Mathf.Abs(twist-4*Mathf.PI)<0.3f,"Sliding guide lost multi-turn material winding: "+twist);
                float initialFirst=Rod(simulation,0).RestLength;
                settings.reel.guideSlideSpeed=0.2f;
                for(int i=0;i<120;i++)
                {
                    anchors[1].position+=Vector3.right*0.0005f;
                    simulation.Step(0.02f);
                }
                Advance(simulation,100);
                twist=Rod(simulation,0).TotalTwistRadians()+Rod(simulation,1).TotalTwistRadians();
                Check(Rod(simulation,0).RestLength>initialFirst+0.035f,"Moving a guide did not transfer wound material");
                Check(Mathf.Abs(twist-4*Mathf.PI)<0.5f,"Guide material transfer lost winding: "+twist);
            }
        }
        private static void AssertFinite(RobotHoseSimulation simulation)
        {
            var spans=(Array)typeof(RobotHoseSimulation).GetField("spans",BindingFlags.Instance|BindingFlags.NonPublic).GetValue(simulation);
            for(int i=0;i<spans.Length;i++) foreach(var point in Rod(simulation,i).Positions)
                Check(!float.IsNaN(point.sqrMagnitude) && !float.IsInfinity(point.sqrMagnitude),"Cable became nonfinite under constrained robot motion");
        }
        private static void ContactRoute(Scene scene)
        {
            var anchors=new[]{Anchor(new Vector3(240,5,0)),Anchor(new Vector3(241,5,0))};
            var route=new Vector3[21];
            for(int i=0;i<route.Length;i++) route[i]=Vector3.Lerp(anchors[0].position,anchors[1].position,i/20f);
            using(var simulation=new RobotHoseSimulation(anchors,new[]{route},0.015f,Settings(1,1.5f),0,scene))
            {
                var rod=Rod(simulation,0); int middle=rod.Positions.Length/2;
                rod.Positions[middle]+=Vector3.up*0.3f;
                rod.ResolveNodeContact(middle,Vector3.up,0.001f);
                simulation.Step(0.02f);
                Check(simulation.RequiredDeployedLength>1.1f,"Automatic route estimate ignored obstacle contact detour");
            }
        }
    }
}
