using System;
using RobotIK;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using System.Reflection;
namespace RobotIK.Editor
{
public static class ElasticCableRodChecks
{
    [MenuItem("Tools/Robot IK/Validate Elastic Rod %#F10")]
    public static void RunInEditor()
    {
        string report;
        try
        {
            report = RunNumerical();
            RunUnityIntegration();
            report += "UNITY_ELASTIC_ROD_CHECKS_PASSED: rendering, editable routes, twist stripes, collider contact, cleanup.\n";
            Debug.Log(report);
        }
        catch(Exception error)
        {
            report = "UNITY_ELASTIC_ROD_CHECKS_FAILED: " + error;
            Debug.LogException(error);
        }
        System.IO.Directory.CreateDirectory("Temp/CableHoseChecks");
        System.IO.File.WriteAllText("Temp/CableHoseChecks/UnityRodChecks.txt",report);
    }
    [MenuItem("Tools/Robot IK/Validate Elastic Rod %#F10", true)]
    private static bool CanRunInEditor() => !EditorApplication.isPlayingOrWillChangePlaymode;
    private static object Private(object target,string name) => target.GetType().GetField(name,BindingFlags.NonPublic|BindingFlags.Instance).GetValue(target);
    private static void Call(object target,string name,params object[] arguments) => target.GetType().GetMethod(name,BindingFlags.NonPublic|BindingFlags.Instance).Invoke(target,arguments);
    private static void RunUnityIntegration()
    {
        Scene originalScene=SceneManager.GetActiveScene();
        Scene testScene=EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Additive);
        SceneManager.SetActiveScene(testScene);
        GameObject owner=null;
        try
        {
            owner=new GameObject("Elastic rod validation");owner.SetActive(false);
            var hose=owner.AddComponent<RobotCableHose>();hose.buildOnPlay=false;hose.ApplyElasticRodMaterial();
            hose.startAnchor=new GameObject("Start").transform;hose.endAnchor=new GameObject("End").transform;
            hose.startAnchor.SetParent(owner.transform);hose.endAnchor.SetParent(owner.transform);
            hose.startAnchor.position=new Vector3(1000,1,0);hose.endAnchor.position=new Vector3(1001.2f,1,0);
            hose.cableLength=1.6f;owner.SetActive(true);
            Assert(hose.Rebuild(),"Rod component rebuild");
            GameObject root=(GameObject)Private(hose,"root");
            Mesh mesh=root.GetComponent<MeshFilter>().sharedMesh;
            Assert(mesh.subMeshCount==2 && mesh.GetTriangles(1).Length>0,"Physical twist stripe submesh");
            Assert(root.GetComponentsInChildren<Rigidbody>().Length==0,"Rod must not create a joint chain");
            Vector3[] normals=mesh.normals;Vector3 before=normals[normals.Length-hose.radialSides-1];
            var states=(Array)Private(hose,"rodStates");
            var solver=(ElasticCableRod)states.GetValue(0).GetType().GetField("solver").GetValue(states.GetValue(0));
            Vector3 axis=solver.Frames[solver.EdgeCount-1]*Vector3.forward;
            Quaternion initial=hose.endAnchor.rotation;
            for(int i=1;i<=30;i++)
            {
                hose.endAnchor.rotation=RodMath.Exp(axis*((float)Math.PI*0.5f*i/30))*initial;
                Call(hose,"StepRod",0.02f);
            }
            Call(hose,"UpdateRodTube",false);
            Vector3 after=mesh.normals[mesh.normals.Length-hose.radialSides-1];
            Assert(Vector3.Dot(before,after)<0.8f,"Mesh stripe did not follow material rotation");
            var handle=new GameObject("Route handle").transform;handle.SetParent(owner.transform);
            handle.position=new Vector3(1000.6f,1.5f,0.2f);
            hose.rodRoutes=new[]{new RobotCableHose.RodRouteSpan{points=new[]{handle}}};
            hose.cableLength=hose.MeasureRodRouteLength()*1.05f;
            Assert(hose.Rebuild(),"Editable route rebuild");
            Assert(Math.Abs(hose.GeneratedRestLength-hose.cableLength)<0.0002f,"Component route rest length");
            hose.rodRoutes=Array.Empty<RobotCableHose.RodRouteSpan>();hose.endAnchor.rotation=initial;
            hose.cableLength=1.6f;hose.bendRigidity=0.001f;hose.useGravity=true;
            Assert(hose.Rebuild(),"Contact fixture rebuild");root=(GameObject)Private(hose,"root");
            // This is a query-only test in edit mode; the normal runtime path creates the same probe in Play.
            var probe=new GameObject("Query probe").AddComponent<CapsuleCollider>();
            probe.transform.SetParent(root.transform,false);probe.direction=2;probe.radius=hose.diameter/2;probe.enabled=false;
            typeof(RobotCableHose).GetField("rodProbe",BindingFlags.NonPublic|BindingFlags.Instance).SetValue(hose,probe);
            var floor=GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name="Validation floor";floor.transform.position=new Vector3(1000.6f,0.6f,0);floor.transform.localScale=new Vector3(4,0.2f,4);
            for(int i=0;i<100;i++) Call(hose,"StepRod",0.02f);
            states=(Array)Private(hose,"rodStates");solver=(ElasticCableRod)states.GetValue(0).GetType().GetField("solver").GetValue(states.GetValue(0));
            float minimum=float.PositiveInfinity;foreach(Vector3 point in solver.Positions) minimum=Math.Min(minimum,point.y);
            Vector3 lowest=solver.Positions[0];foreach(Vector3 point in solver.Positions) if(point.y<lowest.y) lowest=point;
            var contacts=Physics.OverlapSphere(lowest,0.021f,~0,QueryTriggerInteraction.Ignore);
            var nodeProbe=(SphereCollider)Private(hose,"rodNodeProbe");
            string detail="; nodeProbe="+(nodeProbe ? nodeProbe.radius.ToString() : "missing")+"; overlap="+contacts.Length+"; floor="+floor.GetComponent<Collider>().bounds;
            if(nodeProbe && Physics.ComputePenetration(nodeProbe,lowest,Quaternion.identity,floor.GetComponent<Collider>(),floor.transform.position,floor.transform.rotation,out Vector3 dn,out float dd)) detail+="; normal="+dn+"; depth="+dd;
            Assert(minimum>=0.715f && minimum<0.75f,"Unity collider penetration/settling: "+minimum+detail);
            hose.ClearGenerated();Assert(!hose.IsGenerated && hose.SegmentCount==0,"Rod component cleanup");
        }
        finally
        {
            if(owner) UnityEngine.Object.DestroyImmediate(owner);
            SceneManager.SetActiveScene(originalScene);
            EditorSceneManager.CloseScene(testScene,true);
        }
    }
    static void Assert(bool value, string message) { if(!value) throw new Exception(message); }
    static ElasticCableRod Arc(float rigidity)
    {
        Vector3[] points = ElasticCableRouting.Build(new[]{new Vector3(-0.6f,1,0),new Vector3(0.6f,1,0)},1.6f,20,Vector3.up);
        var rod = new ElasticCableRod(points,0.02f,0.3f) { BendRigidity=rigidity, TwistRigidity=0.5f, Damping=8 };
        rod.SetPositionPin(0,points[0]); rod.SetPositionPin(20,points[20]);
        rod.SetFramePin(0,rod.Frames[0]); rod.SetFramePin(19,rod.Frames[19]);
        return rod;
    }
    static void Advance(ElasticCableRod rod,int steps,Vector3 gravity,float dt=0.02f)
    { for(int i=0;i<steps;i++) rod.Step(dt,4,24,gravity); }
    public static string RunNumerical()
    {
        var log = new System.Text.StringBuilder();
        var route = new[]{Vector3.zero,new Vector3(0.3f,0.5f,0.2f),new Vector3(0.8f,0.4f,-0.1f),Vector3.right};
        float required=ElasticCableRouting.Length(ElasticCableRouting.Sample(route));
        var sampled=ElasticCableRouting.Build(route,required*1.1f,30,Vector3.up);
        Assert(Math.Abs(ElasticCableRouting.Length(sampled)-required*1.1f)<0.0001f,"Route did not preserve fixed length");
        bool rejected=false;
        try { ElasticCableRouting.Build(route,required*0.5f,30,Vector3.up); } catch(ArgumentException) { rejected=true; }
        Assert(rejected,"Short route accepted");
        var rest=Arc(2); Vector3 original=rest.Positions[10];
        Advance(rest,30,Vector3.zero);
        log.AppendLine("Rest drift="+Vector3.Distance(original,rest.Positions[10])+", stretch="+rest.MaximumRelativeStretch());
        Assert(Vector3.Distance(original,rest.Positions[10])<0.0005f,"Rest shape drift");
        var soft=Arc(0.001f); var hard=Arc(2);
        Advance(soft,150,new Vector3(0,-9.81f,0)); Advance(hard,150,new Vector3(0,-9.81f,0));
        float softSag=Vector3.Distance(original,soft.Positions[10]), hardSag=Vector3.Distance(original,hard.Positions[10]);
        log.AppendLine("Sag: soft="+softSag+", hard="+hardSag+", stretch="+hard.MaximumRelativeStretch());
        Assert(hardSag<softSag*0.5f,"Bend rigidity does not retain shape");
        Assert(hard.MaximumRelativeStretch()<0.03f,"Excessive stretch under gravity");
        var impulse=Arc(2); Vector3 initial=impulse.Positions[10];
        impulse.AddImpulse(10,new Vector3(0,0,0.02f)); Advance(impulse,5,Vector3.zero);
        float peak=Vector3.Distance(initial,impulse.Positions[10]); Advance(impulse,200,Vector3.zero);
        float recovered=Vector3.Distance(initial,impulse.Positions[10]);
        log.AppendLine("Impulse: deformed="+peak+", remaining="+recovered);
        Assert(peak>0.0001f && recovered<peak*0.5f,"No elastic deformation/recovery");
        var line=new Vector3[21]; for(int i=0;i<line.Length;i++) line[i]=new Vector3(0,0,i*0.05f);
        var twist=new ElasticCableRod(line,0.02f,0.3f) { Damping=8 };
        twist.SetPositionPin(0,line[0]); twist.SetPositionPin(20,line[20]); twist.SetFramePin(0,twist.Frames[0]);
        Quaternion end=twist.Frames[19];
        for(int i=0;i<=120;i++)
        {
            twist.SetFramePin(19,RodMath.Exp(Vector3.forward*((float)Math.PI*4*i/120))*end);
            twist.Step(0.02f,4,24,Vector3.zero);
        }
        Advance(twist,60,Vector3.zero);
        log.AppendLine("Twist turns="+twist.TotalTwistRadians()/(2*Math.PI)+", stretch="+twist.MaximumRelativeStretch());
        Assert(Math.Abs(twist.TotalTwistRadians()-4*Math.PI)<0.1,"Twist winding lost across full turns");
        float minTwist=float.MaxValue,maxTwist=float.MinValue;
        for(int i=0;i<19;i++)
        {
            float angle=RodMath.Log(RodMath.Inverse(twist.Frames[i])*twist.Frames[i+1]).z;
            minTwist=Math.Min(minTwist,angle); maxTwist=Math.Max(maxTwist,angle);
        }
        log.AppendLine("Distributed twist range="+minTwist+" .. "+maxTwist);
        Assert(minTwist>0.3f && maxTwist<1f,"Twist is localized or unstable rather than distributed");
        var moving=Arc(2); Quaternion movingStart=moving.Frames[0],movingEnd=moving.Frames[19];
        for(int i=0;i<100;i++)
        {
            float angle=(float)Math.PI*0.2f*i/100;
            Quaternion rotation=RodMath.Exp(Vector3.up*angle);
            moving.SetPositionPin(0,rotation*new Vector3(-0.6f,1,0));
            moving.SetPositionPin(20,rotation*new Vector3(0.6f,1,0));
            moving.SetFramePin(0,rotation*movingStart); moving.SetFramePin(19,rotation*movingEnd);
            moving.Step(0.02f,4,24,Vector3.zero);
        }
        Assert(moving.MaximumRelativeStretch()<0.02f,"Moving attachments stretch rod excessively");
        var floor=Arc(0.001f);
        floor.ProjectContacts=r=>{for(int i=0;i<r.Positions.Length;i++) if(r.Positions[i].y<0.7f) r.ResolveNodeContact(i,Vector3.up,0.7f-r.Positions[i].y);};
        Advance(floor,120,new Vector3(0,-9.81f,0));
        foreach(var p in floor.Positions) Assert(p.y>=0.6999f,"Contact penetration");
        Assert(Vector3.Distance(floor.Positions[0],new Vector3(-0.6f,1,0))<1e-6f,"Contact displaced pin");
        log.AppendLine("ELASTIC_ROD_CHECKS_PASSED");
        return log.ToString();
    }
}



}

