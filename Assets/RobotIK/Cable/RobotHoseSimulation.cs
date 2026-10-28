using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Profiling;

namespace RobotIK
{
    [Serializable]
    public sealed class RobotHosePhysicsSettings
    {
        public bool enabled = true;
        [Tooltip("World meters. Zero captures the authored curve length on entering Play mode.")]
        [Min(0)] public float cableLength;
        [Tooltip("Initial cable outside the feeder. Zero uses the total length.")]
        [Min(0)] public float initialDeployedLength;
        [Tooltip("Manual requested deployed length in world meters. Ignored while Reel > Automatic is enabled. Zero holds current length.")]
        [Min(0)] public float targetDeployedLength;
        [Min(0)] public float feedSpeed = 0.1f;
        [Tooltip("Stop retraction when relative stretch exceeds this value. This is a strain limit, not a measured motor force.")]
        [Range(0.005f, 0.2f)] public float stallStrain = 0.03f;
        [Min(0.001f)] public float massPerMeter = 0.3f;
        [Min(0.0001f)] public float bendRigidity = 2f;
        [Min(0.0001f)] public float twistRigidity = 0.05f;
        [Min(10)] public float axialRigidity = 20000;
        [Range(0, 30)] public float damping = 5;
        [Tooltip("Physics spacing in world meters, independent of the visible tube's Segments Per Span. Rebuild after changing.")]
        [Min(0.01f)] public float targetSegmentLength = 0.06f;
        [Range(1, 12)] public int substeps = 2;
        [Range(4, 64)] public int iterations = 10;
        public bool useGravity = true;
        public bool selfCollision;
        public LayerMask collisionLayers = ~0;
        public Collider[] ignoredColliders = Array.Empty<Collider>();
        public RobotHoseReelSettings reel = new RobotHoseReelSettings();
    }

    /// <summary>
    /// Scene adapter: owns rod spans, clamp/guide attachments, material transfer and
    /// Unity contact queries. Reel decisions live in a separate controller.
    /// </summary>
    public sealed class RobotHoseSimulation : IDisposable
    {
        private sealed class Span
        {
            public ElasticCableRod rod;
            public Quaternion startOffset, endOffset;
            public Vector3[] renderPositions;
            public Quaternion[] renderFrames;
            public bool startHeld, endHeld;
            public Vector3[] routeContacts;
            public float[] contactExpiry;
        }
        private readonly Transform[] anchors;
        private readonly Span[] spans;
        private readonly RobotHosePhysicsSettings settings;
        private readonly RobotBaseSixAxisMotionPlanner planner;
        private readonly RobotHoseContacts contacts;
        private readonly float[] segmentLengths, guideFlows;
        private readonly bool[] slidingGuides;
        private readonly RobotHoseSpanDemand[] demands;
        private readonly RobotHoseReelController reel;
        private bool paused;
        private float simulationTime;
        private readonly float renderClearance;
        private static readonly ProfilerMarker StepMarker=new ProfilerMarker("RobotHose.Simulation");
        private static readonly ProfilerMarker RodMarker=new ProfilerMarker("RobotHose.RodSolve");
        public float TotalLength { get; }
        public float DeployedLength { get; private set; }
        public float StoredLength => Mathf.Max(0, TotalLength-DeployedLength);
        public bool RetractionBlocked { get; private set; }
        public float MaximumStrain { get; private set; }
        public int SegmentCount { get { int count=0; foreach(var span in spans) count+=span.rod.EdgeCount; return count; } }
        public int BroadphaseQueries => contacts.BroadphaseQueries;
        public int SweepQueries => contacts.SweepQueries;
        public int PenetrationQueries => contacts.PenetrationQueries;
        public int SelfCollisionPairs { get; private set; }
        public RobotHoseReelState ReelState => reel.State;
        public float RequiredDeployedLength => reel.RequiredLength;
        public float AutomaticTargetLength => reel.TargetLength;
        public int ReelConnectedSpanCount => reel.ReelSpanCount;

        public RobotHoseSimulation(Transform[] anchors, Vector3[][] routes, float radius,
            RobotHosePhysicsSettings settings, int layer, Scene scene, RobotHoseMidpointMode[] midpointModes=null)
        {
            if (anchors == null || routes == null || routes.Length == 0 || anchors.Length != routes.Length+1)
                throw new ArgumentException("Provide one cable route between each ordered attachment pair.");
            foreach (var anchor in anchors) if (!anchor) throw new ArgumentException("A cable attachment is missing.");
            foreach (float value in new[] {radius, settings.cableLength, settings.initialDeployedLength,
                settings.targetDeployedLength,settings.feedSpeed,settings.stallStrain,settings.massPerMeter,
                settings.bendRigidity,settings.twistRigidity,settings.axialRigidity,settings.damping,settings.targetSegmentLength})
                if (float.IsNaN(value) || float.IsInfinity(value) || value<0)
                    throw new ArgumentException("Cable dimensions and physics settings must be finite and nonnegative.");
            if (radius<=0 || settings.massPerMeter<=0)
                throw new ArgumentException("Cable radius and mass per meter must be positive.");
            this.anchors = anchors;
            this.settings = settings;
            renderClearance=radius*0.15f;
            planner = anchors[0].GetComponentInParent<RobotBaseSixAxisMotionPlanner>();
            var lengths = new float[routes.Length];
            float authoredLength = 0;
            for (int i=0; i<routes.Length; i++)
            {
                if (routes[i]==null || routes[i].Length<4 || routes[i].Length>513)
                    throw new ArgumentException("Each initial cable span needs between 3 and 512 segments.");
                foreach(var point in routes[i])
                    if(float.IsNaN(point.sqrMagnitude) || float.IsInfinity(point.sqrMagnitude))
                        throw new ArgumentException("Cable attachment positions must be finite.");
                lengths[i] = ElasticCableRouting.Length(routes[i]);
                if (lengths[i] < radius*2) throw new ArgumentException("A hose span is shorter than its diameter.");
                for (int j=1; j<routes[i].Length; j++)
                    if (Vector3.Distance(routes[i][j-1],routes[i][j]) < 1e-5f)
                        throw new ArgumentException("The initial hose curve has coincident samples; adjust its handles.");
                authoredLength += lengths[i];
            }
            TotalLength = settings.cableLength > 0 ? settings.cableLength : authoredLength;
            DeployedLength = settings.initialDeployedLength > 0 ? settings.initialDeployedLength : TotalLength;
            if (DeployedLength > TotalLength+1e-5f || DeployedLength < authoredLength-1e-4f)
                throw new ArgumentException("Initial deployed length must fit the authored curve and not exceed total cable length.");
            spans = new Span[routes.Length];
            slidingGuides=new bool[routes.Length-1];
            guideFlows=new float[slidingGuides.Length];
            segmentLengths=new float[routes.Length];
            demands=new RobotHoseSpanDemand[routes.Length];
            var initialLengths=new float[routes.Length];
            for(int i=0;i<slidingGuides.Length;i++)
                slidingGuides[i]=midpointModes!=null && i<midpointModes.Length && midpointModes[i]==RobotHoseMidpointMode.SlidingGuide;
            if(settings.reel==null) settings.reel=new RobotHoseReelSettings();
            foreach(float value in new[]{settings.reel.desiredSlack,settings.reel.slackTolerance,settings.reel.extensionSpeed,
                settings.reel.retractionSpeed,settings.reel.acceleration,settings.reel.guideSlideSpeed,settings.reel.extensionStrain,settings.reel.retractionDelay})
                if(float.IsNaN(value) || float.IsInfinity(value) || value<0)
                    throw new ArgumentException("Reel settings must be finite and nonnegative.");
            for (int i=0; i<spans.Length; i++)
            {
                // Extra available length belongs to the reel-connected span only.
                Vector3[] points = (Vector3[])routes[i].Clone();
                if (i==0 && DeployedLength > authoredLength+1e-5f)
                    AddSlack(points, lengths[0]+DeployedLength-authoredLength);
                float restLength=ElasticCableRouting.Length(points);
                // Older serialized settings do not contain this new field; zero
                // must resolve to the normal default rather than a very fine grid.
                float spacing=settings.targetSegmentLength>=0.01f?settings.targetSegmentLength:0.06f;
                points=ResampleRoute(points,spacing,radius*0.15f);
                var rod = new ElasticCableRod(points, radius+renderClearance, settings.massPerMeter) { CoupledBending=true };
                rod.SetRestLength(restLength,settings.massPerMeter);
                spans[i] = new Span { rod=rod,
                    startOffset=RodMath.Inverse(anchors[i].rotation)*rod.Frames[0],
                    endOffset=RodMath.Inverse(anchors[i+1].rotation)*rod.Frames[rod.EdgeCount-1],
                    startHeld=i==0 || !slidingGuides[i-1], endHeld=i==spans.Length-1 || !slidingGuides[i] };
                segmentLengths[i]=rod.RestLength/rod.EdgeCount;
                initialLengths[i]=rod.RestLength;
            }
            UpdateDeployedLength();
            reel=new RobotHoseReelController(slidingGuides,initialLengths,settings.reel);
            // Allocate scene resources only after all route validation succeeds.
            contacts = new RobotHoseContacts(radius+renderClearance, layer, scene, settings);
            foreach (var span in spans)
            {
                span.rod.ProjectContacts = contacts.Project;
                span.rod.PrepareContacts = contacts.BeginSubstep;
            }
            ApplyPins();
        }

        private static void AddSlack(Vector3[] points, float length)
        {
            var original = (Vector3[])points.Clone();
            Vector3 bow = RodMath.Normal((points[points.Length-1]-points[0]).normalized, Vector3.down);
            float low=0, high=length;
            for (int k=0; k<48; k++)
            {
                float amplitude=(low+high)*0.5f;
                for (int i=1; i<points.Length-1; i++)
                    points[i]=original[i]+bow*(amplitude*Mathf.Sin(Mathf.PI*i/(points.Length-1)));
                if (ElasticCableRouting.Length(points)<length) low=amplitude; else high=amplitude;
            }
        }

        private void ApplyPins()
        {
            for (int i=0; i<spans.Length; i++)
            {
                var span=spans[i]; var rod=span.rod;
                rod.SetPositionPin(0, anchors[i].position);
                rod.SetPositionPin(rod.EdgeCount, anchors[i+1].position);
                if(span.startHeld) rod.SetFramePin(0, anchors[i].rotation*span.startOffset);
                if(span.endHeld) rod.SetFramePin(rod.EdgeCount-1, anchors[i+1].rotation*span.endOffset);
            }
            for(int i=0;i<slidingGuides.Length;i++)
            {
                if(!slidingGuides[i]) continue;
                var left=spans[i].rod; var right=spans[i+1].rod;
                Quaternion a=left.Frames[left.EdgeCount-1], b=right.Frames[0];
                Quaternion common=RodMath.Slerp(a,RodMath.FromTo(b*Vector3.forward,a*Vector3.forward)*b,0.5f);
                left.SetTwistPin(left.EdgeCount-1,common);
                right.SetTwistPin(0,RodMath.FromTo(common*Vector3.forward,b*Vector3.forward)*common);
            }
        }

        public bool AttachmentsValid
        {
            get { foreach (var anchor in anchors) if (!anchor) return false; return true; }
        }

        public void Step(float dt)
        {
            using(StepMarker.Auto()) StepSimulation(dt);
        }

        private void StepSimulation(float dt)
        {
            if (!AttachmentsValid) return;
            bool planning=planner && (planner.CurrentMotionState==RobotBaseSixAxisMotionPlanner.MotionState.Planning || planner.IsRrtSearchInProgress);
            if (planning)
            {
                if (!paused) foreach(var span in spans) span.rod.ResetVelocities();
                paused=true; return;
            }
            paused=false;
            simulationTime+=dt;
            Physics.SyncTransforms();
            Feed(dt);
            ApplyPins();
            MaximumStrain=0; SelfCollisionPairs=0; contacts.ResetCounters();
            foreach (var span in spans)
            {
                var rod=span.rod;
                rod.BendRigidity=Mathf.Max(0.0001f,settings.bendRigidity);
                rod.TwistRigidity=Mathf.Max(0.0001f,settings.twistRigidity);
                rod.AxialRigidity=Mathf.Max(10,settings.axialRigidity);
                rod.Damping=Mathf.Max(0,settings.damping); rod.SelfCollision=settings.selfCollision;
                using(RodMarker.Auto()) rod.Step(dt,settings.substeps,settings.iterations,settings.useGravity?Physics.gravity:Vector3.zero);
                MaximumStrain=Mathf.Max(MaximumStrain,rod.MaximumRelativeStretch());
                SelfCollisionPairs+=rod.SelfCollisionPairsTested;
            }
        }

        private void Feed(float dt)
        {
            RetractionBlocked=false;
            ObserveSpans();
            float delta=reel.PlanReel(TotalLength,DeployedLength,settings.targetDeployedLength,settings.feedSpeed,demands,dt);
            delta=Mathf.Clamp(delta,-segmentLengths[0]*0.2f,segmentLengths[0]*0.2f);
            delta=Mathf.Min(delta,Mathf.Max(0,TotalLength-DeployedLength));
            if(delta>0)
            {
                if(CanGrow(spans[0].rod,true,delta,segmentLengths[0]))
                    spans[0].rod=spans[0].rod.FeedAtStart(delta,segmentLengths[0],settings.massPerMeter);
                else reel.BlockRetraction();
                ObserveSpans();
            }
            reel.PlanGuides(demands,delta<0?delta:0,guideFlows);
            for(int i=0;i<guideFlows.Length;i++) if(guideFlows[i]>0) TransferGuide(i,guideFlows[i],dt);
            for(int i=guideFlows.Length-1;i>=0;i--) if(guideFlows[i]<0) TransferGuide(i,guideFlows[i],dt);
            if(delta<0)
            {
                if(CanConsume(0,true,-delta))
                    spans[0].rod=spans[0].rod.FeedAtStart(delta,segmentLengths[0],settings.massPerMeter);
                else { RetractionBlocked=true; reel.BlockRetraction(); }
            }
            UpdateDeployedLength();
            reel.FinishStep();
        }

        private void TransferGuide(int guide,float flow,float dt)
        {
            float amount=Mathf.Min(Mathf.Abs(flow),Mathf.Max(0,settings.reel.guideSlideSpeed)*dt);
            amount=Mathf.Min(amount,Mathf.Min(segmentLengths[guide],segmentLengths[guide+1])*0.2f);
            if(amount<1e-7f) return;
            bool forward=flow>0;
            int donor=forward?guide:guide+1, receiver=forward?guide+1:guide;
            bool donorStart=!forward, receiverStart=forward;
            if(!CanConsume(donor,donorStart,amount) || !CanGrow(spans[receiver].rod,receiverStart,amount,segmentLengths[receiver])) return;
            var sample=spans[donor].rod.SampleBoundaryMaterial(donorStart);
            var source=spans[donor].rod; var destination=spans[receiver].rod;
            float total=source.RestLength+destination.RestLength;
            spans[donor].rod=donorStart?source.FeedAtStart(-amount,segmentLengths[donor],settings.massPerMeter):
                source.FeedAtEnd(-amount,segmentLengths[donor],settings.massPerMeter);
            float transferred=source==spans[donor].rod?amount:source.RestLength-spans[donor].rod.RestLength;
            spans[receiver].rod=receiverStart?destination.FeedAtStart(transferred,segmentLengths[receiver],settings.massPerMeter):
                destination.FeedAtEnd(transferred,segmentLengths[receiver],settings.massPerMeter);
            // Correct only roundoff; the pair budget cannot grow through a guide.
            float expected=total-spans[donor].rod.RestLength;
            if(Mathf.Abs(spans[receiver].rod.RestLength-expected)>1e-7f)
                spans[receiver].rod.SetRestLength(expected,settings.massPerMeter);
            spans[receiver].rod.ReceiveBoundaryMaterial(receiverStart,transferred,sample);
        }

        private static bool CanGrow(ElasticCableRod rod,bool start,float amount,float segmentLength) =>
            rod.EdgeCount<512 || rod.RestLengths[start?0:rod.EdgeCount-1]+amount<=segmentLength*1.5f;

        private bool CanConsume(int index,bool start,float amount)
        {
            var rod=spans[index].rod;
            RobotHoseSpanDemand demand=MeasureSpan(index);
            return demand.tensileStrain<=settings.stallStrain && rod.RestLength-amount>=demand.requiredLength &&
                rod.RestLengths[start?0:rod.EdgeCount-1]-amount>=segmentLengths[index]*0.1f;
        }

        private void ObserveSpans()
        { for(int i=0;i<spans.Length;i++) demands[i]=MeasureSpan(i); }

        private RobotHoseSpanDemand MeasureSpan(int index)
        {
            var span=spans[index]; var rod=span.rod;
            if(span.routeContacts==null || span.routeContacts.Length!=rod.Positions.Length)
            {
                span.routeContacts=new Vector3[rod.Positions.Length];
                span.contactExpiry=new float[rod.Positions.Length];
            }
            Vector3 previous=anchors[index].position;
            float required=0;
            if(span.startHeld)
            {
                Vector3 lead=previous+(anchors[index].rotation*span.startOffset)*Vector3.forward*rod.RestLengths[0];
                required+=Vector3.Distance(previous,lead); previous=lead;
            }
            for(int node=1;node<rod.EdgeCount;node++)
            {
                if(rod.HasContact(node))
                {
                    span.routeContacts[node]=rod.Positions[node];
                    span.contactExpiry[node]=simulationTime+0.15f;
                }
                if(span.contactExpiry[node]<=simulationTime) continue;
                Vector3 point=span.routeContacts[node]; required+=Vector3.Distance(previous,point); previous=point;
            }
            Vector3 end=anchors[index+1].position;
            if(span.endHeld)
            {
                Vector3 lead=end-(anchors[index+1].rotation*span.endOffset)*Vector3.forward*rod.RestLengths[rod.EdgeCount-1];
                required+=Vector3.Distance(previous,lead); previous=lead;
            }
            required+=Vector3.Distance(previous,end);
            float tensile=0, extra=0;
            for(int edge=0;edge<rod.EdgeCount;edge++)
            {
                Vector3 a=edge==0?anchors[index].position:rod.Positions[edge];
                Vector3 b=edge==rod.EdgeCount-1?end:rod.Positions[edge+1];
                float stretch=Vector3.Distance(a,b)-rod.RestLengths[edge];
                tensile=Mathf.Max(tensile,stretch/rod.RestLengths[edge]); extra+=Mathf.Max(0,stretch);
            }
            return new RobotHoseSpanDemand { length=rod.RestLength,requiredLength=Mathf.Max(required,segmentLengths[index]*2.2f),
                tensileStrain=tensile,stretchedLength=extra };
        }

        private void UpdateDeployedLength()
        { DeployedLength=0; foreach(var span in spans) DeployedLength+=span.rod.RestLength; }

        public void GetRenderSamples(List<Vector3> positions, List<Quaternion> frames, List<float> distances, float alpha, int renderSegmentsPerSpan=0)
        {
            positions.Clear(); frames.Clear(); distances.Clear();
            float distance=StoredLength;
            foreach(var span in spans)
            {
                var rod=span.rod;
                int count=renderSegmentsPerSpan>0?Mathf.Max(renderSegmentsPerSpan,rod.EdgeCount):rod.EdgeCount;
                if(span.renderPositions==null || span.renderPositions.Length!=rod.Positions.Length)
                {
                    span.renderPositions=new Vector3[rod.Positions.Length];
                    span.renderFrames=new Quaternion[rod.EdgeCount];
                }
                for(int i=0;i<rod.Positions.Length;i++)
                    span.renderPositions[i]=Vector3.LerpUnclamped(rod.PreviousPositions[i],rod.Positions[i],alpha);
                for(int i=0;i<rod.EdgeCount;i++)
                    span.renderFrames[i]=RodMath.Slerp(rod.PreviousFrames[i],rod.Frames[i],alpha);
                int edge=0; float edgeStart=0;
                for(int i=0;i<=count;i++)
                {
                    float at=rod.RestLength*i/count;
                    if(renderSegmentsPerSpan<=0)
                    {
                        if(i>0 || positions.Count==0)
                        {
                            positions.Add(span.renderPositions[i]);
                            frames.Add(span.renderFrames[Mathf.Min(i,rod.EdgeCount-1)]);
                            distances.Add(distance+edgeStart);
                        }
                        if(i<rod.EdgeCount) edgeStart+=rod.RestLengths[i];
                        continue;
                    }
                    while(edge<rod.EdgeCount-1 && edgeStart+rod.RestLengths[edge]<at)
                        edgeStart+=rod.RestLengths[edge++];
                    float t=Mathf.Clamp01((at-edgeStart)/rod.RestLengths[edge]);
                    Vector3 a=span.renderPositions[edge], b=span.renderPositions[edge+1];
                    Vector3 straight=Vector3.LerpUnclamped(a,b,t);
                    Vector3 before=span.renderPositions[Mathf.Max(0,edge-1)], after=span.renderPositions[Mathf.Min(rod.EdgeCount,edge+2)];
                    float length=Vector3.Distance(a,b);
                    Vector3 m0=(b-before).normalized*length, m1=(after-a).normalized*length;
                    float t2=t*t,t3=t2*t;
                    Vector3 smooth=(2*t3-3*t2+1)*a+(t3-2*t2+t)*m0+(-2*t3+3*t2)*b+(t3-t2)*m1;
                    // A bounded curve remains inside the collision capsule envelope.
                    Vector3 center=straight+Vector3.ClampMagnitude(smooth-straight,renderClearance);
                    if(i>0 || positions.Count==0)
                    {
                        positions.Add(center);
                        frames.Add(RodMath.Slerp(span.renderFrames[edge],span.renderFrames[Mathf.Min(edge+1,rod.EdgeCount-1)],t));
                        distances.Add(distance+at);
                    }
                }
                distance+=rod.RestLength;
            }
        }

        public void Dispose() => contacts?.Dispose();

        private static Vector3[] ResampleRoute(Vector3[] source,float spacing,float tolerance)
        {
            var distances=new float[source.Length];
            for(int i=1;i<source.Length;i++) distances[i]=distances[i-1]+Vector3.Distance(source[i-1],source[i]);
            float length=distances[distances.Length-1];
            int count=Mathf.Clamp(Mathf.CeilToInt(length/spacing),4,256);
            while(true)
            {
                var result=new Vector3[count+1];
                int index=0;
                for(int i=0;i<=count;i++)
                {
                    float at=length*i/count;
                    while(index<source.Length-2 && distances[index+1]<at) index++;
                    result[i]=Vector3.LerpUnclamped(source[index],source[index+1],(at-distances[index])/(distances[index+1]-distances[index]));
                }
                result[0]=source[0]; result[count]=source[source.Length-1];
                float error=0;
                for(int i=1;i<source.Length-1;i++)
                {
                    int edge=Mathf.Min(count-1,(int)(distances[i]/length*count));
                    Vector3 delta=result[edge+1]-result[edge];
                    float t=delta.sqrMagnitude>1e-12f?Mathf.Clamp01(Vector3.Dot(source[i]-result[edge],delta)/delta.sqrMagnitude):0;
                    error=Mathf.Max(error,Vector3.Distance(source[i],result[edge]+delta*t));
                }
                if(error<=tolerance || count>=256) return result;
                count=Mathf.Min(256,count+Mathf.Max(2,count/4));
            }
        }
    }
}
