using System;
using UnityEngine;

namespace RobotIK
{
    public enum RobotHoseMidpointMode { FixedClamp, SlidingGuide }
    public enum RobotHoseReelState { Holding, Extending, Retracting, RetractionBlocked, TotalLengthReached, FixedSpanOverextended }

    [Serializable]
    public sealed class RobotHoseReelSettings
    {
        [Tooltip("Enable automatic slack/tension control. Off preserves manual Target Deployed Length control.")]
        public bool automatic;
        [Min(0)] public float desiredSlack = 0.05f;
        [Min(0.001f)] public float slackTolerance = 0.01f;
        [Min(0)] public float extensionSpeed = 0.15f;
        [Min(0)] public float retractionSpeed = 0.05f;
        [Min(0.01f)] public float acceleration = 0.5f;
        [Min(0)] public float guideSlideSpeed = 0.2f;
        [Range(0.001f,0.1f)] public float extensionStrain = 0.015f;
        [Min(0)] public float retractionDelay = 0.25f;
    }

    /// <summary>Reusable span observations, supplied by the scene adapter without extra physics queries.</summary>
    public struct RobotHoseSpanDemand
    {
        public float length, requiredLength, tensileStrain, stretchedLength;
    }

    /// <summary>
    /// Reel and frictionless-guide policy only. No Transform, collider, rendering or
    /// rod mutation dependencies. The adapter executes rate-limited material flows.
    /// </summary>
    public sealed class RobotHoseReelController
    {
        private readonly bool[] guides;
        private readonly float[] slackWeights;
        private readonly RobotHoseReelSettings settings;
        private float velocity, quietTime;
        private bool lastAutomatic;
        public int ReelSpanCount { get; }
        public float TargetLength { get; private set; }
        public float RequiredLength { get; private set; }
        public RobotHoseReelState State { get; private set; }
        public bool FixedGroupOverextended { get; private set; }

        public RobotHoseReelController(bool[] guides,float[] initialLengths,RobotHoseReelSettings settings)
        {
            this.guides=(bool[])guides.Clone();
            slackWeights=(float[])initialLengths.Clone();
            this.settings=settings;
            int count=1;
            while(count<initialLengths.Length && guides[count-1]) count++;
            ReelSpanCount=count;
        }

        public float PlanReel(float total,float deployed,float manualTarget,float manualSpeed,
            RobotHoseSpanDemand[] spans,float dt)
        {
            if(lastAutomatic!=settings.automatic)
            { velocity=0; quietTime=0; lastAutomatic=settings.automatic; }
            float required=0, fixedLength=0, strain=0, stretch=0;
            for(int i=0;i<spans.Length;i++)
            {
                if(i<ReelSpanCount)
                {
                    required+=spans[i].requiredLength;
                    strain=Mathf.Max(strain,spans[i].tensileStrain);
                    stretch+=spans[i].stretchedLength;
                }
                else fixedLength+=spans[i].length;
            }
            RequiredLength=required+fixedLength;
            float target=manualTarget>0?manualTarget:deployed;
            if(settings.automatic)
            {
                target=RequiredLength+Mathf.Max(0,settings.desiredSlack);
                if(strain>Mathf.Max(0.001f,settings.extensionStrain))
                {
                    // Geometry can underestimate a new snag. Signed tensile strain
                    // requests payout; compression never triggers extension.
                    target=Mathf.Max(target,deployed+stretch+settings.slackTolerance);
                    quietTime=0;
                }
                else if(strain<settings.extensionStrain*0.5f) quietTime+=dt;
                else quietTime=0;
            }
            TargetLength=Mathf.Clamp(target,0,total);
            float error=TargetLength-deployed;
            float tolerance=settings.automatic?Mathf.Max(0.001f,settings.slackTolerance):1e-7f;
            bool limited=RequiredLength>total+tolerance ||
                (target>total+tolerance && deployed>=total-tolerance);
            if(Mathf.Abs(error)<=tolerance ||
                (settings.automatic && error<0 && quietTime<settings.retractionDelay))
            {
                velocity=0;
                State=limited?RobotHoseReelState.TotalLengthReached:RobotHoseReelState.Holding;
                return 0;
            }
            float speed=settings.automatic?(error>0?settings.extensionSpeed:settings.retractionSpeed):manualSpeed;
            float desiredVelocity=Mathf.Sign(error)*Mathf.Max(0,speed);
            velocity=settings.automatic?Mathf.MoveTowards(velocity,desiredVelocity,Mathf.Max(0.01f,settings.acceleration)*dt):desiredVelocity;
            float delta=velocity*dt;
            if(delta*error<=0) delta=0;
            delta=Mathf.Clamp(delta,-Mathf.Abs(error),Mathf.Abs(error));
            State=limited?RobotHoseReelState.TotalLengthReached:
                delta>0?RobotHoseReelState.Extending:delta<0?RobotHoseReelState.Retracting:RobotHoseReelState.Holding;
            return delta;
        }

        /// <summary>
        /// Boundary flow is positive toward the end. A fixed clamp is always zero.
        /// Group budgets are conserved; planned reel withdrawal is funded by guide
        /// flow before the adapter consumes material at the inlet.
        /// </summary>
        public void PlanGuides(RobotHoseSpanDemand[] spans,float pendingReelDelta,float[] flows)
        {
            Array.Clear(flows,0,flows.Length);
            FixedGroupOverextended=false;
            for(int first=0;first<spans.Length;)
            {
                int last=first;
                while(last<guides.Length && guides[last]) last++;
                float budget=first==0?pendingReelDelta:0, required=0, weight=0;
                for(int i=first;i<=last;i++)
                { budget+=spans[i].length; required+=spans[i].requiredLength; weight+=slackWeights[i]; }
                if(first>0 && required>budget+settings.slackTolerance) FixedGroupOverextended=true;
                float flow=first==0?pendingReelDelta:0;
                for(int i=first;i<last;i++)
                {
                    float desired=budget>=required?spans[i].requiredLength+(budget-required)*slackWeights[i]/weight:
                        budget*spans[i].requiredLength/Mathf.Max(1e-6f,required);
                    flow-=desired-spans[i].length;
                    flows[i]=flow;
                }
                first=last+1;
            }
        }

        public void BlockRetraction()
        {
            if(State!=RobotHoseReelState.TotalLengthReached) State=RobotHoseReelState.RetractionBlocked;
            velocity=0; quietTime=0;
        }

        public void FinishStep()
        {
            if(FixedGroupOverextended) State=RobotHoseReelState.FixedSpanOverextended;
        }
    }
}
