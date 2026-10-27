using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace RobotIK
{
    public enum PlanDiagnosticCode { None, IkNoSolution, RobotReach, JointLimit, TrackLimit, Collision, Clearance, Singularity, JointContinuity, WristBranch, StretchLimit, PoseClosure, InvalidSetup, TimeLimit, CheckLimit, AttemptLimit, IterationLimit, NoCompleteRoute, Cancelled, ConfigurationChanged, MotionStopped, EndpointValidation }
    public enum PlanReportOutcome { Running, Complete, Failed, SearchIncomplete, Cancelled, Stopped }
    [Serializable] public sealed class PlanDiagnosticEvent
    {
        internal string RetentionKey;
        public int Id, Attempt, Sample, Joint = -1;
        public PlanDiagnosticCode Code;
        public string Phase, Message, RobotObject, ObstacleObject, EvidenceKind;
        public int RobotId, ObstacleId;
        public bool HasTcp, HasE1, HasJoints, HasMeasurement, HasLimit, HasRange, HasBase;
        public Vector3 TcpPosition, BasePosition, BaseScale;
        public Quaternion TcpRotation, BaseRotation;
        public float E1, Measured, Required, Minimum, Maximum;
        public float[] Joints;
        public string Units;
        public PlanDiagnosticEvent Copy()
        { var result = (PlanDiagnosticEvent)MemberwiseClone(); result.Joints = Joints == null ? null : (float[])Joints.Clone(); return result; }
        public string Title => Code.ToString() + (string.IsNullOrEmpty(RobotObject) ? string.Empty : " · " + RobotObject.Substring(RobotObject.LastIndexOf('/') + 1));
        public string Details()
        {
            var b = new StringBuilder();
            b.AppendLine(Code + " · " + (Phase ?? "Unknown phase") + $" · attempt {Attempt}, query {Sample}");
            b.AppendLine(Message ?? string.Empty);
            b.AppendLine(HasE1 ? $"E1: {E1:F4} mm" : "E1: not applicable / unavailable");
            if (HasTcp) b.AppendLine($"Rejected TCP (world mm): {TcpPosition.x * 1000:F4}, {TcpPosition.y * 1000:F4}, {TcpPosition.z * 1000:F4}");
            else b.AppendLine("Rejected TCP: not captured; no spatial marker available.");
            if (HasTcp) b.AppendLine($"TCP world quaternion (xyzw): {TcpRotation.x:G9}, {TcpRotation.y:G9}, {TcpRotation.z:G9}, {TcpRotation.w:G9}");
            if (HasJoints) b.AppendLine("q1…q6 (degrees): " + string.Join(", ", Array.ConvertAll(Joints, x => x.ToString("G9", CultureInfo.InvariantCulture))));
            if (HasMeasurement) b.AppendLine($"Measured: {Measured:G9} {Units}");
            if (HasLimit) b.AppendLine($"Required: {Required:G9} {Units}");
            if (HasRange) b.AppendLine($"Allowed: [{Minimum:G9}, {Maximum:G9}] {Units}");
            if (Joint >= 0) b.AppendLine("Joint: J" + (Joint + 1));
            if (!string.IsNullOrEmpty(RobotObject)) b.AppendLine("Robot: " + RobotObject);
            if (!string.IsNullOrEmpty(ObstacleObject)) b.AppendLine("Obstacle: " + ObstacleObject);
            if (!string.IsNullOrEmpty(EvidenceKind)) b.AppendLine("Evidence: " + EvidenceKind);
            b.AppendLine("Scope: rejected candidate/configuration; alternatives are not proven impossible.");
            return b.ToString();
        }
    }
    [Serializable] public sealed class PlanDiagnosticCounter { public PlanDiagnosticCode Code; public int Observations; }
    [Serializable] public sealed class PlanDiagnosticReport
    {
        public string RunId, StartedUtc, Planner, SettingsSnapshot, Summary, StartSummary, EndSummary;
        public PlanReportOutcome Outcome;
        public PlanDiagnosticCode StopCode;
        public bool Stale;
        public int Revision, ConfigurationQueries, CollisionEvaluations, CacheHits, Connections, DroppedEvents;
        public List<PlanDiagnosticEvent> Events = new List<PlanDiagnosticEvent>();
        public List<PlanDiagnosticCounter> Counters = new List<PlanDiagnosticCounter>();
        private int sequence;
        private Dictionary<string, int> retentionCounts;
        public void Add(PlanDiagnosticEvent item)
        {
            var counter = Counters.Find(x => x.Code == item.Code);
            if (counter == null) { counter = new PlanDiagnosticCounter { Code = item.Code }; Counters.Add(counter); }
            counter.Observations++; item = item.Copy(); item.Id = ++sequence;
            string Key(PlanDiagnosticEvent e) => e.RetentionKey ?? (e.RetentionKey = e.Code + "|" + e.Phase + "|" + e.RobotObject + "|" + e.ObstacleObject + "|" + e.Joint);
            if (retentionCounts == null)
            {
                retentionCounts = new Dictionary<string, int>();
                foreach (var e in Events) { string key = Key(e); retentionCounts.TryGetValue(key, out int n); retentionCounts[key] = n + 1; }
            }
            if (Events.Count >= 128)
            {
                // Keep rare causes/object pairs when repeated invalid analytical
                // branches would otherwise push every collision out of the report.
                int victim = 0, largest = 1;
                for (int i = 0; i < Events.Count; i++) if (retentionCounts[Key(Events[i])] > largest) { victim = i; largest = retentionCounts[Key(Events[i])]; }
                retentionCounts[Key(Events[victim])]--;
                Events.RemoveAt(victim); DroppedEvents++;
            }
            string itemKey = Key(item); retentionCounts.TryGetValue(itemKey, out int total); retentionCounts[itemKey] = total + 1;
            Events.Add(item); Revision++;
        }
        public static string StatusTitle(PlanDiagnosticCode code) => code switch
        {
            PlanDiagnosticCode.None => "Plan completed",
            PlanDiagnosticCode.CheckLimit => "Search incomplete — check limit",
            PlanDiagnosticCode.TimeLimit => "Search incomplete — time limit",
            PlanDiagnosticCode.AttemptLimit => "Search incomplete — attempt limit",
            PlanDiagnosticCode.IterationLimit => "Search incomplete — iteration limit",
            PlanDiagnosticCode.IkNoSolution => "IK unresolved at requested pose",
            PlanDiagnosticCode.EndpointValidation => "Endpoint pose validation failed",
            PlanDiagnosticCode.RobotReach => "Pose outside geometric reach",
            PlanDiagnosticCode.JointLimit => "Joint limit exceeded",
            PlanDiagnosticCode.TrackLimit => "Track limit exceeded",
            PlanDiagnosticCode.Singularity => "Singularity margin violated",
            PlanDiagnosticCode.StretchLimit => "Strict stretch limit exceeded",
            PlanDiagnosticCode.Collision => "Collision rejected",
            PlanDiagnosticCode.Clearance => "Clearance requirement violated",
            PlanDiagnosticCode.WristBranch => "No permitted wrist-branch connection",
            PlanDiagnosticCode.JointContinuity => "Joint continuity limit exceeded",
            PlanDiagnosticCode.PoseClosure => "TCP pose accuracy check failed",
            PlanDiagnosticCode.InvalidSetup => "Planner setup invalid",
            PlanDiagnosticCode.Cancelled => "Planning cancelled by user",
            PlanDiagnosticCode.MotionStopped => "Motion stopped by user",
            PlanDiagnosticCode.ConfigurationChanged => "Settings or scene changed",
            _ => "No complete connecting route found"
        };
        public string ToText()
        {
            var b = new StringBuilder(); b.AppendLine($"Run {RunId} · {StartedUtc} · {Planner}");
            b.AppendLine($"{Outcome} [{StopCode}]: {StatusTitle(StopCode)}"); b.AppendLine(Summary);
            b.AppendLine($"Queries {ConfigurationQueries}; actual collision evaluations {CollisionEvaluations}; cache hits {CacheHits}; connections {Connections}.");
            b.AppendLine(StartSummary); b.AppendLine(EndSummary); b.AppendLine("Settings snapshot: " + SettingsSnapshot);
            b.AppendLine($"Historical/stale: {Stale}. Omitted representative events: {DroppedEvents}. Observations are not unique geometric obstructions.");
            foreach (var c in Counters) b.AppendLine(c.Code + ": " + c.Observations + " rejection observations");
            foreach (var e in Events) b.AppendLine($"\nEvent {e.Id}\n" + e.Details());
            b.AppendLine("A bounded search failure does not prove that all robot/track routes are impossible. TCP markers identify rejected configurations, not physical contact points.");
            return b.ToString();
        }
    }
}
