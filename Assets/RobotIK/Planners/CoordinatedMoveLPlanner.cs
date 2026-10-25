using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    public sealed class CoordinatedStateEvaluation
    {
        public bool Valid;
        public Pose Tcp;
        public string Failure;
    }

    /// <summary>Bounded layered candidate search over a prescribed piecewise Cartesian path.
    /// Scene callbacks are synchronous; the enumerator yields between small batches.</summary>
    public sealed class CoordinatedMoveLPlanner
    {
        public sealed class World
        {
            public Func<Pose, float, IReadOnlyList<ArmTrackState>> Solve;
            public Func<ArmTrackState, CoordinatedStateEvaluation> Evaluate;
            public Func<bool> ShouldStop;
            public Func<Pose, float> StationForPose;
            public float? WristZeroOffsetDegrees;
            public Func<ArmTrackState, float> Penalty;
            public Action<string> Progress;
            public Action<PlanDiagnosticCode, ArmTrackState, Pose?, string, float?, float?> Rejected;
            public Action<PlanDiagnosticCode, string> LimitReached;
            public IReadOnlyList<float> StartStations;
            public float MinimumE1, MaximumE1, PreferredE1;
            public float CartesianStepMillimeters = 20, TrackStepMillimeters = 40;
            public float PositionToleranceMeters = .001f, RotationToleranceDegrees = .25f;
            public float JointSweepStepDegrees = .25f, TrackSweepStepMillimeters = 2;
            public int BeamWidth = 12, MaximumSamples = 2048, MaximumChecks = 100000;
        }
        public sealed class Result
        {
            public bool Found;
            public PlanDiagnosticCode StopCode = PlanDiagnosticCode.NoCompleteRoute;
            public string Failure = "No continuous coordinated MoveL route.";
            public int Checks, Layers;
            public readonly List<ArmTrackState> States = new List<ArmTrackState>();
            public readonly List<Pose> Poses = new List<Pose>();
        }
        private sealed class Node
        {
            public ArmTrackState State;
            public Node Parent;
            public double Cost;
            public float PeakPenalty;
            public Pose Pose;
            public List<ArmTrackState> Edge;
            public List<Pose> EdgePoses;
        }
        private readonly World world;
        private bool reportedCheckLimit;
        private readonly System.Diagnostics.Stopwatch frame = System.Diagnostics.Stopwatch.StartNew();
        public CoordinatedMoveLPlanner(World world) { this.world = world ?? throw new ArgumentNullException(nameof(world)); }
        private bool YieldFrame() => frame.Elapsed.TotalMilliseconds >= 3;
        private bool Stop(Result result)
        {
            if (world.ShouldStop?.Invoke() == true) { result.Failure = "Coordinated search cancelled or time budget reached."; return true; }
            if (result.Checks >= world.MaximumChecks) { result.StopCode = PlanDiagnosticCode.CheckLimit; result.Failure = "Coordinated configuration-check limit reached.";
                if (!reportedCheckLimit) { reportedCheckLimit = true; world.LimitReached?.Invoke(PlanDiagnosticCode.CheckLimit, result.Failure + $" {result.Checks}/{world.MaximumChecks} checks."); } return true; }
            return false;
        }
        private static Pose Mix(Pose a, Pose b, float t) => new Pose(Vector3.Lerp(a.position, b.position, t), Quaternion.Slerp(a.rotation, b.rotation, t));
        public static ArmTrackState Interpolate(ArmTrackState a, ArmTrackState b, float t)
        {
            var q = new float[6]; for (int i = 0; i < 6; i++) q[i] = a[i] + (b[i] - a[i]) * t;
            return new ArmTrackState(q, a.E1Millimeters + (b.E1Millimeters - a.E1Millimeters) * t);
        }
        private bool Check(ArmTrackState state, Pose pose, Result result)
        {
            if (Stop(result)) return false;
            if (state == null || !state.HasTrackAxis || state.E1Millimeters < world.MinimumE1 || state.E1Millimeters > world.MaximumE1)
            { result.Failure = "Invalid coordinated state or E1 range."; world.Rejected?.Invoke(PlanDiagnosticCode.TrackLimit, state, pose, result.Failure, null, null); return false; }
            result.Checks++;
            var check = world.Evaluate(state);
            if (check == null || !check.Valid) { result.Failure = check?.Failure ?? "Invalid combined state."; return false; }
            if (!FinitePose(check.Tcp) || Vector3.Distance(check.Tcp.position, pose.position) > world.PositionToleranceMeters ||
                Quaternion.Angle(check.Tcp.rotation, pose.rotation) > world.RotationToleranceDegrees)
            { result.Failure = "Combined interpolation departs from the exact Cartesian segment."; world.Rejected?.Invoke(PlanDiagnosticCode.PoseClosure, state, check.Tcp, result.Failure, null, null); return false; }
            return true;
        }
        public IEnumerator Search(IReadOnlyList<Pose> targets, Result result)
        {
            if (result == null) throw new ArgumentNullException(nameof(result));
            result.Found = false; result.States.Clear(); result.Poses.Clear(); result.Checks = result.Layers = 0;
            if (targets == null || targets.Count < 2 || world.StartStations == null || world.StartStations.Count == 0 ||
                world.Solve == null || world.Evaluate == null || !LinearTrackAxis.Finite(world.CartesianStepMillimeters) || !LinearTrackAxis.Finite(world.TrackStepMillimeters) ||
                !LinearTrackAxis.Finite(world.MinimumE1) || !LinearTrackAxis.Finite(world.MaximumE1) || !LinearTrackAxis.Finite(world.PreferredE1) ||
                !LinearTrackAxis.Finite(world.JointSweepStepDegrees) || !LinearTrackAxis.Finite(world.TrackSweepStepMillimeters) ||
                !LinearTrackAxis.Finite(world.PositionToleranceMeters) || !LinearTrackAxis.Finite(world.RotationToleranceDegrees) ||
                world.PositionToleranceMeters < 0 || world.RotationToleranceDegrees < 0 || world.JointSweepStepDegrees <= 0 || world.TrackSweepStepMillimeters <= 0 ||
                world.MinimumE1 > world.MaximumE1 || world.CartesianStepMillimeters <= 0 || world.TrackStepMillimeters <= 0 || world.BeamWidth < 1 || world.MaximumSamples < 2 || world.MaximumChecks < 1)
            { result.Failure = "Invalid coordinated search inputs."; yield break; }
            foreach (var target in targets) if (!FinitePose(target)) { result.Failure = "Cartesian target pose must be finite and have a valid rotation."; yield break; }
            for (int refinement = 0; refinement < 4; refinement++)
            {
                if (Stop(result)) yield break;
                var samples = Samples(targets, world.CartesianStepMillimeters / (1 << refinement));
                if (samples == null) { result.StopCode = PlanDiagnosticCode.CheckLimit; result.Failure = "Coordinated Cartesian sample limit reached."; world.LimitReached?.Invoke(PlanDiagnosticCode.CheckLimit, result.Failure + $" Capacity {world.MaximumSamples} samples."); yield break; }
                var frontier = new List<Node>();
                foreach (float station in world.StartStations)
                {
                    if (!LinearTrackAxis.Finite(station) || station < world.MinimumE1 || station > world.MaximumE1) continue;
                    if (Stop(result)) yield break;
                    foreach (var state in world.Solve(targets[0], station) ?? Array.Empty<ArmTrackState>())
                    {
                        if (state != null && state.E1Millimeters.Equals(station) && Check(state, targets[0], result)) frontier.Add(new Node { State = state, Pose = targets[0], Cost = Math.Abs((double)station - world.PreferredE1) / 40, PeakPenalty = world.Penalty?.Invoke(state) ?? 0 });
                        if (YieldFrame()) { yield return null; frame.Restart(); }
                    }
                    if (YieldFrame()) { yield return null; frame.Restart(); }
                }
                frontier = Prune(frontier);
                if (frontier.Count == 0) { result.Failure = "No collision-free, nonsingular exact Start configuration at sampled E1 values."; yield break; }
                for (int layer = 1; layer < samples.Count && frontier.Count > 0; layer++)
                {
                    if (Stop(result)) yield break;
                    result.Layers++;
                    world.Progress?.Invoke($"Coordinated MoveL: sample {layer}/{samples.Count - 1}, {frontier.Count} continuous candidates, refinement {refinement + 1}/4.");
                    var solutions = new Dictionary<float, IReadOnlyList<ArmTrackState>>();
                    var next = new Dictionary<ArmTrackStateKey, Node>();
                    foreach (var parent in frontier)
                    {
                        foreach (float delta in new[] { 0f, -.5f, .5f, -1f, 1f })
                        {
                            if (world.StationForPose != null && delta != 0) continue;
                            if (Stop(result)) yield break;
                            float station = world.StationForPose != null ? world.StationForPose(samples[layer]) : Mathf.Clamp(parent.State.E1Millimeters + delta * world.TrackStepMillimeters, world.MinimumE1, world.MaximumE1);
                            if (!LinearTrackAxis.Finite(station) || station < world.MinimumE1 || station > world.MaximumE1) continue;
                            if (!solutions.TryGetValue(station, out var candidates))
                            { candidates = world.Solve(samples[layer], station) ?? Array.Empty<ArmTrackState>(); solutions[station] = candidates; }
                            foreach (var candidate in candidates)
                            {
                                if (Stop(result)) yield break;
                                if (candidate == null || !candidate.HasTrackAxis || !candidate.E1Millimeters.Equals(station) || Mathf.Abs(candidate.E1Millimeters - parent.State.E1Millimeters) > world.TrackStepMillimeters + .0001f) continue;
                                float jump = 0; double cost = parent.Cost + Math.Abs((double)candidate.E1Millimeters - parent.State.E1Millimeters) / 20;
                                for (int j = 0; j < 6; j++) { float d = Mathf.Abs(candidate[j] - parent.State[j]); jump = Mathf.Max(jump, d); cost += d * (j >= 3 ? 2 : 1); }
                                if (jump > 5.00001f) { result.Failure = "No continuous arm branch: raw joint step exceeds 5 degrees; wrist flips forbidden."; world.Rejected?.Invoke(PlanDiagnosticCode.JointContinuity, candidate, samples[layer], result.Failure, jump, 5f); continue; }
                                var key = new ArmTrackStateKey(candidate, 0);
                                if (world.Penalty == null && next.TryGetValue(key, out var better) && better.Cost <= cost) continue;
                                var edge = new List<ArmTrackState>(); var edgePoses = new List<Pose>();
                                bool valid = false;
                                var validation = Edge(parent, candidate, samples[layer], result, edge, edgePoses, ok => valid = ok);
                                while (validation.MoveNext()) yield return validation.Current;
                                if (valid)
                                {
                                    float peak = parent.PeakPenalty; double weighted = 0, length = 0; var previous = parent.State;
                                    if (world.Penalty != null) foreach (var state in edge)
                                    {
                                        float p = world.Penalty(state), prior = world.Penalty(previous); peak = Mathf.Max(peak, p, prior);
                                        double step = Math.Abs((double)state.E1Millimeters - previous.E1Millimeters) / 20;
                                        for (int j = 0; j < 6; j++) step += Math.Abs(state[j] - previous[j]) * (j >= 3 ? 2 : 1);
                                        weighted += step * (p + prior) * .5; length += step; previous = state;
                                    }
                                    if (length > 0) cost += (cost - parent.Cost) * weighted / length;
                                    var node = new Node { State = candidate, Parent = parent, Pose = samples[layer], Cost = cost, PeakPenalty = peak, Edge = edge, EdgePoses = edgePoses };
                                    if (!next.TryGetValue(key, out var old) || node.Cost + node.PeakPenalty * 100 < old.Cost + old.PeakPenalty * 100) next[key] = node;
                                }
                                if (YieldFrame()) { yield return null; frame.Restart(); }
                            }
                            if (YieldFrame()) { yield return null; frame.Restart(); }
                        }
                    }
                    frontier = Prune(new List<Node>(next.Values));
                }
                if (frontier.Count > 0)
                {
                    if (Stop(result)) yield break;
                    var chain = new List<Node>(); for (var node = frontier[0]; node != null; node = node.Parent) chain.Add(node); chain.Reverse();
                    result.States.Add(chain[0].State); result.Poses.Add(chain[0].Pose);
                    for (int i = 1; i < chain.Count; i++) { result.States.AddRange(chain[i].Edge); result.Poses.AddRange(chain[i].EdgePoses); }
                    result.Found = true; result.Failure = null; yield break;
                }
            }
            result.Failure = "Bounded coordinated search found no complete continuous route. " + result.Failure;
        }
        private IEnumerator Edge(Node from, ArmTrackState to, Pose pose, Result result, List<ArmTrackState> states, List<Pose> poses, Action<bool> done)
        {
            if (world.WristZeroOffsetDegrees.HasValue && Mathf.Sin((from.State[4] + world.WristZeroOffsetDegrees.Value) * Mathf.Deg2Rad) * Mathf.Sin((to[4] + world.WristZeroOffsetDegrees.Value) * Mathf.Deg2Rad) <= 0)
            { result.Failure = "Prescribed corridor would change wrist branch."; world.Rejected?.Invoke(PlanDiagnosticCode.WristBranch, to, pose, result.Failure, null, null); done(false); yield break; }
            float joint = 0; for (int j = 0; j < 6; j++) joint = Mathf.Max(joint, Mathf.Abs(from.State[j] - to[j]));
            int steps = Mathf.Max(2, Mathf.CeilToInt(joint / Mathf.Max(.01f, world.JointSweepStepDegrees)));
            steps = Mathf.Max(steps, Mathf.CeilToInt(Mathf.Abs(to.E1Millimeters - from.State.E1Millimeters) / world.TrackSweepStepMillimeters));
            steps = Mathf.Max(steps, Mathf.CeilToInt(Vector3.Distance(from.Pose.position, pose.position) / .002f));
            steps = Mathf.Max(steps, Mathf.CeilToInt(Quaternion.Angle(from.Pose.rotation, pose.rotation) / .5f));
            if (steps > 512) { result.Failure = "Combined edge sweep limit exceeded."; done(false); yield break; }
            for (int i = 1; i <= steps; i++)
            {
                float t = i / (float)steps; var state = i == steps ? to : Interpolate(from.State, to, t); var expected = Mix(from.Pose, pose, t);
                if (!Check(state, expected, result)) { done(false); yield break; }
                states.Add(state); poses.Add(expected);
                if (YieldFrame()) { yield return null; frame.Restart(); }
            }
            done(true);
        }
        private List<Pose> Samples(IReadOnlyList<Pose> targets, float step)
        {
            var poses = new List<Pose> { targets[0] };
            for (int i = 1; i < targets.Count; i++)
            {
                double count = Math.Max(1, Math.Max(Math.Ceiling(Vector3.Distance(targets[i - 1].position, targets[i].position) * 1000 / step), Math.Ceiling(Quaternion.Angle(targets[i - 1].rotation, targets[i].rotation) / 2)));
                if (count > world.MaximumSamples - poses.Count) return null;
                for (int j = 1; j <= (int)count; j++) poses.Add(j == (int)count ? targets[i] : Mix(targets[i - 1], targets[i], j / (float)count));
            }
            return poses;
        }
        internal static bool FinitePose(Pose pose)
        {
            var q = pose.rotation;
            float norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            return LinearTrackAxis.Finite(pose.position) && LinearTrackAxis.Finite(q.x) && LinearTrackAxis.Finite(q.y) && LinearTrackAxis.Finite(q.z) && LinearTrackAxis.Finite(q.w) &&
                LinearTrackAxis.Finite(norm) && Mathf.Abs(norm - 1f) < .001f;
        }
        private List<Node> Prune(List<Node> nodes)
        {
            nodes.Sort((a, b) => { int c = (a.Cost + a.PeakPenalty * 100).CompareTo(b.Cost + b.PeakPenalty * 100); return c != 0 ? c : a.State.E1Millimeters.CompareTo(b.State.E1Millimeters); });
            var result = new List<Node>(); var counts = new Dictionary<float, int>();
            foreach (var node in nodes)
            {
                counts.TryGetValue(node.State.E1Millimeters, out int count);
                if (count >= 2) continue;
                result.Add(node); counts[node.State.E1Millimeters] = count + 1;
                if (result.Count >= world.BeamWidth) break;
            }
            return result;
        }
    }
}
