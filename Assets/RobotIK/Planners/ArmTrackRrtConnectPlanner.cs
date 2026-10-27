using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Bidirectional configuration-space search. Edges are densely checked
    /// in all seven physical coordinates; Cartesian guide geometry is only a bias.</summary>
    public sealed class ArmTrackRrtConnectPlanner
    {
        private sealed class Node
        {
            public ArmTrackState State;
            public Pose Pose;
            public Node Parent;
            public List<ArmTrackState> Edge;
            public List<Pose> Poses;
        }
        private readonly TrackTransferPlanner.World world;
        private readonly float minimumE1, maximumE1;
        private readonly System.Diagnostics.Stopwatch slice = System.Diagnostics.Stopwatch.StartNew();
        public int StartTreeNodes { get; private set; }
        public int GoalTreeNodes { get; private set; }
        public ArmTrackRrtConnectPlanner(TrackTransferPlanner.World world, float minimumE1, float maximumE1)
        { this.world = world ?? throw new ArgumentNullException(nameof(world)); this.minimumE1 = minimumE1; this.maximumE1 = maximumE1; }
        private bool Stop(TrackTransferPlanner.Result r) => world.ShouldStop?.Invoke() == true || r.Checks >= world.MaximumChecks;
        private bool Branch(ArmTrackState a, ArmTrackState b) =>
            Mathf.Sin((a[4] + world.WristZeroOffsetDegrees) * Mathf.Deg2Rad) * Mathf.Sin((b[4] + world.WristZeroOffsetDegrees) * Mathf.Deg2Rad) > 0;
        private double Distance(ArmTrackState a, ArmTrackState b)
        {
            double e1 = (a.E1Millimeters - b.E1Millimeters) / Math.Max(100d, (maximumE1 - minimumE1) * .25d), d = e1 * e1;
            for (int j = 0; j < 6; j++) { double q = (a[j] - b[j]) / 45d; d += q * q * (j >= 3 ? 1.5 : 1); }
            return d;
        }
        private bool Check(ArmTrackState state, TrackTransferPlanner.Result r, out Pose pose)
        {
            pose = default;
            if (Stop(r)) return false;
            if (!state.HasTrackAxis || state.E1Millimeters < minimumE1 || state.E1Millimeters > maximumE1) return false;
            for (int j = 0; j < 6; j++) if (state[j] < world.JointLimits[j].x || state[j] > world.JointLimits[j].y) return false;
            r.Checks++; var check = world.Evaluate(state);
            if (check == null || !check.Valid || !CoordinatedMoveLPlanner.FinitePose(check.Tcp))
            { r.Failure = check?.Failure ?? "Invalid combined configuration."; return false; }
            pose = check.Tcp; return true;
        }
        private IEnumerator Edge(Node parent, ArmTrackState to, TrackTransferPlanner.Result r, Action<Node> done, Func<bool> trialAllowed = null)
        {
            if (!Branch(parent.State, to))
            { r.Failure="Hybrid edge would change wrist branch.";world.Rejected?.Invoke(PlanDiagnosticCode.WristBranch,to,null,r.Failure,null,null);done(null);yield break; }
            float angle = 0;
            for (int j = 0; j < 6; j++) angle = Mathf.Max(angle, Mathf.Abs(to[j] - parent.State[j]));
            double samples = Math.Max(1, Math.Ceiling(Math.Max((double)angle / world.JointStepDegrees,
                Math.Abs((double)to.E1Millimeters - parent.State.E1Millimeters) / world.TrackStepMillimeters)));
            if (samples > 4096) { r.Failure="Hybrid edge sweep exceeds 4096 samples.";done(null);yield break; }
            int count = (int)samples;
            var states = new List<ArmTrackState>(); var poses = new List<Pose>();
            for (int i = 1; i <= count; i++)
            {
                if (trialAllowed != null && !trialAllowed()) { done(null); yield break; }
                var state = i == count ? to : CoordinatedMoveLPlanner.Interpolate(parent.State, to, i / (float)count);
                if (!Branch(parent.State, state))
                { r.Failure="Hybrid interpolation crosses a wrist branch boundary.";world.Rejected?.Invoke(PlanDiagnosticCode.WristBranch,state,null,r.Failure,null,null);done(null);yield break; }
                if (!Check(state, r, out var pose)) { done(null); yield break; }
                states.Add(state); poses.Add(pose);
                if (slice.Elapsed.TotalMilliseconds >= 3) { yield return null; slice.Restart(); }
            }
            done(new Node { State = to, Pose = poses[poses.Count - 1], Parent = parent, Edge = states, Poses = poses });
        }
        private IEnumerator Extend(List<Node> tree, ArmTrackState target, TrackTransferPlanner.Result r, Action<Node> done)
        {
            Node nearest = null; double closest = double.PositiveInfinity;
            foreach (var node in tree)
            {
                double distance = Distance(node.State, target) * (1 + (world.Penalty?.Invoke(node.State) ?? 0));
                if (distance < closest) { closest = distance; nearest = node; }
            }
            if (nearest == null || Distance(nearest.State, target) < 1e-18) { done(nearest); yield break; }
            float scale = Mathf.Abs(target.E1Millimeters - nearest.State.E1Millimeters) / Mathf.Clamp((maximumE1-minimumE1)/20,5,80);
            for (int j = 0; j < 6; j++) scale = Mathf.Max(scale, Mathf.Abs(target[j] - nearest.State[j]) / 8);
            var to = scale <= 1 ? target : CoordinatedMoveLPlanner.Interpolate(nearest.State, target, 1 / scale);
            Node added = null; var edge = Edge(nearest, to, r, value => added = value);
            try { while (edge.MoveNext()) yield return edge.Current; } finally { (edge as IDisposable)?.Dispose(); }
            if (added != null) tree.Add(added);
            done(added);
        }
        private static void Publish(Node forward, Node backward, TrackTransferPlanner.Result result)
        {
            var chain = new List<Node>(); for (var n = forward; n != null; n = n.Parent) chain.Add(n); chain.Reverse();
            result.States.Add(chain[0].State); result.Poses.Add(chain[0].Pose);
            for (int i = 1; i < chain.Count; i++) { result.States.AddRange(chain[i].Edge); result.Poses.AddRange(chain[i].Poses); }
            for (var node = backward; node.Parent != null; node = node.Parent)
            {
                for (int i = node.Edge.Count - 2; i >= 0; i--) { result.States.Add(node.Edge[i]); result.Poses.Add(node.Poses[i]); }
                result.States.Add(node.Parent.State); result.Poses.Add(node.Parent.Pose);
            }
            result.Found = true; result.Failure = null;
        }
        public IEnumerator Search(ArmTrackState start, ArmTrackState goal, IReadOnlyList<float[]> postures, TrackTransferPlanner.Result result)
        {
            result.Found = false; result.States.Clear(); result.Poses.Clear(); result.Checks = result.Iterations = 0;
            StartTreeNodes = GoalTreeNodes = 0;
            if (start == null || goal == null || world.Evaluate == null || world.JointLimits == null || world.JointLimits.Length != 6 ||
                !LinearTrackAxis.Finite(minimumE1) || !LinearTrackAxis.Finite(maximumE1) || minimumE1 > maximumE1 ||
                !LinearTrackAxis.Finite(world.JointStepDegrees) || !LinearTrackAxis.Finite(world.TrackStepMillimeters) ||
                world.JointStepDegrees <= 0 || world.TrackStepMillimeters <= 0 || world.MaximumChecks < 1 || world.MaximumIterations < 1)
            { result.Failure = "Invalid hybrid RRT-Connect inputs."; yield break; }
            foreach (var limit in world.JointLimits) if (!LinearTrackAxis.Finite(limit.x) || !LinearTrackAxis.Finite(limit.y) || limit.x > limit.y)
            { result.Failure = "Invalid hybrid joint limits."; yield break; }
            if (!Check(start, result, out var a) || !Check(goal, result, out var b)) yield break;
            if (!Branch(start, goal)) { result.Failure = "Hybrid connection would change wrist branch."; yield break; }
            var first = new Node { State = start, Pose = a }; var last = new Node { State = goal, Pose = b };
            // Cheap direct configuration edge before sampling. A failed edge is
            // discarded completely; neither it nor its partial prefix executes.
            Node direct = null; var directSearch = Edge(first, goal, result, n => direct = n);
            try { while (directSearch.MoveNext()) yield return directSearch.Current; } finally { (directSearch as IDisposable)?.Dispose(); }
            if (direct != null && !Stop(result)) { StartTreeNodes=2;GoalTreeNodes=1;Publish(direct,last,result);yield break; }
            var startTree = new List<Node> { first }; var goalTree = new List<Node> { last }; var random = new System.Random(world.Seed);
            // Seed both trees with held carry postures. This is a cheap connection
            // opportunity, not a shortcut around validation: every retract, rail
            // sweep and approach is checked at the same dense resolution.
            int seedStart = result.Checks;
            bool SeedAllowed() => result.Checks - seedStart < 1000;
            foreach (var posture in postures ?? Array.Empty<float[]>())
            {
                if (Stop(result)) yield break;
                if (!SeedAllowed()) break;
                bool unchanged = true; for (int j = 0; j < 6; j++) if (posture[j] != start[j]) unchanged = false;
                if (unchanged) continue;
                Node departure = null, arrival = null, bridge = null;
                var retract = Edge(first, new ArmTrackState(posture, start.E1Millimeters), result, n => departure = n, SeedAllowed);
                try { while (retract.MoveNext()) yield return retract.Current; } finally { (retract as IDisposable)?.Dispose(); }
                if (departure == null) continue;
                var approach = Edge(last, new ArmTrackState(posture, goal.E1Millimeters), result, n => arrival = n, SeedAllowed);
                try { while (approach.MoveNext()) yield return approach.Current; } finally { (approach as IDisposable)?.Dispose(); }
                if (arrival == null) continue;
                var carry = Edge(departure, arrival.State, result, n => bridge = n, SeedAllowed);
                try { while (carry.MoveNext()) yield return carry.Current; } finally { (carry as IDisposable)?.Dispose(); }
                if (bridge != null && !Stop(result))
                {
                    StartTreeNodes = 3; GoalTreeNodes = 2;
                    Publish(bridge, arrival, result); yield break;
                }
            }
            for (int iteration = 0; iteration < world.MaximumIterations && !Stop(result); iteration++)
            {
                result.Iterations = iteration + 1;
                bool forward = iteration % 2 == 0; var grow = forward ? startTree : goalTree; var other = forward ? goalTree : startTree;
                ArmTrackState target;
                double choice = random.NextDouble();
                if (choice < .2) target = other[random.Next(other.Count)].State;
                else if (choice < .5 && postures != null && postures.Count > 0)
                    target = new ArmTrackState(postures[random.Next(postures.Count)], Mathf.Lerp(minimumE1, maximumE1, (float)random.NextDouble()));
                else
                {
                    var q = new float[6]; for (int j = 0; j < 6; j++) q[j] = Mathf.Lerp(world.JointLimits[j].x, world.JointLimits[j].y, (float)random.NextDouble());
                    target = new ArmTrackState(q, Mathf.Lerp(minimumE1, maximumE1, (float)random.NextDouble()));
                }
                if (!Branch(start, target)) continue;
                Node grown = null; var extension = Extend(grow, target, result, n => grown = n);
                try { while (extension.MoveNext()) yield return extension.Current; } finally { (extension as IDisposable)?.Dispose(); }
                if (grown == null) continue;
                for (int connect = 0; connect < 64 && !Stop(result); connect++)
                {
                    Node joined = null; var search = Extend(other, grown.State, result, n => joined = n);
                    try { while (search.MoveNext()) yield return search.Current; } finally { (search as IDisposable)?.Dispose(); }
                    if (joined == null) break;
                    if (Distance(joined.State, grown.State) < 1e-18)
                    {
                        if (!Stop(result)) Publish(forward ? grown : joined, forward ? joined : grown, result);
                        StartTreeNodes = startTree.Count; GoalTreeNodes = goalTree.Count; yield break;
                    }
                }
                StartTreeNodes = startTree.Count; GoalTreeNodes = goalTree.Count;
                world.Progress?.Invoke($"Arm + E1 RRT-Connect · {StartTreeNodes}/{GoalTreeNodes} tree nodes · {result.Checks} checks");
                if (slice.Elapsed.TotalMilliseconds >= 3) { yield return null; slice.Restart(); }
            }
            result.Failure = "No complete hybrid RRT-Connect route within the search bounds. " + result.Failure;
            if(result.Checks>=world.MaximumChecks)world.LimitReached?.Invoke(PlanDiagnosticCode.CheckLimit,$"Hybrid connection reached {result.Checks}/{world.MaximumChecks} configuration queries.");
            else if(result.Iterations>=world.MaximumIterations)world.LimitReached?.Invoke(PlanDiagnosticCode.IterationLimit,$"Hybrid connection reached {result.Iterations}/{world.MaximumIterations} extension rounds.");
        }
    }
}
