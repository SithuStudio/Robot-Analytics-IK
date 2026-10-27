using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Collision-checked joint/track transfer with free TCP. E1 progresses toward
    /// the destination; arm retracts/repositions continuously without wrist-branch changes.</summary>
    public sealed class TrackTransferPlanner
    {
        public sealed class World
        {
            public Func<ArmTrackState, CoordinatedStateEvaluation> Evaluate;
            public Func<bool> ShouldStop;
            public Action<string> Progress;
            public Action<PlanDiagnosticCode, ArmTrackState, Pose?, string, float?, float?> Rejected;
            public Action<PlanDiagnosticCode, string> LimitReached;
            public Vector2[] JointLimits;
            public int MaximumIterations = 2000;
            public int MaximumChecks = 30000;
            public float JointStepDegrees = .25f;
            public float TrackStepMillimeters = 2;
            public float WristZeroOffsetDegrees;
            public int Seed = 617;
            public Func<bool> CanRefine;
            public bool RefineAutomatically = true;
            public Func<ArmTrackState, float> Penalty;
        }
        public sealed class Result
        {
            public bool Found;
            public string Failure;
            public int Checks, Iterations;
            public int RemovedControlPoints;
            public readonly List<ArmTrackState> States = new List<ArmTrackState>();
            public readonly List<Pose> Poses = new List<Pose>();
        }
        private sealed class Node
        { public ArmTrackState State; public Node Parent; public List<ArmTrackState> Edge; public List<Pose> Poses; }
        private readonly World world;
        private bool reportedCheckLimit;
        private readonly System.Diagnostics.Stopwatch frame = System.Diagnostics.Stopwatch.StartNew();
        public TrackTransferPlanner(World world) { this.world = world ?? throw new ArgumentNullException(nameof(world)); }
        private bool Stop(Result r)
        {
            if (world.ShouldStop?.Invoke() == true) { r.Failure = "Transfer cancelled or time budget reached."; return true; }
            if (r.Checks >= world.MaximumChecks) { r.Failure = "Transfer configuration-check limit reached.";
                if (!reportedCheckLimit) { reportedCheckLimit = true; world.LimitReached?.Invoke(PlanDiagnosticCode.CheckLimit, r.Failure + $" {r.Checks}/{world.MaximumChecks} checks."); } return true; }
            return false;
        }
        private bool SameWristBranch(ArmTrackState a, ArmTrackState b)
        {
            float x = Mathf.Sin((a[4] + world.WristZeroOffsetDegrees) * Mathf.Deg2Rad);
            float y = Mathf.Sin((b[4] + world.WristZeroOffsetDegrees) * Mathf.Deg2Rad);
            return x * y > 0;
        }
        private bool Check(ArmTrackState state, Result r, out Pose pose)
        {
            pose = default;
            if (Stop(r)) return false;
            for (int i = 0; i < 6; i++)
                if (state[i] < world.JointLimits[i].x || state[i] > world.JointLimits[i].y)
                { r.Failure = "Transfer posture exceeds joint limits."; return false; }
            r.Checks++; var check = world.Evaluate(state);
            if (check == null || !check.Valid || !CoordinatedMoveLPlanner.FinitePose(check.Tcp))
            { r.Failure = check?.Failure ?? "Invalid transfer state."; return false; }
            pose = check.Tcp; return true;
        }
        private IEnumerator Edge(ArmTrackState a, ArmTrackState b, Result result, List<ArmTrackState> states, List<Pose> poses, Action<bool> done, Func<bool> continueSampling = null)
        {
            if (!SameWristBranch(a, b)) { result.Failure = "Transfer would change wrist branch."; world.Rejected?.Invoke(PlanDiagnosticCode.WristBranch, b, null, result.Failure, null, null); done(false); yield break; }
            float angle = 0; for (int i = 0; i < 6; i++) angle = Mathf.Max(angle, Mathf.Abs(b[i] - a[i]));
            double samples = Math.Max(1, Math.Max(Math.Ceiling(angle / world.JointStepDegrees), Math.Ceiling(Math.Abs((double)b.E1Millimeters - a.E1Millimeters) / world.TrackStepMillimeters)));
            if (samples > 4096) { result.Failure = "Transfer edge sweep limit reached."; done(false); yield break; }
            for (int i = 1; i <= (int)samples; i++)
            {
                if (continueSampling != null && !continueSampling()) { done(false); yield break; }
                var state = i == (int)samples ? b : CoordinatedMoveLPlanner.Interpolate(a, b, i / (float)samples);
                if (!Check(state, result, out Pose pose)) { done(false); yield break; }
                states.Add(state); poses.Add(pose);
                if (frame.Elapsed.TotalMilliseconds >= 3) { yield return null; frame.Restart(); }
            }
            done(true);
        }
        public IEnumerator Search(ArmTrackState start, ArmTrackState goal, IReadOnlyList<float[]> transportPostures, Result result)
        {
            var search = SearchRaw(start, goal, transportPostures, result);
            while (search.MoveNext()) yield return search.Current;
            if (!result.Found || !world.RefineAutomatically) yield break;
            var refinement = Refine(result);
            while (refinement.MoveNext()) yield return refinement.Current;
            result.Failure = null;
        }
        private IEnumerator SearchRaw(ArmTrackState start, ArmTrackState goal, IReadOnlyList<float[]> transportPostures, Result result)
        {
            result.Found = false; result.States.Clear(); result.Poses.Clear(); result.Checks = result.Iterations = 0;
            result.RemovedControlPoints = 0;
            if (start == null || goal == null || !start.HasTrackAxis || !goal.HasTrackAxis || world.Evaluate == null ||
                world.JointLimits == null || world.JointLimits.Length != 6 || !LinearTrackAxis.Finite(world.JointStepDegrees) ||
                !LinearTrackAxis.Finite(world.TrackStepMillimeters) || world.JointStepDegrees <= 0 || world.TrackStepMillimeters <= 0)
            { result.Failure = "Invalid transfer request."; yield break; }
            foreach (var limit in world.JointLimits)
                if (!LinearTrackAxis.Finite(limit.x) || !LinearTrackAxis.Finite(limit.y) || limit.x > limit.y) { result.Failure = "Invalid transfer joint limits."; yield break; }
            if (!Check(start, result, out Pose startPose) || !Check(goal, result, out _)) yield break;
            if (!SameWristBranch(start, goal)) { result.Failure = "No same-wrist-branch transfer to approach pose."; world.Rejected?.Invoke(PlanDiagnosticCode.WristBranch, goal, null, result.Failure, null, null); yield break; }
            // Cheap structured trials: retract at departure, carry one posture along
            // the rail, then reach the approach posture with E1 already locked.
            foreach (var posture in transportPostures ?? Array.Empty<float[]>())
            {
                if (Stop(result)) yield break;
                var route = new List<ArmTrackState> { start }; var routePoses = new List<Pose> { startPose };
                var departure = new ArmTrackState(posture, start.E1Millimeters); var arrival = new ArmTrackState(posture, goal.E1Millimeters);
                bool all = true;
                foreach (var pair in new[] { new[] { start, departure }, new[] { departure, arrival }, new[] { arrival, goal } })
                {
                    bool valid = false; var edge = Edge(pair[0], pair[1], result, route, routePoses, ok => valid = ok);
                    while (edge.MoveNext()) yield return edge.Current;
                    if (!valid) { all = false; break; }
                }
                if (all && world.ShouldStop?.Invoke() != true)
                { result.States.AddRange(route); result.Poses.AddRange(routePoses); result.Found = true; result.Failure = null; yield break; }
            }
            // General continuous seven-coordinate transfer if a held posture cannot pass.
            var random = new System.Random(world.Seed); var tree = new List<Node> { new Node { State = start } };
            float direction = Mathf.Sign(goal.E1Millimeters - start.E1Millimeters);
            for (int iteration = 0; iteration < world.MaximumIterations; iteration++)
            {
                if (frame.Elapsed.TotalMilliseconds >= 3) { yield return null; frame.Restart(); }
                if (Stop(result)) yield break;
                result.Iterations = iteration + 1;
                world.Progress?.Invoke($"Transfer posture search {iteration + 1}/{world.MaximumIterations}; {result.Checks} swept checks.");
                ArmTrackState sample;
                if (random.NextDouble() < .3) sample = goal;
                else
                {
                    var q = new float[6]; for (int j = 0; j < 6; j++) q[j] = Mathf.Lerp(world.JointLimits[j].x, world.JointLimits[j].y, (float)random.NextDouble());
                    sample = new ArmTrackState(q, Mathf.Lerp(start.E1Millimeters, goal.E1Millimeters, (float)random.NextDouble()));
                }
                Node nearest = null; double best = double.PositiveInfinity;
                foreach (var node in tree)
                {
                    if (direction * (sample.E1Millimeters - node.State.E1Millimeters) < -.0001f) continue;
                    double distance = Distance(node.State, sample) * (1 + (world.Penalty?.Invoke(node.State) ?? 0)); if (distance < best) { best = distance; nearest = node; }
                }
                if (nearest == null) continue;
                // Directed E1 progress makes distant arm postures at the frontier
                // look worse than departure nodes. Occasionally expand a frontier
                // node directly so the arm can retract again between obstacles.
                if (random.NextDouble() < .25)
                {
                    var candidate = tree[random.Next(tree.Count)];
                    if (direction * (sample.E1Millimeters - candidate.State.E1Millimeters) >= -.0001f) nearest = candidate;
                }
                if (random.NextDouble() < .3) sample = new ArmTrackState(sample.JointAnglesDegrees, nearest.State.E1Millimeters);
                float fraction = (float)Math.Min(1, 8 / Math.Max(.0001, Distance(nearest.State, sample)));
                var next = CoordinatedMoveLPlanner.Interpolate(nearest.State, sample, fraction);
                var edgeStates = new List<ArmTrackState>(); var edgePoses = new List<Pose>(); bool connected = false;
                var extension = Edge(nearest.State, next, result, edgeStates, edgePoses, ok => connected = ok);
                while (extension.MoveNext()) yield return extension.Current;
                if (!connected) continue;
                var added = new Node { State = next, Parent = nearest, Edge = edgeStates, Poses = edgePoses }; tree.Add(added);
                if (Distance(next, goal) > 16) continue;
                var tailStates = new List<ArmTrackState>(); var tailPoses = new List<Pose>(); connected = false;
                var tail = Edge(next, goal, result, tailStates, tailPoses, ok => connected = ok);
                while (tail.MoveNext()) yield return tail.Current;
                if (!connected) continue;
                if (world.ShouldStop?.Invoke() == true) yield break;
                var chain = new List<Node>(); for (var node = added; node.Parent != null; node = node.Parent) chain.Add(node); chain.Reverse();
                result.States.Add(start); result.Poses.Add(startPose);
                foreach (var node in chain) { result.States.AddRange(node.Edge); result.Poses.AddRange(node.Poses); }
                result.States.AddRange(tailStates); result.Poses.AddRange(tailPoses); result.Found = true; result.Failure = null; yield break;
            }
            result.Failure = "No validated transfer within the bounded posture search. " + result.Failure;
            if (result.Iterations >= world.MaximumIterations) world.LimitReached?.Invoke(PlanDiagnosticCode.IterationLimit, $"Local transfer attempt exhausted {result.Iterations}/{world.MaximumIterations} posture iterations.");
        }
        // Shortcut the actual configuration route, never just its drawn TCP line.
        // Every replacement retains dense FK, collision and singularity validation.
        // Budget exhaustion keeps the complete route, including any accepted shortcuts.
        public IEnumerator Refine(Result result)
        {
            if (!result.Found || result.States.Count < 3) yield break;
            var anchors = new List<int> { 0 };
            for (int i = 1; i < result.States.Count - 1; i++)
                if (!SameDirection(result.States[i - 1], result.States[i], result.States[i + 1])) anchors.Add(i);
            anchors.Add(result.States.Count - 1);
            var output = new List<ArmTrackState> { result.States[0] };
            var poses = new List<Pose> { result.Poses[0] };
            int checks = result.Checks, attempts = 0, at = 0;
            var budget = System.Diagnostics.Stopwatch.StartNew();
            bool Available() => attempts < 96 && result.Checks - checks < 6000 && budget.Elapsed.TotalMilliseconds < 2000 &&
                result.Checks < world.MaximumChecks - 128 && world.ShouldStop?.Invoke() != true && (world.CanRefine?.Invoke() ?? true);
            while (at < anchors.Count - 1)
            {
                int selected = at + 1;
                List<ArmTrackState> replacement = null; List<Pose> replacementPoses = null;
                // Farthest first, then progressively shorter skips. Avoid spending
                // the entire refinement budget on a single obstructed departure.
                for (int span = anchors.Count - 1 - at; span >= 2 && Available(); span /= 2)
                {
                    int to = at + span; attempts++;
                    world.Progress?.Invoke($"Refining transfer: shortcut {attempts}/96; {result.RemovedControlPoints} unnecessary corners removed.");
                    var candidate = new List<ArmTrackState>(); var candidatePoses = new List<Pose>(); bool valid = false;
                    var edge = Edge(result.States[anchors[at]], result.States[anchors[to]], result, candidate, candidatePoses, ok => valid = ok, Available);
                    while (edge.MoveNext()) yield return edge.Current;
                    if (!valid) continue;
                    if (world.Penalty != null)
                    {
                        double Score(IReadOnlyList<ArmTrackState> list, int begin, int end, out float peak)
                        {
                            double score = 0; peak = world.Penalty(list[begin]);
                            for (int i = begin + 1; i <= end; i++)
                            { float a = world.Penalty(list[i - 1]), b = world.Penalty(list[i]); peak = Mathf.Max(peak, a, b); score += Distance(list[i - 1], list[i]) * (1 + .5 * (a + b)); }
                            return score;
                        }
                        var proposed = new List<ArmTrackState> { result.States[anchors[at]] }; proposed.AddRange(candidate);
                        double oldScore = Score(result.States, anchors[at], anchors[to], out float oldPeak);
                        double newScore = Score(proposed, 0, proposed.Count - 1, out float newPeak);
                        if (newPeak > oldPeak + .00001f || newScore > oldScore + .0001) continue;
                    }
                    selected = to; replacement = candidate; replacementPoses = candidatePoses;
                    result.RemovedControlPoints += span - 1; break;
                }
                if (replacement != null) { output.AddRange(replacement); poses.AddRange(replacementPoses); }
                else for (int i = anchors[at] + 1; i <= anchors[selected]; i++) { output.Add(result.States[i]); poses.Add(result.Poses[i]); }
                at = selected;
            }
            result.States.Clear(); result.States.AddRange(output); result.Poses.Clear(); result.Poses.AddRange(poses);
            result.Failure = null;
        }
        public IEnumerator RefineBidirectional(Result result)
        {
            var forward = Refine(result); while (forward.MoveNext()) yield return forward.Current;
            if (world.ShouldStop?.Invoke() == true || !(world.CanRefine?.Invoke() ?? true)) yield break;
            // Work on a private reversed copy so cancellation can never publish a
            // reversed or incomplete trajectory.
            var reverse = new Result { Found = result.Found, Checks = result.Checks, RemovedControlPoints = result.RemovedControlPoints };
            for (int i = result.States.Count - 1; i >= 0; i--) { reverse.States.Add(result.States[i]); reverse.Poses.Add(result.Poses[i]); }
            var backward = Refine(reverse); while (backward.MoveNext()) yield return backward.Current;
            result.Checks = reverse.Checks;
            result.RemovedControlPoints = reverse.RemovedControlPoints;
            result.States.Clear(); result.Poses.Clear();
            for (int i = reverse.States.Count - 1; i >= 0; i--) { result.States.Add(reverse.States[i]); result.Poses.Add(reverse.Poses[i]); }
        }
        // Round genuine joint-space corners, replacing the executable dense states.
        // Every blend edge is swept-checked; a rejected/budget-limited blend leaves
        // the complete original window untouched. Final Cartesian MoveL is separate.
        public IEnumerator Smooth(Result result, float maximumTcpDeviationMillimeters)
        {
            if (!result.Found || result.States.Count < 4 || !LinearTrackAxis.Finite(maximumTcpDeviationMillimeters) || maximumTcpDeviationMillimeters <= 0) yield break;
            int accepted = 0;
            for (int corner = 2; corner < result.States.Count - 2 && accepted < 32; corner++)
            {
                if (world.ShouldStop?.Invoke() == true || !(world.CanRefine?.Invoke() ?? true) || result.Checks >= world.MaximumChecks - 128) yield break;
                if (SameDirection(result.States[corner - 1], result.States[corner], result.States[corner + 1])) continue;
                int first = Math.Max(0, corner - 8), last = Math.Min(result.States.Count - 1, corner + 8);
                var a = result.States[first]; var control = result.States[corner]; var b = result.States[last];
                var states = new List<ArmTrackState>(); var poses = new List<Pose>(); var prior = a; bool valid = true;
                float oldPenalty = 0;
                if (world.Penalty != null) for (int i = first; i <= last; i++) oldPenalty = Mathf.Max(oldPenalty, world.Penalty(result.States[i]));
                for (int sample = 1; sample <= 24 && valid; sample++)
                {
                    float t = sample / 24f;
                    var state = sample == 24 ? b : CoordinatedMoveLPlanner.Interpolate(CoordinatedMoveLPlanner.Interpolate(a, control, t), CoordinatedMoveLPlanner.Interpolate(control, b, t), t);
                    bool edgeValid = false;
                    var edge = Edge(prior, state, result, states, poses, ok => edgeValid = ok, world.CanRefine);
                    try { while (edge.MoveNext()) yield return edge.Current; } finally { (edge as IDisposable)?.Dispose(); }
                    valid = edgeValid; prior = state;
                }
                for (int i = 0; i < states.Count && valid; i++)
                {
                    if (world.Penalty != null && world.Penalty(states[i]) > oldPenalty + .00001f) { valid = false; break; }
                    float distance = float.PositiveInfinity;
                    for (int j = first + 1; j <= last; j++)
                    {
                        var start = result.Poses[j - 1].position; var delta = result.Poses[j].position - start;
                        float t = delta.sqrMagnitude < 1e-12f ? 0 : Mathf.Clamp01(Vector3.Dot(poses[i].position - start, delta) / delta.sqrMagnitude);
                        distance = Mathf.Min(distance, Vector3.Distance(poses[i].position, start + delta * t));
                    }
                    if (distance > maximumTcpDeviationMillimeters * .001f) valid = false;
                }
                if (valid && states.Count > 0 && world.ShouldStop?.Invoke() != true && (world.CanRefine?.Invoke() ?? true))
                {
                    result.States.RemoveRange(first + 1, last - first); result.Poses.RemoveRange(first + 1, last - first);
                    result.States.InsertRange(first + 1, states); result.Poses.InsertRange(first + 1, poses);
                    corner = first + states.Count; accepted++;
                    world.Progress?.Invoke($"Refining transfer: {accepted} validated bounded corner blends.");
                }
            }
        }
        private static bool SameDirection(ArmTrackState a, ArmTrackState b, ArmTrackState c)
        {
            double dot = 0, xLength = 0, yLength = 0;
            for (int i = 0; i < 7; i++)
            {
                double x = i == 6 ? (b.E1Millimeters - a.E1Millimeters) / 2d : b[i] - a[i];
                double y = i == 6 ? (c.E1Millimeters - b.E1Millimeters) / 2d : c[i] - b[i];
                dot += x * y; xLength += x * x; yLength += y * y;
            }
            return xLength < 1e-12 || yLength < 1e-12 || dot > 0 && dot * dot >= .9999 * xLength * yLength;
        }
        private static double Distance(ArmTrackState a, ArmTrackState b)
        {
            double sum = Math.Pow(((double)a.E1Millimeters - b.E1Millimeters) / 2, 2);
            for (int i = 0; i < 6; i++) sum += Math.Pow((double)a[i] - b[i], 2) * (i >= 3 ? 2 : 1);
            return Math.Sqrt(sum);
        }
    }
}
