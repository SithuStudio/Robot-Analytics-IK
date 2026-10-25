using System;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace RobotIK.Editor
{
    /// <summary>Scene inspection and explicit setup commands; never changes a scene on import.</summary>
    public static class LinearTrackSceneSetup
    {
        private static string PathOf(Transform t) => t == null ? "(none)" : (t.parent == null ? t.name : PathOf(t.parent) + "/" + t.name);
        private static T[] All<T>() where T : Component => SceneManager.GetActiveScene().GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<T>(true)).ToArray();

        [MenuItem("Robot IK/Track/Set up sample locked track in active scene")]
        public static void SetupFromMenu()
        {
            try { Debug.Log(Setup()); }
            catch (Exception e) { Debug.LogException(e); }
        }

        private static string Setup()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play mode before scene setup.");
            var scene = SceneManager.GetActiveScene();
            var planners = All<RobotBaseSixAxisMotionPlanner>();
            if (planners.Length != 1) throw new InvalidOperationException("Scene setup requires exactly one six-axis planner.");
            var planner = planners[0]; var robot = planner.transform;
            var arm = robot.GetComponent<RobotBaseSixAxisRig>();
            if (arm == null || !arm.IsReady) throw new InvalidOperationException("The existing arm joint bindings must be valid.");
            if (robot.GetComponent<RobotLinearTrackRig>() != null)
                throw new InvalidOperationException("Robot Base already has a track adapter. Edit its current profile instead of overwriting it.");
            Transform FindUnique(string name) => All<Transform>().SingleOrDefault(t => t.name == name);
            var root = FindUnique("TrackRoot") ?? FindUnique("Track Root");
            var carriage = FindUnique("Carriage");
            if (root == null || carriage == null || carriage.parent != root)
                throw new InvalidOperationException("Expected the existing TrackRoot / Carriage hierarchy. No geometry was changed.");
            if (Vector3.Distance(root.lossyScale, Vector3.one) > .00001f)
                throw new InvalidOperationException("TrackRoot must have unit scale before setup.");

            // Capture the live editor state, including unsaved user edits, before any mutation.
            string backupFolder = "Assets/Scenes/TrackSetupBackups";
            Directory.CreateDirectory(backupFolder);
            string backup = backupFolder + "/" + scene.name + " before track setup " + DateTime.Now.ToString("yyyyMMdd-HHmmss") + ".unity";
            if (!EditorSceneManager.SaveScene(scene, backup, true)) throw new IOException("Could not save the pre-setup scene copy.");
            var unchanged = All<Transform>().ToDictionary(t => t, t => t.localToWorldMatrix);
            var robotPosition = robot.position; var robotRotation = robot.rotation;
            var jointPositions = arm.Joints.Select(j => j.localPosition).ToArray();
            var jointRotations = arm.Joints.Select(j => j.localRotation).ToArray();
            var start = planner.StartPose; var end = planner.EndTarget;
            var startMatrix = start != null ? start.localToWorldMatrix : Matrix4x4.identity;
            var endMatrix = end != null ? end.localToWorldMatrix : Matrix4x4.identity;
            Undo.IncrementCurrentGroup(); int undo = Undo.GetCurrentGroup(); Undo.SetCurrentGroupName("Set up calibrated locked E1");
            try
            {
                // The mistakenly attached adapter also required a duplicate arm rig on TrackRoot.
                var misplaced = root.GetComponent<RobotLinearTrackRig>();
                if (misplaced != null) Undo.DestroyObjectImmediate(misplaced);
                var duplicate = root.GetComponent<RobotBaseSixAxisRig>();
                if (duplicate != null && duplicate.Joints.SequenceEqual(arm.Joints)) Undo.DestroyObjectImmediate(duplicate);
                Undo.RecordObject(root.gameObject, "Name TrackRoot"); root.name = "TrackRoot";
                Undo.RecordObject(root.gameObject, "Enable TrackRoot"); root.gameObject.SetActive(true);

                // Put visual scaling on the geometry, never on the kinematic carriage.
                var children = Enumerable.Range(0, carriage.childCount).Select(carriage.GetChild).ToArray();
                foreach (var child in children) Undo.SetTransformParent(child, root, "Preserve carriage child world geometry");
                Undo.RecordObject(carriage, "Normalize carriage scale"); carriage.localScale = Vector3.one;
                foreach (var child in children) Undo.SetTransformParent(child, carriage, "Restore carriage geometry");
                var mount = FindUnique("RobotMount");
                if (mount == null) { mount = new GameObject("RobotMount").transform; Undo.RegisterCreatedObjectUndo(mount.gameObject, "Create RobotMount"); }
                Undo.SetTransformParent(mount, carriage, "Mount outside scaled geometry");
                Undo.RecordObject(mount, "Preserve current robot installation");
                mount.localScale = Vector3.one; mount.SetPositionAndRotation(robotPosition, robotRotation);
                Undo.SetTransformParent(robot, mount, "Mount existing Robot Base on carriage");
                if (!robot.gameObject.activeSelf) throw new InvalidOperationException("The existing Robot Base must be active.");

                var track = Undo.AddComponent<RobotLinearTrackRig>(robot.gameObject);
                Vector3 home = carriage.localPosition * 1000f; Vector3 rotation = carriage.localEulerAngles;
                var profile = new TrackPoseProfile
                {
                    Home = new TrackPoseSample("Home · sample", 0, home, rotation),
                    Intermediate = new[] { new TrackPoseSample("Mid · SAMPLE calibration", 2000,
                        home + new Vector3(2, 1, 2000), (Quaternion.Euler(.1f, .15f, -.05f) * carriage.localRotation).eulerAngles) },
                    End = new TrackPoseSample("End · sample", 4000, home + new Vector3(0, 0, 4000), rotation),
                    Corrections = TrackCorrectionChannels.PositionOnly
                };
                if (!track.TryConfigure(root, carriage, profile, out string error)) throw new InvalidOperationException(error);
                var movingBodies = carriage.GetComponentsInChildren<Collider>(true).Where(c => !c.transform.IsChildOf(robot)).ToArray();
                var rail = root.Find("Stationary rail");
                var railBodies = rail != null ? rail.GetComponentsInChildren<Collider>(true) : Array.Empty<Collider>();
                var fixedBaseBodies = robot.GetComponentsInChildren<Collider>(true)
                    .Where(c => !arm.Joints.Any(j => c.transform.IsChildOf(j))).ToArray();
                var contacts = new System.Collections.Generic.List<TrackAllowedContact>();
                Physics.SyncTransforms();
                foreach (var body in movingBodies)
                    foreach (var other in railBodies.Concat(fixedBaseBodies))
                    {
                        var bounds = body.bounds; bounds.Expand(.1f);
                        if (bounds.Intersects(other.bounds)) contacts.Add(new TrackAllowedContact { First = body, Second = other });
                    }
                var so = new SerializedObject(track);
                AssignArray(so.FindProperty("carriageColliders"), movingBodies);
                AssignArray(so.FindProperty("stationaryRailColliders"), railBodies);
                var allowed = so.FindProperty("allowedContacts"); allowed.arraySize = contacts.Count;
                for (int i = 0; i < contacts.Count; i++)
                {
                    allowed.GetArrayElementAtIndex(i).FindPropertyRelative("First").objectReferenceValue = contacts[i].First;
                    allowed.GetArrayElementAtIndex(i).FindPropertyRelative("Second").objectReferenceValue = contacts[i].Second;
                }
                so.ApplyModifiedProperties();
                var plannerSo = new SerializedObject(planner); plannerSo.FindProperty("linearTrack").objectReferenceValue = track; plannerSo.ApplyModifiedProperties();
                if (!track.TrySetEnabled(true, out error) || !track.ValidateConfiguration(out error)) throw new InvalidOperationException(error);

                // Visible hierarchy references for inspecting sample station poses, outside Carriage.
                var stations = new GameObject("Calibration samples (SAMPLE data)").transform;
                Undo.RegisterCreatedObjectUndo(stations.gameObject, "Create calibration station references");
                Undo.SetTransformParent(stations, root, "Parent calibration references");
                stations.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity); stations.localScale = Vector3.one;
                foreach (var sample in new[] { profile.Home }.Concat(profile.Intermediate).Concat(new[] { profile.End }))
                {
                    var marker = new GameObject(sample.Name + " · E1 " + sample.E1Millimeters + " mm").transform;
                    Undo.RegisterCreatedObjectUndo(marker.gameObject, "Create sample calibration pose");
                    Undo.SetTransformParent(marker, stations, "Parent sample calibration pose");
                    marker.SetLocalPositionAndRotation(sample.PositionMillimeters * .001f, Quaternion.Euler(sample.RotationDegrees));
                    marker.localScale = Vector3.one;
                }
                // Geometry, targets and every arm joint must remain where the user left them.
                foreach (var pair in unchanged)
                    if (pair.Key != null && pair.Key != carriage && pair.Key != mount && !SameMatrix(pair.Value, pair.Key.localToWorldMatrix))
                        throw new InvalidOperationException("Setup changed the world pose or scale of " + PathOf(pair.Key));
                for (int i = 0; i < arm.Joints.Length; i++)
                    if (arm.Joints[i].localPosition != jointPositions[i] || Quaternion.Angle(arm.Joints[i].localRotation, jointRotations[i]) > .01f)
                        throw new InvalidOperationException("Setup changed arm joint J" + (i + 1));
                if ((start != null && !SameMatrix(startMatrix, start.localToWorldMatrix)) || (end != null && !SameMatrix(endMatrix, end.localToWorldMatrix)))
                    throw new InvalidOperationException("TCP endpoint targets moved.");
                foreach (var obj in new UnityEngine.Object[] { track, planner, robot, mount, carriage })
                { EditorUtility.SetDirty(obj); if (PrefabUtility.IsPartOfPrefabInstance(obj)) PrefabUtility.RecordPrefabInstancePropertyModifications(obj); }
                EditorSceneManager.MarkSceneDirty(scene);
                if (!EditorSceneManager.SaveScene(scene)) throw new IOException("Scene setup passed validation but could not save.");
                Undo.CollapseUndoOperations(undo); Selection.activeGameObject = robot.gameObject; SceneView.RepaintAll();
                return "SUCCESS: scene saved; world geometry and six joint local poses preserved.\nBackup: " + backup + "\n" +
                    "Sample E1: Home 0 mm, Mid 2000 mm (+2 mm X, +1 mm Y; optional rotations), End 4000 mm. Selected Home.\n" +
                    "Carriage bodies: " + movingBodies.Length + "; rail bodies: " + railBodies.Length + "; exact mounting contact pairs: " + contacts.Count + "\n" +
                    string.Join("\n", contacts.Select(p => "Mount contact: " + PathOf(p.First.transform) + " / " + PathOf(p.Second.transform))) + "\n" + Inspect();
            }
            catch { Undo.RevertAllDownToGroup(undo); throw; }
        }

        private static void AssignArray(SerializedProperty property, Collider[] colliders)
        { property.arraySize = colliders.Length; for (int i = 0; i < colliders.Length; i++) property.GetArrayElementAtIndex(i).objectReferenceValue = colliders[i]; }
        private static bool SameMatrix(Matrix4x4 a, Matrix4x4 b)
        { for (int i = 0; i < 16; i++) if (Mathf.Abs(a[i] - b[i]) > .0001f) return false; return true; }

        [MenuItem("Robot IK/Track/Verify locked track setup")]
        public static void VerifyFromMenu()
        { try { Debug.Log(Verify()); } catch (Exception e) { Debug.LogException(e); } }

        private static string Verify()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play mode before setup verification.");
            var planner = All<RobotBaseSixAxisMotionPlanner>().Single();
            var track = planner.LinearTrack;
            if (track == null || !track.TrackEnabled || !track.ValidateConfiguration(out _)) throw new InvalidOperationException("The track configuration is invalid.");
            var arm = planner.GetComponent<RobotBaseSixAxisRig>();
            var ik = planner.GetComponent<RobotBaseSixAxisAnalyticIK>();
            if (ik == null) throw new InvalidOperationException("Existing analytical IK component is missing.");
            var originalAngles = arm.GetJointAnglesDegrees();
            var originalPositions = arm.Joints.Select(j => j.localPosition).ToArray();
            var originalRotations = arm.Joints.Select(j => j.localRotation).ToArray();
            float selected = track.SelectedE1Millimeters;
            var start = planner.StartPose.localToWorldMatrix; var end = planner.EndTarget.localToWorldMatrix;
            var armSettings = EditorJsonUtility.ToJson(arm); var ikSettings = EditorJsonUtility.ToJson(ik);
            var s = new StringBuilder(); var owner = new object();
            try
            {
                if (!ik.CalibrateFromRig()) throw new InvalidOperationException(ik.CalibrationStatus);
                foreach (float e1 in new[] { track.MinimumMillimeters, 500f, 2000f, track.MaximumMillimeters })
                {
                    if (!track.TrySetPosition(e1, out string error)) throw new InvalidOperationException(error);
                    // Use a non-singular, limit-respecting posture to test the existing solver
                    // at each translated base location, without changing the robot model.
                    if (!arm.SetJointAngles(new[] { 10f, 20f, 30f, 40f, 50f, 60f })) throw new InvalidOperationException("Verification posture violates configured limits.");
                    Transform tcp = arm.ToolFrame != null ? arm.ToolFrame : arm.Joints[5];
                    Vector3 targetPosition = tcp.position; Quaternion targetRotation = tcp.rotation;
                    if (!ik.SolveAll(targetPosition, targetRotation, out var solutions) || solutions.Count == 0)
                        throw new InvalidOperationException("Existing arm IK failed at E1 " + e1);
                    if (!SameMatrix(start, planner.StartPose.localToWorldMatrix) || !SameMatrix(end, planner.EndTarget.localToWorldMatrix))
                        throw new InvalidOperationException("Track preview moved world-fixed TCP targets.");
                    s.AppendLine($"PASS E1 {e1:F0} mm: six-axis IK returned {solutions.Count} solution(s); targets fixed.");
                }
                if (!track.TryAcquire(owner, out var stamp, out string lockError)) throw new InvalidOperationException(lockError);
                if (track.TrySetPosition(selected, out _) || track.TrySetProfile(track.Profile, out _) || track.TrySetEnabled(false, out _))
                    throw new InvalidOperationException("Track lock allowed an edit.");
                if (!stamp.Matches(track, out _)) throw new InvalidOperationException("Track lock stamp was unstable.");
                s.AppendLine("PASS locked E1 rejects position, calibration and enable edits.");
            }
            finally
            {
                track.Release(owner);
                track.TrySetPosition(selected, out _);
                arm.SetJointAngles(originalAngles);
                for (int i = 0; i < arm.Joints.Length; i++)
                { arm.Joints[i].localPosition = originalPositions[i]; arm.Joints[i].localRotation = originalRotations[i]; }
                Physics.SyncTransforms();
            }
            if (armSettings != EditorJsonUtility.ToJson(arm) || ikSettings != EditorJsonUtility.ToJson(ik))
                throw new InvalidOperationException("Verification changed serialized six-axis settings.");
            s.AppendLine("PASS restored selected E1 and all six original joint poses; arm and IK serialized settings unchanged.");
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
            if (!EditorSceneManager.SaveScene(SceneManager.GetActiveScene())) throw new IOException("Could not save the restored Home pose.");
            return s + Inspect();
        }
        private static string Inspect()
        {
            var scene = SceneManager.GetActiveScene();
            var s = new StringBuilder($"Scene: {scene.path}\nDirty: {scene.isDirty}\nPlay mode: {EditorApplication.isPlaying}\n");
            foreach (var p in All<RobotBaseSixAxisMotionPlanner>())
                s.AppendLine($"PLANNER {PathOf(p.transform)} active={p.gameObject.activeInHierarchy} Start={PathOf(p.StartPose)} End={PathOf(p.EndTarget)} Obstacles={PathOf(p.ObstaclesRoot)}");
            foreach (var r in All<RobotBaseSixAxisRig>())
                s.AppendLine($"RIG {PathOf(r.transform)} active={r.gameObject.activeInHierarchy} scale={r.transform.lossyScale:F5} joints={string.Join(",", r.Joints.Select(PathOf))}");
            foreach (var t in All<Transform>().Where(t => t.name.Contains("Track") || t.name.Contains("Carriage") || t.name.Contains("RobotMount") || t.name.Contains("Stationary") || t.GetComponent<RobotBaseSixAxisMotionPlanner>() != null))
                s.AppendLine($"TRANSFORM {PathOf(t)} activeSelf={t.gameObject.activeSelf} position={t.position:F5} rotation={t.eulerAngles:F5} localScale={t.localScale:F5} worldScale={t.lossyScale:F5}");
            foreach (var t in All<RobotLinearTrackRig>())
                s.AppendLine($"TRACK {PathOf(t.transform)} enabled={t.TrackEnabled} E1={t.SelectedE1Millimeters} valid={t.ValidateConfiguration(out string error)} error={error}");
            return s.ToString();
        }
    }
}
