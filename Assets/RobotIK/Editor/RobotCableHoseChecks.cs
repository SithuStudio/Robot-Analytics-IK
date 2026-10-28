using System;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace RobotIK.Editor
{
    // Runs in an isolated batch editor, never in the user's open scene.
    [InitializeOnLoad]
    public static class RobotCableHoseChecks
    {
        private const string Pending = "RobotIK.CableHoseChecks.Pending";
        static RobotCableHoseChecks()
        {
            if (SessionState.GetBool(Pending, false)) EditorApplication.playModeStateChanged += OnPlayMode;
        }
        public static void RunBatch()
        {
            if (!Application.isBatchMode) throw new InvalidOperationException("Run in a separate Unity batch editor.");
            try
            {
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                CheckPreview();
                SessionState.SetBool(Pending, true);
                EditorApplication.playModeStateChanged -= OnPlayMode;
                EditorApplication.playModeStateChanged += OnPlayMode;
                EditorApplication.EnterPlaymode();
            }
            catch (Exception e) { Fail(e); }
        }
        private static void OnPlayMode(PlayModeStateChange state)
        {
            if (state != PlayModeStateChange.EnteredPlayMode || !SessionState.GetBool(Pending, false)) return;
            SessionState.SetBool(Pending, false);
            EditorApplication.delayCall += () =>
            {
                try { CheckPhysics(); CheckStiffConduit(); Debug.Log("CABLE_HOSE_CHECKS_PASSED"); EditorApplication.Exit(0); }
                catch (Exception e) { Fail(e); }
            };
        }
        private static void Fail(Exception e)
        {
            SessionState.SetBool(Pending, false);
            Debug.LogException(e);
            EditorApplication.Exit(1);
        }
        private static void Require(bool value, string message)
        { if (!value) throw new Exception("Cable hose check failed: " + message); }
        private static GameObject Root(RobotCableHose hose) => (GameObject)typeof(RobotCableHose).GetField("root", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(hose);
        private static RobotCableHose Fixture(out GameObject owner)
        {
            owner = new GameObject("Cable test owner");
            owner.SetActive(false);
            var hose = owner.AddComponent<RobotCableHose>();
            hose.buildOnPlay = false;
            hose.behavior = RobotCableHose.CableBehavior.StiffConduit;
            hose.startAnchor = new GameObject("Test start").transform;
            hose.endAnchor = new GameObject("Test end").transform;
            hose.startAnchor.SetParent(owner.transform);
            hose.endAnchor.SetParent(owner.transform);
            hose.startAnchor.position = new Vector3(-0.6f, 0.3f, 0);
            hose.endAnchor.position = new Vector3(0.6f, 0.3f, 0);
            hose.cableLength = 1.6f;
            hose.targetSegmentLength = 0.08f;
            owner.SetActive(true);
            return hose;
        }
        private static void CheckPreview()
        {
            var hose = Fixture(out var owner);
            Require(hose.Rebuild(), "preview builds");
            Require(Mathf.Abs(hose.GeneratedRestLength - 1.6f) < 0.00001f, "requested rest length");
            Require(Root(hose).transform.parent == null, "world-space generated root");
            Require(Root(hose).GetComponentsInChildren<Rigidbody>().Length == 0, "preview has no physics");
            CheckMesh(hose);
            var clamp = new GameObject("Test clamp").transform;
            clamp.SetParent(owner.transform);
            clamp.position = new Vector3(0, 0.3f, 0);
            hose.clamps = new[] { clamp };
            Require(hose.Rebuild(), "clamp preview builds");
            CheckMesh(hose);
            hose.cableLength = 1.2f;
            Require(hose.Rebuild(), "taut cable builds");
            CheckMesh(hose);
            hose.cableLength = 0.1f;
            Require(!hose.ValidateConfiguration(out _), "short cable rejected");
            hose.cableLength = 10;
            hose.maximumSegments = 4;
            Require(!hose.ValidateConfiguration(out _), "excessive segment count rejected");
            hose.maximumSegments = 128;
            hose.clamps = new Transform[] { null };
            Require(!hose.ValidateConfiguration(out _), "null clamp rejected");
            hose.ClearGenerated();
            Require(!hose.IsGenerated && hose.SegmentCount == 0, "preview cleanup");
            UnityEngine.Object.DestroyImmediate(owner);
            Debug.Log("Cable hose editor checks passed: slack, taut, clamps, mesh, invalid inputs, cleanup.");
        }
        private static void CheckMesh(RobotCableHose hose)
        {
            var mesh = Root(hose).GetComponent<MeshFilter>().sharedMesh;
            var v = mesh.vertices;
            var normals = mesh.normals;
            var indices = mesh.triangles;
            Require(v.Length > 0 && v.Length == normals.Length, "mesh buffers");
            foreach (Vector3 point in v) Require(!float.IsNaN(point.sqrMagnitude) && !float.IsInfinity(point.sqrMagnitude), "finite vertices");
            float length = 0;
            int stride = hose.radialSides + 1;
            int spanCount = (hose.clamps == null ? 0 : hose.clamps.Length) + 1;
            // UV distance is measured from the physical rest lengths, including duplicate rings at clamps.
            var uv = mesh.uv;
            Require(Mathf.Abs(uv[uv.Length - 1].y * hose.textureRepeatMeters - hose.cableLength) < 0.0001f, "UV arc length");
            var joints = Root(hose).GetComponentsInChildren<Transform>();
            foreach (var segment in joints)
            {
                if (!segment.name.StartsWith("Segment ")) continue;
                // Consecutive ring centers recover each polygon's actual centerline length below.
                length += 1;
            }
            Require((int)length == hose.SegmentCount, "segment count");
            Require(v.Length / stride == hose.SegmentCount + spanCount, "clamp ring layout");
            for (int i = 0; i < indices.Length; i += 3)
            {
                Vector3 face = Vector3.Cross(v[indices[i + 1]] - v[indices[i]], v[indices[i + 2]] - v[indices[i]]);
                Require(Vector3.Dot(face, normals[indices[i]] + normals[indices[i + 1]] + normals[indices[i + 2]]) > 0, "outward triangle winding");
            }
            // Recover centerline rings from their opposite sides (radialSides is even in the fixture).
            int ringOffset = 0;
            float measured = 0;
            Transform previous = hose.startAnchor;
            for (int span = 0; span < spanCount; span++)
            {
                Transform next = span == spanCount - 1 ? hose.endAnchor : hose.clamps[span];
                float totalChord = 0;
                Transform routePrevious = hose.startAnchor;
                if (hose.clamps != null) foreach (Transform routeClamp in hose.clamps)
                {
                    totalChord += Vector3.Distance(routePrevious.position, routeClamp.position);
                    routePrevious = routeClamp;
                }
                totalChord += Vector3.Distance(routePrevious.position, hose.endAnchor.position);
                int count = Mathf.Max(3, Mathf.CeilToInt(hose.cableLength * Vector3.Distance(previous.position, next.position) / totalChord / hose.targetSegmentLength));
                for (int j = 0; j < count; j++)
                {
                    int a = (ringOffset + j) * stride, b = a + stride;
                    Vector3 p = (v[a] + v[a + hose.radialSides / 2]) * 0.5f;
                    Vector3 q = (v[b] + v[b + hose.radialSides / 2]) * 0.5f;
                    measured += Vector3.Distance(p, q);
                }
                ringOffset += count + 1;
                previous = next;
            }
            Require(Mathf.Abs(measured - hose.cableLength) < 0.0001f, "actual generated centerline length");
        }
        private static void Tick(RobotCableHose hose, int count)
        {
            for (int i = 0; i < count; i++)
            {
                hose.SendMessage("FixedUpdate");
                Physics.SyncTransforms();
                Physics.Simulate(0.01f);
            }
            hose.SendMessage("LateUpdate");
        }
        private static void CheckPhysics()
        {
            Physics.simulationMode = SimulationMode.Script;
            var hose = Fixture(out var owner);
            hose.behavior = RobotCableHose.CableBehavior.FlexibleHose;
            hose.angularStiffness = 0;
            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Cable test floor";
            floor.transform.position = new Vector3(0, -0.1f, 0);
            floor.transform.localScale = new Vector3(10, 0.2f, 10);
            Require(hose.Rebuild(), "runtime builds");
            var root = Root(hose);
            var bodies = root.GetComponentsInChildren<Rigidbody>();
            var colliders = root.GetComponentsInChildren<CapsuleCollider>();
            Require(bodies.Length == hose.SegmentCount + 2, "link and proxy bodies");
            Require(colliders.Length == hose.SegmentCount, "capsule count");
            Require(Physics.GetIgnoreCollision(colliders[0], colliders[colliders.Length - 1]), "self collision off");
            Tick(hose, 600);
            float minimumY = float.PositiveInfinity;
            foreach (var collider in colliders) minimumY = Mathf.Min(minimumY, collider.bounds.min.y);
            Require(minimumY > -0.04f && minimumY < 0.04f, "settles on floor without tunneling: " + minimumY);
            CheckJointGaps(root);
            hose.endAnchor.position += Vector3.up * 0.1f;
            Tick(hose, 200);
            CheckJointGaps(root);
            Require(Mathf.Abs(hose.GeneratedRestLength - 1.6f) < 0.00001f, "length unchanged by moving anchors");
            var clamp = new GameObject("Runtime clamp").transform;
            clamp.SetParent(owner.transform);
            clamp.position = new Vector3(0, 0.5f, 0);
            hose.clamps = new[] { clamp };
            hose.cableLength = 1.8f;
            Require(hose.Rebuild(), "runtime clamp rebuild");
            Require(!root.activeSelf, "old physics deactivated immediately on rebuild");
            Tick(hose, 300);
            CheckJointGaps(Root(hose));
            UnityEngine.Object.DestroyImmediate(clamp.gameObject);
            hose.SendMessage("FixedUpdate");
            Require(!hose.IsGenerated, "destroyed anchor clears simulation");
            UnityEngine.Object.DestroyImmediate(owner);
            UnityEngine.Object.DestroyImmediate(floor);
            Debug.Log("Cable hose play checks passed: floor collision, endpoint movement, joint gaps, fixed length, clamps, rebuild, anchor deletion.");
        }
        private static float SettleDisplacement(RobotCableHose hose)
        {
            Require(hose.Rebuild(), "material fixture builds");
            var capsules = Root(hose).GetComponentsInChildren<CapsuleCollider>();
            var positions = new Vector3[capsules.Length];
            for (int i = 0; i < capsules.Length; i++) positions[i] = capsules[i].attachedRigidbody.position;
            Tick(hose, 500);
            float total = 0;
            for (int i = 0; i < capsules.Length; i++) total += Vector3.Distance(positions[i], capsules[i].attachedRigidbody.position);
            return total / capsules.Length;
        }
        private static void CheckStiffConduit()
        {
            var hose = Fixture(out var owner);
            hose.startAnchor.position += Vector3.up * 3;
            hose.endAnchor.position += Vector3.up * 3;
            hose.behavior = RobotCableHose.CableBehavior.FlexibleHose;
            float flexibleSag = SettleDisplacement(hose);
            hose.ApplyStiffConduitMaterial();
            float stiffSag = SettleDisplacement(hose);
            Require(stiffSag < flexibleSag * 0.7f, $"stiff conduit retains arch better: stiff={stiffSag}, flexible={flexibleSag}");
            var root = Root(hose);
            var capsules = root.GetComponentsInChildren<CapsuleCollider>();
            var middle = capsules[capsules.Length / 2].attachedRigidbody;
            Vector3 before = middle.position;
            middle.AddForce(Vector3.forward * 0.1f, ForceMode.Impulse);
            float peak = 0;
            for (int i = 0; i < 100; i++)
            {
                Tick(hose, 1);
                peak = Mathf.Max(peak, Mathf.Abs(middle.position.z - before.z));
            }
            Require(peak > 0.00001f, "stiff conduit still deforms under external load");
            CheckJointGaps(root);
            hose.startAnchor.rotation = Quaternion.Euler(0, 20, 0) * hose.startAnchor.rotation;
            Tick(hose, 200);
            CheckJointGaps(root);
            Require(capsules[0].attachedRigidbody.solverIterations >= 32, "stiff material uses increased solver iterations");
            UnityEngine.Object.DestroyImmediate(owner);
            Debug.Log("Cable hose stiff conduit checks passed: reduced sag, elastic deformation, moving mounts, joint gaps.");
        }
        private static void CheckJointGaps(GameObject root)
        {
            foreach (var joint in root.GetComponentsInChildren<ConfigurableJoint>())
            {
                Rigidbody body = joint.GetComponent<Rigidbody>();
                Vector3 a = body.position + body.rotation * joint.anchor;
                Vector3 b = joint.connectedBody.position + joint.connectedBody.rotation * joint.connectedAnchor;
                Require(Vector3.Distance(a, b) < 0.04f, "joint remains attached: " + Vector3.Distance(a, b));
            }
        }
    }
}

