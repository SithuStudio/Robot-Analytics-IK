using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace RobotIK.Editor
{
    [CustomEditor(typeof(RobotCableHose))]
    public sealed class RobotCableHoseEditor : UnityEditor.Editor
    {
        private bool changedSinceBuild;
        public override void OnInspectorGUI()
        {
            var hose = (RobotCableHose)target;
            EditorGUILayout.HelpBox("Assign start/end transforms and optional clamps in cable order. Length is fixed in world meters. Elastic Rod simulates the rest route and material twist; Stiff Conduit and Flexible Hose retain the older joint-chain modes. Rebuild after changing settings or the initial pose.", MessageType.Info);
            EditorGUI.BeginChangeCheck();
            if (hose.behavior == RobotCableHose.CableBehavior.ElasticRod)
            {
                serializedObject.Update();
                DrawPropertiesExcluding(serializedObject, "m_Script", "angularStiffness", "angularDamping", "bendLimitDegrees",
                    "twistLimitDegrees", "elasticDampingRatio", "elasticBendLimitDegrees", "maximumElasticTorque", "mountBendLimitDegrees",
                    "solverIterations", "physicsMaterial");
                serializedObject.ApplyModifiedProperties();
            }
            else DrawDefaultInspector();
            if (EditorGUI.EndChangeCheck()) changedSinceBuild = true;
            if (GUILayout.Button("Apply Elastic Rod Material"))
            {
                Undo.RecordObject(hose, "Set elastic rod material");
                hose.ApplyElasticRodMaterial();
                EditorUtility.SetDirty(hose);
                changedSinceBuild = true;
            }
            if (hose.behavior == RobotCableHose.CableBehavior.ElasticRod)
            {
                EditorGUILayout.HelpBox("Route handles define the REST curve at rebuild; clamps are moving attachments. Held mounts capture the initial cable direction and material orientation. Bend Rigidity controls shape retention; Twist Rigidity controls torsion. Stripes follow the physical cross sections.", MessageType.Info);
                using (new EditorGUI.DisabledScope(Application.isPlaying || !HasAttachments(hose)))
                {
                    if (GUILayout.Button("Create / Fill Route Handles"))
                    { CreateRouteHandles(hose); changedSinceBuild = true; }
                    if (GUILayout.Button("Fit Length to Editable Route"))
                    {
                        try
                        {
                            Undo.RecordObject(hose, "Fit cable rest length to route");
                            hose.cableLength = hose.MeasureRodRouteLength() + 0.00001f;
                            EditorUtility.SetDirty(hose); changedSinceBuild = true;
                        }
                        catch (ArgumentException e) { Debug.LogWarning(e.Message, hose); }
                    }
                }
            }
            if (GUILayout.Button("Apply Stiff Conduit Material (legacy joints)"))
            {
                Undo.RecordObject(hose, "Set stiff conduit material");
                hose.ApplyStiffConduitMaterial();
                EditorUtility.SetDirty(hose);
                changedSinceBuild = true;
            }
            if (hose.behavior == RobotCableHose.CableBehavior.StiffConduit)
                EditorGUILayout.HelpBox("Bend Rigidity controls hardness (try 2, then 5 or 10). The generated arch is the rest shape. Held mounts capture that initial direction; no anchor-axis alignment is required. Rebuild to apply changes.", MessageType.Info);
            bool valid = hose.ValidateConfiguration(out string error);
            if (!valid) EditorGUILayout.HelpBox(error, MessageType.Error);
            if (hose.IsGenerated && changedSinceBuild)
                EditorGUILayout.HelpBox("Settings or attachments changed. Rebuild to apply them. Edit-mode preview does not simulate or follow moving anchors.", MessageType.Warning);
            using (new EditorGUI.DisabledScope(!valid))
                if (GUILayout.Button(Application.isPlaying ? "Rebuild / Reset Cable" : "Generate / Rebuild Preview"))
                {
                    hose.Rebuild();
                    changedSinceBuild = false;
                    SceneView.RepaintAll();
                }
            using (new EditorGUI.DisabledScope(!hose.IsGenerated))
                if (GUILayout.Button("Clear Generated Cable")) hose.ClearGenerated();
            if (hose.IsGenerated)
            {
                EditorGUILayout.LabelField(hose.IsRodGenerated ? "Rod elements" : "Physics segments", hose.SegmentCount.ToString());
                if (hose.IsRodGenerated && Application.isPlaying && hose.RodMaximumRelativeStretch > 0.05f)
                    EditorGUILayout.HelpBox("Rod constraints are under strain. Increase substeps/iterations, add slack, or remove conflicting contacts at mounts.", MessageType.Warning);
                EditorGUILayout.LabelField("Rest length", hose.GeneratedRestLength.ToString("F3") + " m");
                if (hose.MaximumSpanExtension > 0.001f)
                    EditorGUILayout.HelpBox($"Attachments exceed a fixed span length by {hose.MaximumSpanExtension:F3} m. Add slack in the starting pose or reduce robot travel. Physics joints can stretch under an impossible constraint.", MessageType.Warning);
            }
            EditorGUILayout.HelpBox("Ignore mounting collars using Ignored Colliders. Collision Layers is combined with Unity's layer collision matrix. Assign an HDRP material for builds. Cable contacts do not affect robot motion planning.", MessageType.None);
        }

        [MenuItem("GameObject/Robot IK/Cable Hose", false, 10)]
        private static void Create(MenuCommand command)
        {
            var owner = new GameObject("Robot Cable Hose");
            Undo.RegisterCreatedObjectUndo(owner, "Create robot cable hose");
            GameObjectUtility.SetParentAndAlign(owner, command.context as GameObject);
            var hose = owner.AddComponent<RobotCableHose>();
            hose.startAnchor = CreateAnchor(owner.transform, "Cable Start", Vector3.zero);
            hose.endAnchor = CreateAnchor(owner.transform, "Cable End", Vector3.right);
            hose.cableLength = 1.4f;
            CreateRouteHandles(hose);
            Selection.activeGameObject = owner;
        }
        [MenuItem("Tools/Robot IK/Use Elastic Rod on Selected Cable")]
        private static void UpgradeSelectedCable()
        {
            var hose=Selection.activeGameObject.GetComponent<RobotCableHose>();
            Undo.RecordObject(hose,"Use elastic rod cable");
            hose.ApplyElasticRodMaterial();CreateRouteHandles(hose);EditorUtility.SetDirty(hose);
            if(hose.ValidateConfiguration(out string error)) hose.Rebuild();
            else Debug.LogWarning("Elastic rod configured. " + error,hose);
        }
        [MenuItem("Tools/Robot IK/Use Elastic Rod on Selected Cable",true)]
        private static bool CanUpgradeSelectedCable() => !Application.isPlaying && Selection.activeGameObject &&
            Selection.activeGameObject.GetComponent<RobotCableHose>();
        private static bool HasAttachments(RobotCableHose hose)
        {
            if (!hose.startAnchor || !hose.endAnchor) return false;
            if (hose.clamps != null) foreach (Transform clamp in hose.clamps) if (!clamp) return false;
            return true;
        }
        private static void CreateRouteHandles(RobotCableHose hose)
        {
            if (!HasAttachments(hose)) return;
            Undo.RecordObject(hose, "Create cable route handles");
            var anchors = new List<Transform> { hose.startAnchor };
            if (hose.clamps != null) anchors.AddRange(hose.clamps);
            anchors.Add(hose.endAnchor);
            var routes = new RobotCableHose.RodRouteSpan[anchors.Count-1];
            float chord = 0;
            for (int i=1;i<anchors.Count;i++) chord += Vector3.Distance(anchors[i-1].position,anchors[i].position);
            Transform holder = null;
            for (int i=0;i<routes.Length;i++)
            {
                if (hose.rodRoutes != null && i<hose.rodRoutes.Length && hose.rodRoutes[i] != null &&
                    hose.rodRoutes[i].points != null && hose.rodRoutes[i].points.Length>0)
                { routes[i]=hose.rodRoutes[i]; continue; }
                if (!holder)
                {
                    var routeObject = new GameObject("Cable Rest Route");
                    Undo.RegisterCreatedObjectUndo(routeObject,"Create cable rest route");
                    routeObject.transform.SetParent(hose.transform,false);
                    holder=routeObject.transform;
                }
                float distance=Vector3.Distance(anchors[i].position,anchors[i+1].position);
                if(distance<1e-5f) continue;
                float length=Mathf.Max(distance,hose.cableLength*distance/Mathf.Max(1e-5f,chord));
                // Route handles sample the current automatic arch but are not runtime pins.
                Vector3[] arch=AutomaticArch(anchors[i].position,anchors[i+1].position,length,hose.initialBowDirection);
                var points=new Transform[2];
                for(int j=0;j<2;j++)
                {
                    var point=new GameObject($"Span {i+1} Route {j+1}");
                    Undo.RegisterCreatedObjectUndo(point,"Create cable route point");
                    point.transform.SetParent(holder,false);point.transform.position=arch[j];points[j]=point.transform;
                }
                routes[i]=new RobotCableHose.RodRouteSpan{points=points};
            }
            hose.rodRoutes=routes;
            EditorUtility.SetDirty(hose);SceneView.RepaintAll();
        }
        private static Vector3[] AutomaticArch(Vector3 a,Vector3 b,float length,Vector3 bow)
        {
            // A sampled arc is accessed through the managed route builder; the rest route may then be edited freely.
            Vector3[] path=ElasticCableRouting.Build(new[]{a,b},length,12,bow);
            return new[]{path[4],path[8]};
        }
        private void OnSceneGUI()
        {
            var hose=(RobotCableHose)target;
            if(hose.behavior!=RobotCableHose.CableBehavior.ElasticRod || hose.rodRoutes==null || Application.isPlaying) return;
            var visited=new HashSet<Transform>();
            for(int i=0;i<hose.rodRoutes.Length;i++)
            {
                var route=hose.rodRoutes[i]; if(route==null || route.points==null) continue;
                for(int j=0;j<route.points.Length;j++)
                {
                    Transform point=route.points[j];if(!point || !visited.Add(point)) continue;
                    Handles.Label(point.position,$"Route {i+1}.{j+1}");
                    EditorGUI.BeginChangeCheck();
                    Vector3 position=Handles.PositionHandle(point.position,Quaternion.identity);
                    if(!EditorGUI.EndChangeCheck()) continue;
                    Undo.RecordObject(point,"Move cable rest route");point.position=position;
                    EditorUtility.SetDirty(point);changedSinceBuild=true;
                    if(hose.IsGenerated && hose.ValidateConfiguration(out _)) hose.Rebuild();
                    Repaint();
                }
            }
        }
        public override bool RequiresConstantRepaint() => Application.isPlaying;
        private static Transform CreateAnchor(Transform parent, string label, Vector3 position)
        {
            var anchor = new GameObject(label);
            Undo.RegisterCreatedObjectUndo(anchor, "Create cable anchor");
            anchor.transform.SetParent(parent, false);
            anchor.transform.localPosition = position;
            return anchor.transform;
        }
    }
}
