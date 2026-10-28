using UnityEditor;
using UnityEngine;

namespace RobotIK.Editor
{
    [CustomEditor(typeof(RobotHose))]
    public sealed class RobotHoseEditor : UnityEditor.Editor
    {
        private double nextRefresh;
        public override bool RequiresConstantRepaint() => false;
        private void OnEnable() => EditorApplication.update+=RefreshStatus;
        private void OnDisable() => EditorApplication.update-=RefreshStatus;
        private void RefreshStatus()
        {
            if(!Application.isPlaying || EditorApplication.timeSinceStartup<nextRefresh) return;
            nextRefresh=EditorApplication.timeSinceStartup+0.2;
            Repaint();
        }
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            var hose=(RobotHose)target;
            serializedObject.Update();
            var points=serializedObject.FindProperty("midPoints");
            var modes=serializedObject.FindProperty("midpointModes");
            // New entries always default to Fixed Clamp; existing transform references stay intact.
            if(modes.arraySize!=points.arraySize)
            {
                int previous=modes.arraySize;
                modes.arraySize=points.arraySize;
                for(int i=previous;i<modes.arraySize;i++) modes.GetArrayElementAtIndex(i).enumValueIndex=0;
            }
            EditorGUILayout.LabelField("Midpoint Attachment Modes",EditorStyles.boldLabel);
            for(int i=0;i<points.arraySize;i++)
            {
                var point=points.GetArrayElementAtIndex(i).objectReferenceValue;
                EditorGUILayout.PropertyField(modes.GetArrayElementAtIndex(i),new GUIContent(i+": "+(point?point.name:"Unassigned")));
            }
            serializedObject.ApplyModifiedProperties();
            EditorGUILayout.HelpBox("Fixed Clamps hold material. Sliding Guides allow cable to feed through. Enable Physics > Reel > Automatic for slack/tension control. Rebuild after changing midpoint modes.",MessageType.Info);
            if(GUILayout.Button("Fit Length To Current Curve")) hose.FitCableLengthToCurrentCurve();
            if(!Application.isPlaying) return;
            if(GUILayout.Button("Rebuild Physics")) hose.RebuildPhysics();
            EditorGUILayout.LabelField("Deployed length",hose.DeployedLength.ToString("F3")+" m");
            EditorGUILayout.LabelField("Stored length",hose.StoredLength.ToString("F3")+" m");
            EditorGUILayout.LabelField("Retraction blocked",hose.RetractionBlocked?"Yes":"No");
            EditorGUILayout.LabelField("Reel state",hose.ReelState.ToString());
            EditorGUILayout.LabelField("Required / target",hose.RequiredDeployedLength.ToString("F3")+" / "+hose.AutomaticTargetLength.ToString("F3")+" m");
            EditorGUILayout.LabelField("Reel-connected spans",hose.ReelConnectedSpanCount.ToString());
            EditorGUILayout.LabelField("Physics elements",hose.PhysicsSegmentCount.ToString());
            EditorGUILayout.LabelField("Broadphase / sweeps",hose.CollisionBroadphaseQueries+" / "+hose.CollisionSweepQueries+" per tick");
            EditorGUILayout.LabelField("Penetration tests",hose.CollisionPenetrationQueries+" per tick");
            EditorGUILayout.LabelField("Self-collision tests",hose.SelfCollisionPairs+" per tick");
            if(!string.IsNullOrEmpty(hose.LastPhysicsError)) EditorGUILayout.HelpBox(hose.LastPhysicsError,MessageType.Error);
        }
    }
}
