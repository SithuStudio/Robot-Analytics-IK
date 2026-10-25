using UnityEditor;
using UnityEngine;

namespace RobotIK.Editor
{
    [CustomEditor(typeof(RobotLinearTrackRig))]
    public sealed class RobotLinearTrackRigEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            var track = (RobotLinearTrackRig)target;
            var planner = track.GetComponent<RobotBaseSixAxisMotionPlanner>();
            bool locked = track.IsLocked || (planner != null && planner.IsBusy);
            EditorGUILayout.HelpBox(locked
                ? "E1 and calibration are locked. Cancel planning or stop motion before editing."
                : "Setup preview only. E1 remains fixed during six-axis planning and playback. Poses use TrackRoot coordinates: millimeters and degrees.", MessageType.Info);
            using (new EditorGUI.DisabledScope(locked))
            {
                EditorGUI.BeginChangeCheck();
                DrawDefaultInspector();
                bool changed = EditorGUI.EndChangeCheck();
                bool preview = GUILayout.Button("Apply selected E1 · setup preview");
                if ((changed && track.TrackEnabled) || preview)
                {
                    if (track.Carriage != null) Undo.RecordObject(track.Carriage, "Preview track E1");
                    if (!track.TryApplySelected(out string error)) Debug.LogWarning(error, track);
                    if (track.Carriage != null) EditorUtility.SetDirty(track.Carriage);
                    SceneView.RepaintAll();
                }
            }
            if (track.TrackEnabled && !track.ValidateConfiguration(out string validationError))
                EditorGUILayout.HelpBox(validationError, MessageType.Error);
        }
    }
}
