using UnityEditor;
using UnityEngine;

namespace RobotIK.Editor
{
    [CustomEditor(typeof(RobotBaseSixAxisMotionPlanner))]
    public sealed class RobotBaseSixAxisMotionPlannerEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            EditorGUI.BeginChangeCheck();
            DrawDefaultInspector();
            bool inspectorChanged = EditorGUI.EndChangeCheck();

            if (inspectorChanged && Application.isPlaying)
            {
                foreach (Object item in targets)
                    if (item is RobotBaseSixAxisMotionPlanner planner)
                        planner.ApplyRobotBoxColliderScale();
            }

            serializedObject.Update();
            SerializedProperty clearance = serializedObject.FindProperty("minimumObstacleClearance");
            if (clearance != null)
            {
                float clearanceMillimeters = clearance.floatValue * 1000f;
                EditorGUI.BeginChangeCheck();
                clearanceMillimeters = EditorGUILayout.FloatField("Minimum Obstacle Clearance (mm)", clearanceMillimeters);
                if (EditorGUI.EndChangeCheck())
                {
                    clearance.floatValue = Mathf.Max(0f, clearanceMillimeters) / 1000f;
                    serializedObject.ApplyModifiedProperties();
                }
            }

            RobotBaseSixAxisMotionPlanner plannerTarget = (RobotBaseSixAxisMotionPlanner)target;
            if (Application.isPlaying && plannerTarget.IsRrtSearchInProgress)
                Repaint();
            using (new EditorGUI.DisabledScope(true))
            {
                if (plannerTarget.IsRrtSearchInProgress)
                    EditorGUILayout.IntField("Current Search Iteration",
                        plannerTarget.CurrentRrtSearchIterations);
                EditorGUILayout.IntField(plannerTarget.SelectedPlanner == RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.GridAStar
                    ? "Grid Nodes Expanded" : "Planning Iterations", plannerTarget.PlanningIterations);
                if (plannerTarget.SelectedPlanner == RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.GridAStar)
                    EditorGUILayout.IntField("Grid Routes Tried", plannerTarget.GridRouteAttempts);
                if (plannerTarget.SelectedPlanner == RobotBaseSixAxisMotionPlanner.PlannerAlgorithm.CartesianArc)
                    EditorGUILayout.IntField("Cartesian Arc Attempts", plannerTarget.CartesianArcAttempts);
                EditorGUILayout.FloatField("Planning Time (s)", plannerTarget.PlanningTimeSeconds);
                EditorGUILayout.FloatField("Estimated Travel Time (s)", plannerTarget.EstimatedTravelTimeSeconds);
                if (plannerTarget.AdaptiveGoalEnabled || plannerTarget.UsedAdaptiveGoal)
                {
                    EditorGUILayout.Toggle("Adaptive Goal Used", plannerTarget.UsedAdaptiveGoal);
                    EditorGUILayout.IntField("Adaptive Goal Samples Tried", plannerTarget.AdaptiveGoalSamplesTried);
                    EditorGUILayout.FloatField("Goal Position Offset (mm)", plannerTarget.AdaptiveGoalPositionOffsetMillimeters);
                    EditorGUILayout.FloatField("Goal Rotation Offset (degrees)", plannerTarget.AdaptiveGoalRotationOffsetDegrees);
                }
            }

            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                if (GUILayout.Button("Apply Robot Box Collider Scale Now"))
                {
                    foreach (Object item in targets)
                        if (item is RobotBaseSixAxisMotionPlanner planner)
                            planner.ApplyRobotBoxColliderScale();
                }
            }

            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                if (GUILayout.Button("Reset to Start, Plan & Move A to B", GUILayout.Height(30f)))
                {
                    foreach (Object item in targets)
                    {
                        if (item is RobotBaseSixAxisMotionPlanner planner)
                            planner.ResetToStartAndPlan();
                    }
                }

                if (GUILayout.Button("Replay Planned Motion Without Replanning", GUILayout.Height(26f)))
                {
                    foreach (Object item in targets)
                    {
                        if (item is RobotBaseSixAxisMotionPlanner planner)
                            planner.ReplayPlannedMotion();
                    }
                }
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Enter Play mode to plan, move, or replay a stored path. Replay starts from the first saved joint state and does not run the planner again.", MessageType.Info);
        }
    }
}
