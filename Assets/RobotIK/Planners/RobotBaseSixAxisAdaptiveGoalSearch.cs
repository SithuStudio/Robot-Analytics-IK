using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Deterministic nearby TCP pose samples. IK and full robot clearance remain the caller's responsibility.</summary>
    public static class RobotBaseSixAxisAdaptiveGoalSearch
    {
        public readonly struct GoalPose
        {
            public readonly Vector3 Position;
            public readonly Quaternion Rotation;
            public readonly float PositionOffsetMillimeters;
            public readonly float RotationOffsetDegrees;
            internal readonly float Score;
            internal readonly int Order;

            internal GoalPose(Vector3 position, Quaternion rotation, float positionOffset,
                float rotationOffset, float score, int order)
            {
                Position = position; Rotation = rotation;
                PositionOffsetMillimeters = positionOffset; RotationOffsetDegrees = rotationOffset;
                Score = score; Order = order;
            }
        }

        /// <summary>Samples inside a position sphere and orientation-angle bound, sorted by normalized pose deviation. Excludes the exact goal.</summary>
        public static List<GoalPose> CreateCandidates(Vector3 position, Quaternion rotation,
            float positionToleranceMillimeters, float rotationToleranceDegrees, int maximumSamples)
        {
            float positionTolerance = Math.Max(0f, positionToleranceMillimeters);
            float rotationTolerance = Math.Max(0f, Math.Min(180f, rotationToleranceDegrees));
            var candidates = new List<GoalPose>();
            if (maximumSamples <= 0 || (positionTolerance == 0f && rotationTolerance == 0f)) return candidates;
            var directions = new List<Vector3>();
            for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
            for (int z = -1; z <= 1; z++)
                if (x != 0 || y != 0 || z != 0) directions.Add(new Vector3(x, y, z));
            // Axis directions precede diagonal directions at the same offset.
            directions.Sort((a, b) =>
            {
                int byLength = (a.x * a.x + a.y * a.y + a.z * a.z).CompareTo(b.x * b.x + b.y * b.y + b.z * b.z);
                if (byLength != 0) return byLength;
                int byX = a.x.CompareTo(b.x); if (byX != 0) return byX;
                int byY = a.y.CompareTo(b.y); return byY != 0 ? byY : a.z.CompareTo(b.z);
            });
            var axes = new[] { new Vector3(1,0,0), new Vector3(-1,0,0), new Vector3(0,1,0),
                new Vector3(0,-1,0), new Vector3(0,0,1), new Vector3(0,0,-1) };
            for (int shell = 1; shell <= 4; shell++)
            {
                float fraction = shell * 0.25f;
                float shift = positionTolerance * fraction;
                float angle = rotationTolerance * fraction;
                if (positionTolerance > 0f)
                    for (int i = 0; i < directions.Count; i++)
                    {
                        Vector3 d = directions[i];
                        float scale = shift * 0.001f / (float)Math.Sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
                        var shifted = new Vector3(position.x + d.x*scale, position.y + d.y*scale, position.z + d.z*scale);
                        candidates.Add(new GoalPose(shifted, rotation, shift, 0f, fraction*fraction, candidates.Count));
                    }
                if (rotationTolerance > 0f)
                    for (int i = 0; i < axes.Length; i++)
                        candidates.Add(new GoalPose(position, OffsetRotation(rotation, axes[i], angle),
                            0f, angle, fraction*fraction, candidates.Count));
                if (positionTolerance > 0f && rotationTolerance > 0f)
                    for (int i = 0; i < directions.Count; i++)
                    {
                        Vector3 d = directions[i];
                        float scale = shift * 0.001f / (float)Math.Sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
                        var shifted = new Vector3(position.x + d.x*scale, position.y + d.y*scale, position.z + d.z*scale);
                        candidates.Add(new GoalPose(shifted, OffsetRotation(rotation, axes[(i + shell) % axes.Length], angle),
                            shift, angle, 2f*fraction*fraction, candidates.Count));
                    }
            }
            candidates.Sort((a,b) => { int score = a.Score.CompareTo(b.Score); return score != 0 ? score : a.Order.CompareTo(b.Order); });
            if (candidates.Count > maximumSamples) candidates.RemoveRange(maximumSamples, candidates.Count - maximumSamples);
            return candidates;
        }

        // Managed quaternion math also keeps sampling independent of Unity scene APIs.
        private static Quaternion OffsetRotation(Quaternion q, Vector3 localAxis, float degrees)
        {
            double half = degrees * Math.PI / 360.0;
            float sine = (float)Math.Sin(half), cosine = (float)Math.Cos(half);
            float x = localAxis.x*sine, y = localAxis.y*sine, z = localAxis.z*sine;
            return new Quaternion(q.w*x + q.x*cosine + q.y*z - q.z*y,
                q.w*y - q.x*z + q.y*cosine + q.z*x,
                q.w*z + q.x*y - q.y*x + q.z*cosine,
                q.w*cosine - q.x*x - q.y*y - q.z*z);
        }
    }
}
