using System;
using UnityEngine;

namespace RobotIK
{
    /// <summary>Rest-route sampling and fixed-length slack fitting; no Unity objects required.</summary>
    public static class ElasticCableRouting
    {
        public static Vector3[] Sample(Vector3[] controls, int samples = 256)
        {
            if (controls == null || controls.Length < 2) throw new ArgumentException("Route needs two endpoints.");
            samples = Math.Max(samples, controls.Length * 16);
            var result = new Vector3[samples + 1];
            for (int i = 0; i <= samples; i++)
            {
                float u = (float)i / samples * (controls.Length-1);
                int section = Math.Min(controls.Length-2, (int)u);
                float t = u-section;
                Vector3 p0 = controls[Math.Max(0,section-1)], p1 = controls[section];
                Vector3 p2 = controls[section+1], p3 = controls[Math.Min(controls.Length-1,section+2)];
                // Catmull-Rom interpolation, with half-chord endpoint tangents.
                result[i] = 0.5f * ((2*p1) + (-p0+p2)*t + (2*p0-5*p1+4*p2-p3)*t*t + (-p0+3*p1-3*p2+p3)*t*t*t);
            }
            result[0] = controls[0]; result[samples] = controls[controls.Length-1];
            return result;
        }
        public static float Length(Vector3[] points)
        {
            float length = 0;
            for (int i = 1; i < points.Length; i++) length += Vector3.Distance(points[i-1],points[i]);
            return length;
        }
        public static Vector3[] Build(Vector3[] controls, float restLength, int segments, Vector3 bowDirection)
        {
            Vector3[] dense = Sample(controls);
            float required = Length(dense);
            if (restLength + 0.0001f < required)
                throw new ArgumentException($"Route requires {required:F3} m, but only {restLength:F3} m is allocated.");
            if (segments < 3) throw new ArgumentOutOfRangeException(nameof(segments));
            var distances = new float[dense.Length];
            for (int i = 1; i < dense.Length; i++) distances[i] = distances[i-1] + Vector3.Distance(dense[i-1],dense[i]);
            var baseline = new Vector3[segments+1];
            int index = 0;
            for (int i = 0; i <= segments; i++)
            {
                float at = required*i/segments;
                while (index < dense.Length-2 && distances[index+1] < at) index++;
                float width = distances[index+1]-distances[index];
                baseline[i] = Vector3.LerpUnclamped(dense[index],dense[index+1],width > 1e-9f ? (at-distances[index])/width : 0);
            }
            baseline[0] = controls[0]; baseline[segments] = controls[controls.Length-1];
            Vector3 chord = (baseline[segments]-baseline[0]).normalized;
            Vector3 bow = RodMath.Normal(chord,bowDirection);
            var output = new Vector3[segments+1];
            float low = 0, high = Math.Max(0.01f,restLength);
            for (int iteration = 0; iteration < 64; iteration++)
            {
                float amplitude = (low+high)*0.5f;
                for (int i = 0; i <= segments; i++)
                    output[i] = baseline[i] + bow * (amplitude*(float)Math.Sin(Math.PI*i/segments));
                if (Length(output) < restLength) low=amplitude; else high=amplitude;
            }
            output[0] = baseline[0]; output[segments] = baseline[segments];
            return output;
        }
    }
}
