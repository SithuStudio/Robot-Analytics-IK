using System.Collections.Generic;
using UnityEngine;


#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteAlways]

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RobotHose : MonoBehaviour
{
    [Header("Robot attachment points")]
    public Transform startPoint;
    public Transform endPoint;

    [Tooltip("Intermediate points in order from start to end.")]
    public List<Transform> midPoints = new List<Transform>();

    [Header("Curve shape")]
    [Min(0.001f)] public float startHandleLength = 0.25f;
    [Min(0.001f)] public float endHandleLength = 0.25f;

    [Tooltip("Controls curve smoothness around intermediate points.")]
    [Range(0f, 1f)] public float midpointSmoothness = 1f;

    [Header("Tube")]
    [Min(0.0001f)] public float radius = 0.015f;

    [Tooltip("Number of tube sections between each pair of points.")]
    [Range(4, 64)] public int segmentsPerSpan = 16;

    [Range(3, 32)] public int radialSegments = 12;

    [Tooltip("Texture repetitions per unit of hose length.")]
    [Min(0f)] public float textureRepeatsPerUnit = 4f;

    private Mesh hoseMesh;

    private Vector3[] vertices;
    private Vector3[] normals;
    private Vector2[] uvs;
    private int[] triangles;

    private readonly List<Vector3> pathPoints =
        new List<Vector3>();

    private int builtTotalSegments = -1;
    private int builtRadialSegments = -1;

#if UNITY_EDITOR
    private double nextEditorRefresh;
#endif

    private void OnEnable()
    {
        InitializeMesh();

#if UNITY_EDITOR
        // Remove first to avoid duplicate subscriptions.
        EditorApplication.update -= EditorPreviewUpdate;
        EditorApplication.update += EditorPreviewUpdate;
#endif
    }

    private void OnDisable()
    {
#if UNITY_EDITOR
        EditorApplication.update -= EditorPreviewUpdate;
#endif
    }

    private void InitializeMesh()
    {
        if (hoseMesh != null)
            return;

        hoseMesh = new Mesh
        {
            name = "Multi-Point Robot Hose Preview",

            // This is a temporary generated mesh, not a saved asset.
            hideFlags = HideFlags.HideAndDontSave,

            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        hoseMesh.MarkDynamic();

        GetComponent<MeshFilter>().sharedMesh = hoseMesh;

        // Force topology to be rebuilt.
        builtTotalSegments = -1;
        builtRadialSegments = -1;
    }

    private void OnValidate()
    {
        // Only validate data here. Mesh work happens during the update.
        segmentsPerSpan = Mathf.Clamp(segmentsPerSpan, 4, 64);
        radialSegments = Mathf.Clamp(radialSegments, 3, 32);

        radius = Mathf.Max(radius, 0.0001f);
        startHandleLength = Mathf.Max(startHandleLength, 0.001f);
        endHandleLength = Mathf.Max(endHandleLength, 0.001f);
        midpointSmoothness = Mathf.Clamp01(midpointSmoothness);
        textureRepeatsPerUnit = Mathf.Max(textureRepeatsPerUnit, 0f);

        builtTotalSegments = -1;
        builtRadialSegments = -1;
    }

#if UNITY_EDITOR
    private void EditorPreviewUpdate()
    {
        if (Application.isPlaying || this == null || !isActiveAndEnabled)
            return;

        // Limit editor preview updates to approximately 30 FPS.
        double currentTime = EditorApplication.timeSinceStartup;

        if (currentTime < nextEditorRefresh)
            return;

        nextEditorRefresh = currentTime + 1.0 / 30.0;

        RefreshHose();
        SceneView.RepaintAll();
    }
#endif


    private void LateUpdate()
    {
        if (Application.isPlaying)
        {
            RefreshHose();
        }
    }

    private void RefreshHose()
    {
        InitializeMesh();

        if (startPoint == null || endPoint == null)
        {
            hoseMesh.Clear();
            builtTotalSegments = -1;
            return;
        }

        segmentsPerSpan = Mathf.Clamp(segmentsPerSpan, 4, 64);
        radialSegments = Mathf.Clamp(radialSegments, 3, 32);

        CollectPathPoints();

        int spanCount = pathPoints.Count - 1;
        int totalSegments = spanCount * segmentsPerSpan;

        if (totalSegments != builtTotalSegments ||
            radialSegments != builtRadialSegments)
        {
            BuildTopology(totalSegments);
        }

        UpdateHose(totalSegments);
    }


    private void CollectPathPoints()
    {
        pathPoints.Clear();

        pathPoints.Add(
            transform.InverseTransformPoint(startPoint.position)
        );

        if (midPoints != null)
        {
            foreach (Transform point in midPoints)
            {
                if (point != null)
                {
                    pathPoints.Add(
                        transform.InverseTransformPoint(point.position)
                    );
                }
            }
        }

        pathPoints.Add(
            transform.InverseTransformPoint(endPoint.position)
        );
    }

    private void BuildTopology(int totalSegments)
    {
        builtTotalSegments = totalSegments;
        builtRadialSegments = radialSegments;

        int ringSize = radialSegments + 1;
        int vertexCount = (totalSegments + 1) * ringSize;

        vertices = new Vector3[vertexCount];
        normals = new Vector3[vertexCount];
        uvs = new Vector2[vertexCount];
        triangles = new int[totalSegments * radialSegments * 6];

        int index = 0;

        for (int i = 0; i < totalSegments; i++)
        {
            for (int j = 0; j < radialSegments; j++)
            {
                int a = i * ringSize + j;
                int b = a + 1;
                int c = a + ringSize;
                int d = c + 1;

                triangles[index++] = a;
                triangles[index++] = b;
                triangles[index++] = c;

                triangles[index++] = b;
                triangles[index++] = d;
                triangles[index++] = c;
            }
        }

        hoseMesh.Clear();

        // Set initial vertices before assigning triangle indices.
        hoseMesh.vertices = vertices;
        hoseMesh.triangles = triangles;
    }

    private Vector3 GetTangent(int pointIndex)
    {
        if (pointIndex == 0)
        {
            // Matches the old script's Bézier handle behavior.
            return transform.InverseTransformVector(
                startPoint.forward * startHandleLength * 3f
            );
        }

        if (pointIndex == pathPoints.Count - 1)
        {
            // End forward points from the connector into the hose.
            // The curve travels toward that connector.
            return transform.InverseTransformVector(
                -endPoint.forward * endHandleLength * 3f
            );
        }

        return (
            pathPoints[pointIndex + 1] -
            pathPoints[pointIndex - 1]
        ) * (0.5f * midpointSmoothness);
    }

    private void SampleCurve(
        int span,
        float t,
        out Vector3 position,
        out Vector3 tangent)
    {
        Vector3 p0 = pathPoints[span];
        Vector3 p1 = pathPoints[span + 1];

        Vector3 m0 = GetTangent(span);
        Vector3 m1 = GetTangent(span + 1);

        float t2 = t * t;
        float t3 = t2 * t;

        // Cubic Hermite position.
        position =
            (2f * t3 - 3f * t2 + 1f) * p0 +
            (t3 - 2f * t2 + t) * m0 +
            (-2f * t3 + 3f * t2) * p1 +
            (t3 - t2) * m1;

        // Derivative of the Hermite curve.
        tangent =
            (6f * t2 - 6f * t) * p0 +
            (3f * t2 - 4f * t + 1f) * m0 +
            (-6f * t2 + 6f * t) * p1 +
            (3f * t2 - 2f * t) * m1;
    }

    private void UpdateHose(int totalSegments)
    {
        int ringSize = radialSegments + 1;
        int spanCount = pathPoints.Count - 1;

        Vector3 previousTangent = Vector3.forward;
        Vector3 ringNormal = Vector3.up;
        Vector3 previousCenter = Vector3.zero;

        float distanceAlongHose = 0f;

        for (int i = 0; i <= totalSegments; i++)
        {
            int span = Mathf.Min(
                i / segmentsPerSpan,
                spanCount - 1
            );

            float t = (float)(
                i - span * segmentsPerSpan
            ) / segmentsPerSpan;

            SampleCurve(
                span,
                t,
                out Vector3 center,
                out Vector3 tangent
            );

            if (tangent.sqrMagnitude < 0.000001f)
            {
                tangent = previousTangent;
            }

            tangent.Normalize();

            if (i == 0)
            {
                ringNormal = CreateNormal(tangent);
            }
            else
            {
                distanceAlongHose += Vector3.Distance(
                    previousCenter,
                    center
                );

                Quaternion rotation = Quaternion.FromToRotation(
                    previousTangent,
                    tangent
                );

                ringNormal = Vector3.ProjectOnPlane(
                    rotation * ringNormal,
                    tangent
                );

                if (ringNormal.sqrMagnitude < 0.000001f)
                {
                    ringNormal = CreateNormal(tangent);
                }
                else
                {
                    ringNormal.Normalize();
                }
            }

            Vector3 binormal = Vector3.Cross(
                tangent,
                ringNormal
            ).normalized;

            for (int j = 0; j <= radialSegments; j++)
            {
                float u = (float)j / radialSegments;
                float angle = u * Mathf.PI * 2f;

                Vector3 radial =
                    ringNormal * Mathf.Cos(angle) +
                    binormal * Mathf.Sin(angle);

                int vertexIndex = i * ringSize + j;

                vertices[vertexIndex] =
                    center + radial * Mathf.Max(radius, 0.0001f);

                normals[vertexIndex] = radial;

                uvs[vertexIndex] = new Vector2(
                    u,
                    distanceAlongHose * textureRepeatsPerUnit
                );
            }

            previousCenter = center;
            previousTangent = tangent;
        }

        hoseMesh.vertices = vertices;
        hoseMesh.normals = normals;
        hoseMesh.uv = uvs;
        hoseMesh.RecalculateBounds();
    }

    private static Vector3 CreateNormal(Vector3 tangent)
    {
        Vector3 reference =
            Mathf.Abs(Vector3.Dot(tangent, Vector3.up)) > 0.95f
            ? Vector3.right
            : Vector3.up;

        return Vector3.Cross(reference, tangent).normalized;
    }

    private void OnDestroy()
    {
    #if UNITY_EDITOR
            EditorApplication.update -= EditorPreviewUpdate;
    #endif

        if (hoseMesh == null)
            return;

        MeshFilter meshFilter = GetComponent<MeshFilter>();

        if (meshFilter != null &&
            meshFilter.sharedMesh == hoseMesh)
        {
            meshFilter.sharedMesh = null;
        }

        if (Application.isPlaying)
        {
            Destroy(hoseMesh);
        }
        else
        {
            DestroyImmediate(hoseMesh);
        }

        hoseMesh = null;
    }

}
