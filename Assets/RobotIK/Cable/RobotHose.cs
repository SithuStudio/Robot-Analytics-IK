using System.Collections.Generic;
using UnityEngine;
using RobotIK;


#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteAlways]
[DefaultExecutionOrder(10010)]

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RobotHose : MonoBehaviour
{
    [Header("Robot attachment points")]
    public Transform startPoint;
    public Transform endPoint;

    [Tooltip("Intermediate points in order from start to end.")]
    public List<Transform> midPoints = new List<Transform>();
    [HideInInspector] public List<RobotHoseMidpointMode> midpointModes = new List<RobotHoseMidpointMode>();

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

    [Header("Runtime physics and retraction")]
    [Tooltip("The reel feeds through Sliding Guides up to the next Fixed Clamp. Configure modes below.")]
    public RobotHosePhysicsSettings physics = new RobotHosePhysicsSettings();

    public float DeployedLength => simulation != null ? simulation.DeployedLength : 0;
    public float StoredLength => simulation != null ? simulation.StoredLength : 0;
    public bool RetractionBlocked => simulation != null && simulation.RetractionBlocked;
    public int PhysicsSegmentCount => simulation != null ? simulation.SegmentCount : 0;
    public int CollisionBroadphaseQueries => simulation != null ? simulation.BroadphaseQueries : 0;
    public int CollisionSweepQueries => simulation != null ? simulation.SweepQueries : 0;
    public int CollisionPenetrationQueries => simulation != null ? simulation.PenetrationQueries : 0;
    public int SelfCollisionPairs => simulation != null ? simulation.SelfCollisionPairs : 0;
    public RobotHoseReelState ReelState => simulation != null ? simulation.ReelState : RobotHoseReelState.Holding;
    public float RequiredDeployedLength => simulation != null ? simulation.RequiredDeployedLength : 0;
    public float AutomaticTargetLength => simulation != null ? simulation.AutomaticTargetLength : 0;
    public int ReelConnectedSpanCount => simulation != null ? simulation.ReelConnectedSpanCount : 0;
    public string LastPhysicsError { get; private set; }
    private RobotHoseSimulation simulation;
    private bool initializationAttempted;
    private readonly List<Vector3> simulatedCenters = new List<Vector3>();
    private readonly List<Quaternion> simulatedFrames = new List<Quaternion>();
    private readonly List<float> simulatedDistances = new List<float>();

    private Mesh hoseMesh;

    private Vector3[] vertices;
    private Vector3[] normals;
    private Vector2[] uvs;
    private int[] triangles;
    private float[] ringCosines, ringSines;

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
        initializationAttempted = false;

#if UNITY_EDITOR
        // Remove first to avoid duplicate subscriptions.
        EditorApplication.update -= EditorPreviewUpdate;
        EditorApplication.update += EditorPreviewUpdate;
#endif
    }

    private void OnDisable()
    {
        DisposeSimulation();
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
        if (physics == null) physics = new RobotHosePhysicsSettings();
        physics.cableLength = Mathf.Max(0, physics.cableLength);
        physics.initialDeployedLength = Mathf.Max(0, physics.initialDeployedLength);
        physics.targetDeployedLength = Mathf.Max(0, physics.targetDeployedLength);
        physics.feedSpeed = Mathf.Max(0, physics.feedSpeed);
        physics.massPerMeter = Mathf.Max(0.001f, physics.massPerMeter);
        if(physics.targetSegmentLength<0.01f) physics.targetSegmentLength=0.06f;
        if(physics.reel==null) physics.reel=new RobotHoseReelSettings();
        physics.reel.desiredSlack=Mathf.Max(0,physics.reel.desiredSlack);
        physics.reel.slackTolerance=Mathf.Max(0.001f,physics.reel.slackTolerance);
        physics.reel.extensionSpeed=Mathf.Max(0,physics.reel.extensionSpeed);
        physics.reel.retractionSpeed=Mathf.Max(0,physics.reel.retractionSpeed);
        physics.reel.acceleration=Mathf.Max(0.01f,physics.reel.acceleration);
        physics.reel.guideSlideSpeed=Mathf.Max(0,physics.reel.guideSlideSpeed);
        physics.reel.retractionDelay=Mathf.Max(0,physics.reel.retractionDelay);

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

    private void FixedUpdate()
    {
        if (!Application.isPlaying) return;
        if (physics == null || !physics.enabled)
        {
            DisposeSimulation();
            initializationAttempted = false;
            return;
        }
        if (!initializationAttempted) RebuildPhysics();
        if (simulation == null) return;
        if (!simulation.AttachmentsValid)
        {
            LastPhysicsError = "A hose attachment was destroyed. Reassign it and rebuild hose physics.";
            DisposeSimulation();
            return;
        }
        simulation.Step(Time.fixedDeltaTime);
    }

    private void DisposeSimulation()
    {
        simulation?.Dispose();
        simulation = null;
    }

    /// <summary>Use after changing attachment topology, radius, or initial route in Play mode.</summary>
    [ContextMenu("Rebuild Hose Physics")]
    public void RebuildPhysics()
    {
        DisposeSimulation();
        initializationAttempted = true;
        LastPhysicsError = null;
        if (!Application.isPlaying || physics == null || !physics.enabled) return;
        try
        {
            if (!startPoint || !endPoint) throw new System.ArgumentException("Assign hose start and end attachments.");
            CollectPathPoints();
            var anchors = new List<Transform> { startPoint };
            var modes=new List<RobotHoseMidpointMode>();
            if (midPoints != null) for(int i=0;i<midPoints.Count;i++)
            {
                if(!midPoints[i]) continue;
                anchors.Add(midPoints[i]);
                modes.Add(midpointModes!=null && i<midpointModes.Count?midpointModes[i]:RobotHoseMidpointMode.FixedClamp);
            }
            anchors.Add(endPoint);
            var routes = new Vector3[pathPoints.Count-1][];
            for (int span=0; span<routes.Length; span++)
            {
                routes[span] = new Vector3[segmentsPerSpan+1];
                for (int i=0; i<=segmentsPerSpan; i++)
                {
                    SampleCurve(span, (float)i/segmentsPerSpan, out Vector3 position, out _);
                    routes[span][i] = transform.TransformPoint(position);
                }
            }
            Vector3 scale = transform.lossyScale;
            float worldScale = Mathf.Max(Mathf.Abs(scale.x),Mathf.Abs(scale.y),Mathf.Abs(scale.z));
            simulation = new RobotHoseSimulation(anchors.ToArray(), routes, radius*worldScale,
                physics, gameObject.layer, gameObject.scene,modes.ToArray());
        }
        catch (System.ArgumentException error)
        {
            LastPhysicsError = error.Message;
            Debug.LogWarning("RobotHose: " + LastPhysicsError, this);
        }
    }

    [ContextMenu("Fit Cable Length To Current Curve")]
    public void FitCableLengthToCurrentCurve()
    {
        if (!startPoint || !endPoint) return;
#if UNITY_EDITOR
        Undo.RecordObject(this, "Fit hose cable length");
#endif
        CollectPathPoints();
        float length=0;
        for (int span=0;span<pathPoints.Count-1;span++)
        {
            SampleCurve(span,0,out Vector3 previous,out _);
            previous=transform.TransformPoint(previous);
            for(int i=1;i<=segmentsPerSpan;i++)
            {
                SampleCurve(span,(float)i/segmentsPerSpan,out Vector3 point,out _);
                point=transform.TransformPoint(point);
                length+=Vector3.Distance(previous,point); previous=point;
            }
        }
        if (physics == null) physics=new RobotHosePhysicsSettings();
        physics.cableLength=length;
        physics.initialDeployedLength=length;
        physics.targetDeployedLength=length;
#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif
    }

    public void SetTargetDeployedLength(float meters)
    {
        if (float.IsNaN(meters) || float.IsInfinity(meters) || meters<=0)
            throw new System.ArgumentOutOfRangeException(nameof(meters));
        if (physics == null) physics=new RobotHosePhysicsSettings();
        if(physics.reel!=null) physics.reel.automatic=false;
        physics.targetDeployedLength=meters;
    }

    public void SetAutomaticReel(bool enabled)
    {
        if(physics==null) physics=new RobotHosePhysicsSettings();
        if(physics.reel==null) physics.reel=new RobotHoseReelSettings();
        physics.reel.automatic=enabled;
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

        if (Application.isPlaying && simulation != null && physics.enabled)
        {
            UpdateSimulatedHose();
            return;
        }

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
        ringCosines=new float[ringSize]; ringSines=new float[ringSize];
        for(int j=0;j<ringSize;j++)
        {
            float angle=(float)j/radialSegments*Mathf.PI*2;
            ringCosines[j]=Mathf.Cos(angle); ringSines[j]=Mathf.Sin(angle);
        }
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

    private void UpdateSimulatedHose()
    {
        float alpha=Mathf.Clamp01((Time.time-Time.fixedTime)/Time.fixedDeltaTime);
        simulation.GetRenderSamples(simulatedCenters,simulatedFrames,simulatedDistances,alpha,segmentsPerSpan);
        int totalSegments=simulatedCenters.Count-1;
        if(totalSegments!=builtTotalSegments || radialSegments!=builtRadialSegments) BuildTopology(totalSegments);
        int stride=radialSegments+1;
        Vector3 scale=transform.lossyScale;
        float worldRadius=radius*Mathf.Max(Mathf.Abs(scale.x),Mathf.Abs(scale.y),Mathf.Abs(scale.z));
        Matrix4x4 worldToLocal=transform.worldToLocalMatrix, normalToLocal=transform.localToWorldMatrix.transpose;
        for(int i=0;i<=totalSegments;i++)
        {
            Vector3 tangent=(simulatedCenters[Mathf.Min(i+1,totalSegments)]-simulatedCenters[Mathf.Max(0,i-1)]).normalized;
            if(tangent.sqrMagnitude<1e-8f) tangent=simulatedFrames[i]*Vector3.forward;
            Vector3 normal=RodMath.Normal(tangent,simulatedFrames[i]*Vector3.right);
            Vector3 binormal=Vector3.Cross(tangent,normal).normalized;
            for(int j=0;j<=radialSegments;j++)
            {
                float u=(float)j/radialSegments;
                Vector3 radial=normal*ringCosines[j]+binormal*ringSines[j];
                int index=i*stride+j;
                vertices[index]=worldToLocal.MultiplyPoint3x4(simulatedCenters[i]+radial*worldRadius);
                // World normal -> local normal, including nonuniform object scale.
                normals[index]=normalToLocal.MultiplyVector(radial).normalized;
                uvs[index]=new Vector2(u,simulatedDistances[i]*textureRepeatsPerUnit);
            }
        }
        hoseMesh.vertices=vertices; hoseMesh.normals=normals; hoseMesh.uv=uvs;
        hoseMesh.RecalculateBounds();
    }

    private void OnDestroy()
    {
        DisposeSimulation();
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
