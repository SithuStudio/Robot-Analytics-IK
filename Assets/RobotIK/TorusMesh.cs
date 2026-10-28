using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ProceduralTorus : MonoBehaviour
{
    public float ringRadius = 1f;   // Distance from center to tube center
    public float tubeRadius = 0.3f;  // Thickness of the tube
    public int segments = 32;       // Radial segments (around the main ring)
    public int sides = 16;          // Tubular segments (around the tube)



    [ContextMenu("Create Torus")]
    void GenerateTorus()
    {
        Mesh mesh = new Mesh();
        mesh.name = "Procedural Torus";

        int vertexCount = (segments + 1) * (sides + 1);
        Vector3[] vertices = new Vector3[vertexCount];
        int[] triangles = new int[segments * sides * 6];

        // Calculate vertices
        int v = 0;
        for (int seg = 0; seg <= segments; seg++)
        {
            float u = (float)seg / segments * Mathf.PI * 2f;
            for (int side = 0; side <= sides; side++)
            {
                float vAngle = (float)side / sides * Mathf.PI * 2f;

                float x = (ringRadius + tubeRadius * Mathf.Cos(vAngle)) * Mathf.Cos(u);
                float z = (ringRadius + tubeRadius * Mathf.Cos(vAngle)) * Mathf.Sin(u);
                float y = tubeRadius * Mathf.Sin(vAngle);

                vertices[v++] = new Vector3(x, y, z);
            }
        }

        // Build Triangles
        int t = 0;
        for (int seg = 0; seg < segments; seg++)
        {
            for (int side = 0; side < sides; side++)
            {
                int current = seg * (sides + 1) + side;
                int next = current + sides + 1;

                triangles[t++] = current;
                triangles[t++] = current + 1;
                triangles[t++] = next;

                triangles[t++] = next;
                triangles[t++] = current + 1;
                triangles[t++] = next + 1;
            }
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        GetComponent<MeshFilter>().mesh = mesh;
    }
}