using UnityEngine;

public class GridOccupancyScanner : MonoBehaviour
{
    [Header("Scan volume")]
    [Min(0.05f)] public float voxelSize = 0.5f;
    public Vector3 scanSize = new Vector3(10f, 5f, 10f);
    public LayerMask obstacleLayers = Physics.DefaultRaycastLayers;
        
    [Header("Display")]
    public bool drawGrid = true;
    [Range(0.5f, 1f)] public float gizmoCellScale = 0.9f;
    public Color OccupiedArea = Color.red;
    public Color FreeArea = Color.cyan;

    private bool[,,] occupied;
    private int countX;
    private int countY;
    private int countZ;
    private bool hasScanned;
    private RobotIK.CartesianOccupancyGrid planningSnapshot;
    private Matrix4x4 scannedTransform;
    private Vector3 scannedSize;
    private float scannedVoxelSize;
    private int scannedLayers;
    private int scanVersion;
    public float ScanProgress { get; private set; }
    public string LastScanError { get; private set; }

    public bool TryGetPlanningSnapshot(out RobotIK.CartesianOccupancyGrid snapshot, out string error)
    {
        snapshot = null;
        if (!hasScanned || planningSnapshot == null)
        { error = LastScanError ?? "Grid Occupancy Scanner has no completed scan."; return false; }
        if (!scannedTransform.Equals(transform.localToWorldMatrix) || scannedSize != scanSize ||
            scannedVoxelSize != voxelSize || scannedLayers != obstacleLayers.value)
        { error = "Scanner settings or transform changed after scanning. Run Scan Once again."; return false; }
        snapshot = planningSnapshot; error = null; return true;
    }

    // Select the component's menu in the Inspector and choose "Scan Once".
    // You can also call this method from a UI Button or another script.
    [ContextMenu("Scan Once")]
    public void ScanOnce()
    {
        var scan = ScanOverFrames();
        try { while (scan.MoveNext()) { } }
        finally { (scan as System.IDisposable)?.Dispose(); }
    }

    /// <summary>Physics stays on the main thread, with a soft 3 ms frame budget.
    /// The planning snapshot becomes available only after the complete scan.</summary>
    public System.Collections.IEnumerator ScanOverFrames(System.Threading.CancellationToken cancellationToken = default)
    {
        ClearMap();
        int version = scanVersion;
        if (voxelSize <= 0f || scanSize.x <= 0 || scanSize.y <= 0 || scanSize.z <= 0)
        {
            LastScanError = "Voxel Size and scan dimensions must be greater than zero.";
            Debug.LogError(LastScanError, this);
            yield break;
        }
        Vector3 scannerScale = transform.lossyScale;
        if (Mathf.Abs(scannerScale.x)<1e-6f || Mathf.Abs(scannerScale.y)<1e-6f || Mathf.Abs(scannerScale.z)<1e-6f)
        { LastScanError = "Scanner transform scale must be nonzero."; Debug.LogError(LastScanError, this); yield break; }
        Physics.SyncTransforms();

        countX = Mathf.CeilToInt(scanSize.x / voxelSize);
        countY = Mathf.CeilToInt(scanSize.y / voxelSize);
        countZ = Mathf.CeilToInt(scanSize.z / voxelSize);
        if ((long)countX * countY * countZ > 2000000)
        { LastScanError = "Scan exceeds 2,000,000 cells. Increase voxel size or reduce scan volume."; Debug.LogError(LastScanError, this); yield break; }

        occupied = new bool[countX, countY, countZ];
        scannedTransform = transform.localToWorldMatrix; scannedSize = scanSize;
        scannedVoxelSize = voxelSize; scannedLayers = obstacleLayers.value;
        var frame = System.Diagnostics.Stopwatch.StartNew();
        int processed = 0, total = countX * countY * countZ;

        for (int x = 0; x < countX; x++)
        {
            for (int y = 0; y < countY; y++)
            {
                for (int z = 0; z < countZ; z++)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    GetCell(x, y, z, out Vector3 center, out Vector3 size);

                    // Cover the complete cell, including thin obstacles at cell edges.
                    Vector3 scale = transform.lossyScale;
                    Vector3 halfExtents = Vector3.Scale(size, new Vector3(Mathf.Abs(scale.x),Mathf.Abs(scale.y),Mathf.Abs(scale.z))) * 0.5f;

                    occupied[x, y, z] = Physics.CheckBox(
                        center,
                        halfExtents,
                        transform.rotation,
                        obstacleLayers,
                        QueryTriggerInteraction.Ignore
                    );
                    ScanProgress = ++processed / (float)total;
                    if (frame.Elapsed.TotalMilliseconds >= 3 && processed < total)
                    {
                        yield return null;
                        cancellationToken.ThrowIfCancellationRequested();
                        if (version != scanVersion) yield break;
                        if (!scannedTransform.Equals(transform.localToWorldMatrix) || scannedSize != scanSize ||
                            scannedVoxelSize != voxelSize || scannedLayers != obstacleLayers.value)
                        { LastScanError = "Scanner settings or transform changed during scanning. Restart planning."; yield break; }
                        frame.Restart();
                    }
                }
            }
        }

        cancellationToken.ThrowIfCancellationRequested();
        planningSnapshot = new RobotIK.CartesianOccupancyGrid(occupied, transform.TransformPoint(-scanSize * 0.5f),
            transform.TransformVector(Vector3.right * voxelSize), transform.TransformVector(Vector3.up * voxelSize),
            transform.TransformVector(Vector3.forward * voxelSize), scanSize / voxelSize);
        hasScanned = true;
        Debug.Log($"Occupancy scan complete: {countX} x {countY} x {countZ} cells.", this);
    }

    public bool IsOccupied(Vector3 worldPosition)
    {
        return !TryGetPlanningSnapshot(out var snapshot, out _) || !snapshot.IsFree(worldPosition);
    }

    public void ClearMap()
    {
        scanVersion++;
        ScanProgress = 0; LastScanError = null;
        occupied = null;
        hasScanned = false;
        planningSnapshot = null;
    }

    private void GetCell(int x, int y, int z, out Vector3 worldCenter, out Vector3 cellSize)
    {
        Vector3 localMin = -scanSize * 0.5f;

        float startX = x * voxelSize;
        float startY = y * voxelSize;
        float startZ = z * voxelSize;

        cellSize = new Vector3(
            Mathf.Min(voxelSize, scanSize.x - startX),
            Mathf.Min(voxelSize, scanSize.y - startY),
            Mathf.Min(voxelSize, scanSize.z - startZ)
        );

        Vector3 localCenter = localMin + new Vector3(
            startX + cellSize.x * 0.5f,
            startY + cellSize.y * 0.5f,
            startZ + cellSize.z * 0.5f
        );

        worldCenter = transform.TransformPoint(localCenter);
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawGrid || !TryGetPlanningSnapshot(out _, out _) || occupied == null)
            return;

        for (int x = 0; x < countX; x++)
        {
            for (int y = 0; y < countY; y++)
            {
                for (int z = 0; z < countZ; z++)
                {
                    GetCell(x, y, z, out Vector3 center, out Vector3 size);

                    Gizmos.color = occupied[x, y, z]
                        ? OccupiedArea
                        : FreeArea;

                    Gizmos.matrix = Matrix4x4.TRS(
                        center,
                        transform.rotation,
                        Vector3.one
                    );

                    Vector3 scale = transform.lossyScale;
                    Gizmos.DrawCube(Vector3.zero, Vector3.Scale(size, new Vector3(Mathf.Abs(scale.x),Mathf.Abs(scale.y),Mathf.Abs(scale.z))) * gizmoCellScale);
                }
            }
        }

        Gizmos.matrix = Matrix4x4.identity;
    }
}

