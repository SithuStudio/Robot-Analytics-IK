using UnityEngine;
using UnityEngine.SceneManagement;
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
using UnityEngine.InputSystem;
#endif

namespace RobotIK
{
    /// <summary>
    /// Game-view navigation, automatically attached to Camera.main in Play mode.
    /// RMB: orbit; MMB or Shift+RMB: pan; wheel: zoom; RMB+WASD/QE: move;
    /// F: frame the robot; Home: restore the original view. Pointer gestures must
    /// start outside the robot UI. Mouse distances are pixels, not frame-scaled axes.
    /// </summary>
    [DisallowMultipleComponent, RequireComponent(typeof(Camera))]
    public sealed class RobotCameraController : MonoBehaviour
    {
        [SerializeField] private Transform orbitTarget;
        [SerializeField, Min(0.01f)] private float orbitDegreesPerPixel = 0.2f;
        [SerializeField, Min(0.01f)] private float panSensitivity = 1f;
        [SerializeField, Min(0.01f)] private float zoomSensitivity = 0.15f;
        [Tooltip("Limit accumulated wheel input in a single frame to prevent sudden zoom jumps.")]
        [SerializeField, Range(0.1f, 3f)] private float maximumWheelStepsPerFrame = 1f;
        [SerializeField, Min(0.01f)] private float moveSpeedMetersPerSecond = 1.5f;
        [SerializeField, Min(1f)] private float fastMoveMultiplier = 3f;
        [SerializeField, Min(0.01f)] private float minimumDistance = 0.15f;
        [SerializeField, Min(0.1f)] private float maximumDistance = 100f;
        [SerializeField, Min(0.1f)] private float defaultFocusDistance = 3f;
        private Camera view;
        private Vector3 pivot, initialPivot, initialPosition;
        private Quaternion initialRotation;
        private float initialOrthographicSize;
        private bool initialized, hasPointer, navigating;
        private Vector2 previousPointer;
        private readonly CameraWheelInputGuard wheelInput = new CameraWheelInputGuard();

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetRegistration() => SceneManager.sceneLoaded -= OnSceneLoaded;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            SceneManager.sceneLoaded -= OnSceneLoaded;
            SceneManager.sceneLoaded += OnSceneLoaded;
            InstallOnMainCamera();
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) => InstallOnMainCamera();
        private static void InstallOnMainCamera()
        {
            Camera main = Camera.main;
            if (main != null && main.GetComponent<RobotCameraController>() == null)
                main.gameObject.AddComponent<RobotCameraController>();
        }

        private void OnEnable()
        {
            Initialize();
            ResetNavigationInput();
        }

        private void OnDisable() => ResetNavigationInput();
        private void OnApplicationFocus(bool focus) => ResetNavigationInput();

        private void ResetNavigationInput()
        {
            hasPointer = navigating = false;
            wheelInput.Block(Time.unscaledTimeAsDouble);
        }

        private void Initialize()
        {
            if (initialized) return;
            view = GetComponent<Camera>();
            if (orbitTarget == null)
            {
                RobotBaseSixAxisRig rig = FindFirstObjectByType<RobotBaseSixAxisRig>();
                if (rig != null) orbitTarget = rig.transform;
            }
            pivot = orbitTarget != null ? TargetBounds().center : transform.position + transform.forward * defaultFocusDistance;
            initialPivot = pivot;
            initialPosition = transform.position;
            initialRotation = transform.rotation;
            initialOrthographicSize = view.orthographicSize;
            initialized = true;
        }

        private void LateUpdate()
        {
            if (!Application.isFocused || RobotRuntimeSceneEditor.IsManipulatingAny)
            { ResetNavigationInput(); return; }
            if (!ReadInput(out NavigationInput input)) { ResetNavigationInput(); return; }
            Vector2 delta = hasPointer ? input.Position - previousPointer : Vector2.zero;
            previousPointer = input.Position;
            hasPointer = true;
            if (!view.pixelRect.Contains(input.Position) || RobotMotionControls.IsPointerOverControls(input.Position))
            { navigating = false; wheelInput.Block(Time.unscaledTimeAsDouble); return; }
            if (input.OrbitDown || input.PanDown)
            {
                RobotMotionControls.ReleaseControlFocus();
                navigating = true;
                delta = Vector2.zero;
            }
            if (RobotMotionControls.IsEditingControls())
            { navigating = false; wheelInput.Block(Time.unscaledTimeAsDouble); return; }
            if (input.Reset) { ResetView(); wheelInput.Block(Time.unscaledTimeAsDouble); return; }
            if (input.Focus) { FocusTarget(); wheelInput.Block(Time.unscaledTimeAsDouble); return; }
            if (wheelInput.Accept(input.Scroll, Time.unscaledTimeAsDouble)) Zoom(input.Scroll);
            if (!input.OrbitHeld && !input.PanHeld) navigating = false;
            if (!navigating) return;
            if (input.PanHeld || input.Fast) Pan(delta);
            else if (input.OrbitHeld) Orbit(delta);
            if (input.OrbitHeld && input.Move.sqrMagnitude > 0f)
            {
                Vector3 direction = transform.right * input.Move.x + Vector3.up * input.Move.y + transform.forward * input.Move.z;
                Translate(direction.normalized * moveSpeedMetersPerSecond * (input.Fast ? fastMoveMultiplier : 1f) * Time.unscaledDeltaTime);
            }
        }

        public void Orbit(Vector2 pixelDelta)
        {
            Initialize();
            float currentPitch = Mathf.Asin(Mathf.Clamp(-transform.forward.y, -1f, 1f)) * Mathf.Rad2Deg;
            float pitch = Mathf.Clamp(currentPitch - pixelDelta.y * orbitDegreesPerPixel, -85f, 85f);
            Quaternion yaw = Quaternion.AngleAxis(pixelDelta.x * orbitDegreesPerPixel, Vector3.up);
            Quaternion rotation = Quaternion.AngleAxis(pitch - currentPitch, yaw * transform.right) * yaw;
            transform.SetPositionAndRotation(pivot + rotation * (transform.position - pivot), rotation * transform.rotation);
        }

        public void Pan(Vector2 pixelDelta)
        {
            Initialize();
            float scale = view.orthographic ? 2f * view.orthographicSize :
                2f * Vector3.Distance(transform.position, pivot) * Mathf.Tan(view.fieldOfView * 0.5f * Mathf.Deg2Rad);
            scale /= Mathf.Max(1, view.pixelHeight);
            Translate((-transform.right * pixelDelta.x - transform.up * pixelDelta.y) * scale * panSensitivity);
        }

        public void Zoom(float wheelSteps)
        {
            Initialize();
            float steps = CameraWheelInputGuard.ClampSteps(wheelSteps, maximumWheelStepsPerFrame);
            if (steps == 0f) return;
            float factor = Mathf.Exp(-steps * zoomSensitivity);
            float near = Mathf.Max(0.01f, minimumDistance);
            float far = Mathf.Max(near, maximumDistance);
            if (view.orthographic) view.orthographicSize = Mathf.Clamp(view.orthographicSize * factor, near, far);
            else
            {
                Vector3 offset = transform.position - pivot;
                float distance = Mathf.Clamp(offset.magnitude * factor, near, far);
                transform.position = pivot + (offset.sqrMagnitude > 0.000001f ? offset.normalized : -transform.forward) * distance;
            }
        }

        public void Translate(Vector3 worldOffset)
        {
            Initialize();
            transform.position += worldOffset;
            pivot += worldOffset;
        }

        public void ResetView()
        {
            Initialize();
            pivot = initialPivot;
            transform.SetPositionAndRotation(initialPosition, initialRotation);
            view.orthographicSize = initialOrthographicSize;
        }

        public bool FocusTarget()
        {
            Initialize();
            if (orbitTarget == null) return false;
            Bounds bounds = TargetBounds();
            pivot = bounds.center;
            float radius = Mathf.Max(0.05f, bounds.extents.magnitude);
            float halfVertical = Mathf.Max(1f, view.fieldOfView) * Mathf.Deg2Rad * 0.5f;
            float halfHorizontal = Mathf.Atan(Mathf.Tan(halfVertical) * Mathf.Max(0.01f, view.aspect));
            float distance = radius * 1.2f / Mathf.Sin(Mathf.Min(halfVertical, halfHorizontal));
            distance = Mathf.Clamp(distance, Mathf.Max(0.01f, minimumDistance), Mathf.Max(minimumDistance, maximumDistance));
            transform.position = pivot - transform.forward * distance;
            if (view.orthographic) view.orthographicSize = Mathf.Clamp(radius * 1.2f / Mathf.Min(1f, Mathf.Max(0.01f, view.aspect)), minimumDistance, maximumDistance);
            return true;
        }

        private Bounds TargetBounds()
        {
            var bounds = new Bounds(orbitTarget.position, Vector3.one * 0.1f);
            bool found = false;
            foreach (Renderer renderer in orbitTarget.GetComponentsInChildren<Renderer>())
            {
                if (!renderer.enabled || renderer is LineRenderer || renderer is TrailRenderer || renderer.GetComponent<TextMesh>() != null) continue;
                if (!found) { bounds = renderer.bounds; found = true; }
                else bounds.Encapsulate(renderer.bounds);
            }
            return bounds;
        }

        private struct NavigationInput
        {
            public Vector2 Position;
            public Vector3 Move;
            public float Scroll;
            public bool OrbitDown, PanDown, OrbitHeld, PanHeld, Fast, Focus, Reset;
        }

        private static bool ReadInput(out NavigationInput input)
        {
            input = default;
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
            Mouse mouse = Mouse.current;
            if (mouse == null) return false;
            input.Position = mouse.position.ReadValue();
            input.Scroll = mouse.scroll.ReadValue().y / 120f;
            input.OrbitDown = mouse.rightButton.wasPressedThisFrame;
            input.PanDown = mouse.middleButton.wasPressedThisFrame;
            input.OrbitHeld = mouse.rightButton.isPressed;
            input.PanHeld = mouse.middleButton.isPressed;
            Keyboard keys = Keyboard.current;
            if (keys != null)
            {
                input.Fast = keys.leftShiftKey.isPressed || keys.rightShiftKey.isPressed;
                input.Focus = keys.fKey.wasPressedThisFrame;
                input.Reset = keys.homeKey.wasPressedThisFrame;
                input.Move = new Vector3((keys.dKey.isPressed ? 1 : 0) - (keys.aKey.isPressed ? 1 : 0),
                    (keys.eKey.isPressed ? 1 : 0) - (keys.qKey.isPressed ? 1 : 0),
                    (keys.wKey.isPressed ? 1 : 0) - (keys.sKey.isPressed ? 1 : 0));
            }
#else
            if (!Input.mousePresent) return false;
            input.Position = Input.mousePosition;
            input.Scroll = Input.mouseScrollDelta.y;
            input.OrbitDown = Input.GetMouseButtonDown(1); input.PanDown = Input.GetMouseButtonDown(2);
            input.OrbitHeld = Input.GetMouseButton(1); input.PanHeld = Input.GetMouseButton(2);
            input.Fast = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
            input.Focus = Input.GetKeyDown(KeyCode.F); input.Reset = Input.GetKeyDown(KeyCode.Home);
            input.Move = new Vector3((Input.GetKey(KeyCode.D) ? 1 : 0) - (Input.GetKey(KeyCode.A) ? 1 : 0),
                (Input.GetKey(KeyCode.E) ? 1 : 0) - (Input.GetKey(KeyCode.Q) ? 1 : 0),
                (Input.GetKey(KeyCode.W) ? 1 : 0) - (Input.GetKey(KeyCode.S) ? 1 : 0));
#endif
            return true;
        }
    }

    // A wheel gesture that started over UI must become quiet before the scene
    // can claim it. Merely moving the pointer off the panel cannot transfer its
    // remaining wheel/trackpad momentum to the camera.
    internal sealed class CameraWheelInputGuard
    {
        private const double QuietSeconds = 0.15;
        private double blockedUntil;
        private bool ready;

        public void Block(double now)
        {
            ready = false;
            blockedUntil = now + QuietSeconds;
        }

        public bool Accept(float steps, double now)
        {
            if (float.IsNaN(steps) || float.IsInfinity(steps)) { Block(now); return false; }
            bool scrolling = System.Math.Abs(steps) > 0.0001f;
            if (!ready)
            {
                if (scrolling) blockedUntil = now + QuietSeconds;
                else if (now >= blockedUntil) ready = true;
                return false;
            }
            return scrolling;
        }

        public static float ClampSteps(float steps, float maximum)
        {
            if (float.IsNaN(steps) || float.IsInfinity(steps)) return 0f;
            if (float.IsNaN(maximum) || float.IsInfinity(maximum)) maximum = 1f;
            float limit = System.Math.Max(0.1f, System.Math.Min(3f, maximum));
            return System.Math.Max(-limit, System.Math.Min(limit, steps));
        }
    }
}
