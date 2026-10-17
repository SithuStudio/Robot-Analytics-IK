# Seven-axis robot IK

This folder contains a six- or seven-revolute-joint Unity rig builder and a pose IK solver.
It is configured with **standard Denavit–Hartenberg (DH)** rows and solved with damped
least squares.

## Bind the imported Robot Base hierarchy

`RobotBaseSixAxisRig` is the starting rig for the imported `Robot Base` object with nested
`J1` through `J6` transforms. Add it to `Robot Base`, then use **Bind J1-J6 Under Robot Base**
from its component menu. It finds the six named pivots and keeps their imported positions
and child mesh offsets. The configured HH020 local axes are `Y, Z, Z, X, Z, X` for J1 through
J6 and the joint limits match the official robot description.

Keep the reference joint frames intact: J1 through J5 use identity local rotation and J6 uses
its imported `-90°` X mesh-frame rotation. Pose the robot with `SetJointAngles` or the rig
controller, not by editing a joint Transform's local rotation. Direct rotation changes are
captured as a fixed frame when the rig starts and can make J4/J5/J6 fail the spherical-wrist
calibration.

This binding component does not move the pivots or infer link dimensions. An analytical
inverse solver must use the measured pivot geometry, axis directions, zero offsets, and TCP
frame. Do not use the older `Robot7AxisRig` DH table to drive the imported model directly;
that component positions its joint transforms from the table.

## Analytical IK on Robot Base

`RobotBaseSixAxisAnalyticIK` is attached to the imported `Robot Base` prefab alongside
`RobotBaseSixAxisRig`. It reads the actual hierarchy at the zero joint coordinates and
calibrates the shoulder/elbow link vectors, wrist-axis intersection, and tool offset. It
then enumerates the shoulder, elbow, and wrist branches analytically, applies the configured
angle limits, and chooses the solution nearest the current joint state. The API is
`SolveNow(position, rotation, out solution)`; assign a target and enable **Solve Continuously**
for tracking, or use **Solve Current Target Now** from the component menu.

With **Align TCP Z To Target Z** enabled, the TCP's local +Z axis follows the Target's local
+Z (the blue Transform arrow). **Track Target Roll** is enabled by default, so rotating the
Target around that blue axis is passed through to J6. Disable it when only the Z direction
matters; the solver then uses world up to choose the remaining roll. Disable both options when
your controller supplies a complete TCP rotation through `SolveNow`.
The scene's `T1` TCP frame is rotated `Y = -90°` below the imported flange so its local +Z
axis points outward from the physical tool.

## Plan a collision-free move from Start to End Target

`RobotBaseSixAxisMotionPlanner` is separate from the analytic IK component. Its **Planner
Algorithm** dropdown offers Fast RRT-Connect, Optimized RRT-Connect, Cartesian Arc, and 3D
Grid A*. The RRT options search joint space. Cartesian Arc samples smooth TCP-space curves from the current
TCP pose through Start to End Target, solves each sample with analytic IK while favoring the
nearest collision-free IK branch, and checks the complete robot colliders between samples.
Cartesian Arc tries straight and progressively wider arcs in world-up and lateral directions;
the chosen motion is an arc, not an arbitrary user-drawn curve. By default, the tool keeps its
current orientation; enable **Use Marker Orientations** when marker rotations should also be
reached. The planner does not replace or modify the IK solver.

To force a specific approach or extra repositioning motion, add Transform references to
**Intermediate Waypoints** in the planner Inspector, in visit order. Place each Transform at
the desired TCP position; its rotation is used when **Use Marker Orientations** is enabled.
All planner algorithms collision-check each segment through these poses before reaching End
Target. Leave the array empty for direct Start-to-End planning.

The **Wrist Motion Penalty** weights J4-J6 changes when choosing IK branches and searching/ranking
joint-space routes. The **TCP Rotation Motion Penalty** adds the measured TCP orientation travel
along candidate joint-space segments to the route cost; increase it to favor steadier tool
orientation. These settings can trade wrist movement for larger shoulder or elbow movement, and
cannot remove rotation required by the requested TCP poses. Cartesian Arc controls the TCP
orientation directly along each segment when that constraint is needed.

**3D Grid A*** builds a Cartesian occupancy grid from obstacle collider bounds and searches
for a route with 26-connected 3D A*. **Grid Cell Size Millimeters** sets voxel resolution,
**Grid Obstacle Inflation Millimeters** expands obstacle bounds for the initial TCP route search,
and **Maximum Grid Nodes** limits expansions per segment. Each resulting waypoint is solved
with analytic IK, then full robot collider and joint-transition checks validate the route. A grid
route is rejected if those detailed checks fail. The grid uses conservative world-aligned bounds,
so rotated obstacle shapes can appear larger in the grid than they are visually.
**Maximum Grid Route Attempts** controls how many distinct A* routes are tried for a segment;
after an IK or collision rejection, the planner blocks a cell on the failed route and searches
for another candidate.
During Play mode, the 3D grid occupancy search runs on a worker thread using a copied snapshot
of obstacle bounds. IK and full robot-collider validation still run on Unity's main thread.
The Inspector's synchronous **Plan (Do Not Move)** action continues to use the main-thread
planning path.
The **3D Grid A* Gizmos** options draw occupied inflated voxels and the candidate TCP route in
the Scene view. A failed route leaves a red cell and a status location at the collision sample
or the closest explored grid cell when the search could not reach its goal.

For faster planning, select **Fast RRT-Connect** to stop at the first safe route. **Optimized RRT-Connect** continues searching after its first connection and compares candidates by sampled TCP translation distance. This prioritizes the length of the visible TCP route instead of reducing waypoint count or adding wrist/TCP rotation penalties to the distance objective. **Optimized RRT Waypoint Budget** defaults to 5 points including Start and Goal; the simplifier chooses the shortest safe shortcut route within that budget and keeps extra points when needed to preserve collision clearance. Collision checks use the configured resolution.
In Play mode, RRT tree search and route ranking run on a worker thread. Each swept-segment collision request is serviced on Unity's main thread, with a small per-frame request budget. Repeated joint configurations and TCP segment costs reuse cached results during a plan. The shortcut optimizer considers at most 20 evenly sampled RRT path states before selecting its point-budget route; if those shortcuts cannot connect safely, it falls back to the full collision-checked path. The Inspector's synchronous **Plan (Do Not Move)** action remains synchronous.

**TCP Speed Millimeters Per Second** controls translation playback speed along the sampled TCP
path. **TCP Rotation Speed Degrees Per Second** limits orientation changes, including moves where
the TCP rotates with little translation. Both limits affect playback time, not planning time. The
Inspector's read-only **Estimated Travel Time (s)** is computed from the saved path using those
speed limits and refreshes whenever a plan is created or replayed.

1. Add `RobotBaseSixAxisMotionPlanner` to the same `Robot Base` object as the rig and analytic
   IK components.
2. Assign the scene's `Start`, `End Target`, and `Obstacles` transforms in the planner.
   Both target transforms specify the full TCP pose, including rotation.
3. Add primitive colliders (box, sphere, or capsule) to each robot link and colliders to every
   obstacle. Robot colliders beneath `Robot Base` and obstacle colliders beneath `Obstacles`
   are found automatically. If the robot has no colliders, **Generate Robot Box Colliders If
   Missing** creates conservative boxes from its mesh bounds at runtime. Assign **Robot
   Colliders** explicitly to exclude geometry that should not participate. Trigger colliders
   are ignored. Obstacles need actual colliders.
4. In the test scene, planning starts automatically when you enter Play mode. In other scenes,
   enable **Plan On Play**, choose **Plan Collision-Free Motion Start to End** from the
   component menu, or call `PlanAndMove()` from another script. Disable **Play On Plan** to
   plan without automatically executing. `TryPlan()` is also available to inspect
   `PlannedPath`.

The test scene draws the planned TCP route as a cyan Gizmos line in the Scene view (and a
LineRenderer in Game view). The planner searches over multiple frames, shows its current result
in the component Inspector, and disables continuous IK control when it takes over so the
analytic solver cannot overwrite the trajectory. Successful and failed RRT plans report the
number of search iterations; Cartesian Arc reports candidate arcs attempted; Grid A* reports
expanded grid nodes. In Play mode,
**Replay Planned Motion Without Replanning** returns to the saved first joint state and replays
the stored path without searching again. The Inspector also exposes read-only **Planning
Iterations**, **Cartesian Arc Attempts**, and **Planning Time (s)** values alongside estimated
travel time.

The planner checks the full robot collider set against the obstacle colliders at interpolated
joint configurations. **Check Self Collision** optionally checks non-adjacent robot links too.
Tune **Collision Resolution Degrees** for finer checks and **Maximum Iterations** if a narrow
passage needs more search. The planner reports failure when an endpoint is unreachable or in
collision, obstacle collision geometry is missing, or no path is found. Collision checking is discrete;
use a small resolution and conservative collider shapes for narrow clearances. Replan if an
obstacle moves before execution.

**Robot Box Collider Scale** applies to BoxColliders in **Robot Colliders**, including generated
fallback boxes. A value of 1 preserves each collider's authored size; a slightly smaller value
trims boxes that are overly conservative. Reduce it gradually because a collider smaller than the
visible mesh may allow the mesh to approach an obstacle more closely than the collision check
indicates. During Play mode, changing the setting applies it immediately, using each collider's
original size so replanning does not shrink it repeatedly.

### Choose an analytical robot posture

On `RobotBaseSixAxisAnalyticIK`, use **Analytical Branch Selection** to choose the posture:

- **Shoulder Posture**: Automatic, Left, or Right.
- **Elbow Posture**: Automatic, Up, or Down. Up is the positive elbow-plane branch; Down is
  the negative branch.
- **Wrist Posture**: Automatic, Non-Flip, or Flip.

Automatic considers every valid branch and then selects the one nearest to the current joint
state. A forced posture returns no solution when that branch violates a joint limit or cannot
reach the Target.

The solver checks its required geometry at calibration: J1 is perpendicular to parallel J2/J3,
and J4/J5/J6 form a concurrent spherical wrist. If the imported pivots or local axes do not
meet those conditions, calibration reports which condition failed instead of silently returning
incorrect joint values. The arm's serialized axes and limits are based on the official
[HD Hyundai HH020 robot description](https://github.com/hyundai-robotics/hdr_description/blob/jazzy/urdf/robots/hh020/hh020.urdf.xacro).

## Set up the robot

1. Add `Robot7AxisRig` to the robot base. Add `Robot7AxisIK` to the same object. The
   `RobotH020` test scene already has both components attached and bound to its nested
   `J1` through `J6` transforms.
2. In the rig inspector, set six or seven DH rows in joint order. Enter lengths `a` and
   `d` in millimeters. The default **Unity Units per Millimeter** value is `0.001`, for
   scenes where 1 Unity unit represents 1 meter. Set it to `1` if the imported robot model
   is scaled so 1 Unity unit represents 1 millimeter. `alpha` and `thetaOffset` are degrees.
   Set each joint's minimum and maximum angle in degrees.
3. Optionally assign a link mesh prefab to each row. The builder places it under the
   corresponding DH link frame. Orient/position each mesh prefab around that frame's
   local origin before assigning it.
4. Existing nested joints named `J1`, `J2`, etc. are bound automatically. For an empty
   robot root, choose **Build or Bind DH Rig** from the component menu. This creates one
   pivot GameObject per DH row, fixed link frames, and a tool frame under the robot base.
5. Create a target GameObject and assign its Transform in `Robot7AxisIK`. With
   `solveContinuously` enabled, the arm tracks its position and rotation. You can disable
   continuous solving and call `SolveNow()` or `SolveStep(position, rotation)` from your
   own controller.

The rig exposes `EvaluateForwardKinematics(float[] anglesDegrees)` for a pose calculation
without moving the model, `CurrentPose` for the currently applied TCP pose, and
`SetJointAngles(float[])` to apply a joint vector with configured joint limits. The IK
component uses a damped least-squares Jacobian iteration and also clamps each step to the
configured joint limits. Assign a real TCP transform if the imported model has one, or
set the TCP position/rotation offsets in the rig in millimeters/degrees.

The scene uses Unity's **Y-up** convention, so each pivot rotates around its local **+Y**
axis. The standard DH transform is mapped into Unity coordinates as
`Ry(theta) Ty(d) Tx(a) Rx(alpha)`, where `theta` is the solved joint angle plus
`thetaOffset`. `d` is applied along local Y and `a` along local X. Ensure your imported
model's link geometry is aligned to these frames; robot CAD exports often need a per-mesh
rotation/offset.

## Scene view Gizmos

With **Show Gizmos** enabled on `Robot7AxisRig`, the Scene view shows a sphere and local
Y rotation axis at each joint, plus a line between neighboring joint frames. **Show Gizmo Labels**
adds each joint's position relative to the base and each measured link length in millimeters. Assign the
tool/end-effector Transform to **Tool Frame** to also display the final joint-to-tool link.
The rig computes the TCP from the DH table when no tool transform is assigned. Existing
nested hierarchy names `J1`, `J2`, etc. are automatically bound when the rig has matching
joint names.

This is a generic system, not a calibrated Hyundai HH020 model. HH020 is six-axis, so use
six DH rows for it. The solver can also use seven rows for a seven-axis robot.
