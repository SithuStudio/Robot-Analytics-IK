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
