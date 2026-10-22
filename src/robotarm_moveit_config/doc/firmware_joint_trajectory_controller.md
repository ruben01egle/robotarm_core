# Firmware patch: joint_trajectory_controller for MoveIt

This file contains a patch for **robotarm_firmware_lnx** (the RT PC repository). It adds the
`joint_trajectory_controller` that executes the trajectories planned by MoveIt. Without it, MoveIt
can plan for the real robot (and the firmware mock or MuJoCo sim) but cannot execute anything.

It is stored here because the change could not be pushed to the firmware repository (no write
access). Once it has been applied there, this file can be deleted.

| | |
|---|---|
| Target repository | `robotarm_firmware_lnx` |
| Base commit | `705ac5d` ("added mujoco position and velocity interface ...", firmware `main` on 2026-10-22) |
| Tested | Applied to `705ac5d`; built; tested end to end with the firmware mock and the MuJoCo sim, control_center and MoveIt (see the README of `robotarm_moveit_config`, section 9) |

## What the patch changes

| File | Change |
|---|---|
| `src/robotarm_bringup/config/controllers.yaml` | Registers `joint_trajectory_controller`: joints `axis1..axis6`, command interfaces `position` + `velocity` (PVC mode of `moteus_interface`, the same interfaces as joint stream and Cartesian jog), tracking tolerances that abort a goal if a joint deviates more than 0.1 rad while moving or 0.01 rad at the goal (PRELIMINARY, tune on hardware) |
| `src/robotarm_bringup/launch/controllers.launch.py` | Adds it to `INACTIVE_CONTROLLERS`, so hardware, mock and MuJoCo sim load it inactive like the other motion controllers; control_center activates it for a mission (`RequestAction`, `ACTION_MISSION`, `controller_names: [joint_trajectory_controller]`) |
| `src/robotarm_bringup/package.xml` | `exec_depend` on `joint_trajectory_controller` |
| `README.md` | Documents the new controller |

The controller name and joints must match `config/moveit_controllers.yaml` in
`robotarm_moveit_config`.

## How to apply

In a clone of `robotarm_firmware_lnx` at `705ac5d` or later (it still applies if the four files
did not change in the meantime):

```bash
cd <robotarm_firmware_lnx>
git checkout -b feature/joint-trajectory-controller
# extract the patch from this file (everything between the ````diff fences)
sed -n '/^````diff$/,/^````$/{/^````/d;p}' \
  <robotarm_core>/src/robotarm_moveit_config/doc/firmware_joint_trajectory_controller.md > jtc.patch
git apply --check jtc.patch && git apply jtc.patch && rm jtc.patch
git add -A && git commit -m "Add joint_trajectory_controller for MoveIt"
colcon build --packages-select robotarm_bringup
```

If `git apply --check` reports a conflict, the base has changed too much. Then make the four
changes from the table above by hand; the patch below shows the exact lines.

Quick check after building:

```bash
ros2 launch robotarm_bringup robotarm_mock.launch.py
ros2 control list_controllers   # joint_trajectory_controller ... inactive
```

## Patch

````diff
diff --git a/README.md b/README.md
index 2af3e48..8949ac6 100644
--- a/README.md
+++ b/README.md
@@ -150,10 +150,13 @@ graph TD
 - `joint_state_broadcaster` (stock `ros2_controllers` package) — publishes
   `/joint_states` at a configurable rate.
 
-There is currently no trajectory-following controller (e.g. a standard
-`joint_trajectory_controller`) wired up in this repo — motion today is driven
-through the reactive `JointStreamController` and `CartesianJogController` only. More controller options are
-expected as the project develops.
+- `joint_trajectory_controller` (stock `ros2_controllers` package) — executes
+  joint trajectories received on the `~/follow_joint_trajectory` action
+  (`control_msgs/action/FollowJointTrajectory`), e.g. planned by MoveIt on the
+  high-level PC. Outputs `position`+`velocity` like `joint_stream_controller`
+  and `cartesian_jog_controller`, so only one of the three can be active. Aborts
+  a goal if the tracking error exceeds the `constraints` in `controllers.yaml`.
+  Activated by the high-level PC (control_center) for a mission, like the others.
 
 ## Usage / Getting Started
 
@@ -260,10 +263,10 @@ carries several honest rough edges:
   (official mjbots fdcanusb only, no SocketCAN), no retry/checksum logic, and a
   project-specific secondary-encoder homing workaround. Details and caveats are
   in its own README.
-- **Only reactive controllers exist today** (`JointStreamController`,
-  `CartesianJogController`); there's no trajectory-following controller in this
-  repo yet, and more controller options
-  are planned as the project develops.
+- **Motion controllers today**: the reactive `JointStreamController` and
+  `CartesianJogController`, the Cartesian impedance / gravity compensation
+  controllers, and the stock `joint_trajectory_controller` for MoveIt
+  trajectories; more controller options are planned as the project develops.
 - **Joint count is a current-state fact, not a design limit.** The arm has 6
   configured joints today; `moteus_interface`, `robotarm_controllers`, and
   `robotarm_bringup` are all written to be driven by whatever's declared in the
diff --git a/src/robotarm_bringup/config/controllers.yaml b/src/robotarm_bringup/config/controllers.yaml
index 9970457..afaf7d4 100644
--- a/src/robotarm_bringup/config/controllers.yaml
+++ b/src/robotarm_bringup/config/controllers.yaml
@@ -78,6 +78,11 @@ controller_manager:
     cartesian_impedance_controller:
       type: robotarm_controllers/CartesianImpedanceController
 
+    # Executes trajectories planned by MoveIt on the high-level PC (FollowJointTrajectory action
+    # ~/follow_joint_trajectory). Spawned inactive; activated by control_center for a mission.
+    joint_trajectory_controller:
+      type: joint_trajectory_controller/JointTrajectoryController
+
 moteus_hardware_system:
   ros__parameters:
     execution_mode: "pipelined"  # pipelined or strict_sequential
@@ -243,3 +248,40 @@ cartesian_impedance_controller:
     cartesian_damping_ratio: 0.7    # unitless, NOT TUNED
     max_spring_force: 20.0          # N, norm of the translational spring force
     max_spring_moment: 2.0          # Nm, norm of the rotational spring moment, below the 4 Nm of the wrist motors
+
+# Joint names, controller name and action namespace must match moveit_controllers.yaml in
+# robotarm_core/src/robotarm_moveit_config.
+joint_trajectory_controller:
+  ros__parameters:
+    joints:
+      - axis1
+      - axis2
+      - axis3
+      - axis4
+      - axis5
+      - axis6
+    # position + velocity: interpolated position plus velocity feedforward, the PVC mode of
+    # moteus_interface. Same interfaces as joint_stream_controller and cartesian_jog_controller,
+    # so only one of them can be active.
+    command_interfaces:
+      - position
+      - velocity
+    state_interfaces:
+      - position
+      - velocity
+    allow_partial_joints_goal: false
+    allow_nonzero_velocity_at_trajectory_end: false
+    interpolation_method: splines
+    # Safety net against a robot that does not follow the trajectory (blocked joint, collision,
+    # too aggressive limits): the goal is aborted and the controller holds the current position
+    # if the tracking error of any joint exceeds "trajectory" [rad] while moving, or "goal" [rad]
+    # after goal_time [s] at the end. PRELIMINARY values, tune on hardware.
+    constraints:
+      stopped_velocity_tolerance: 0.05  # [rad/s] robot counts as stopped below this
+      goal_time: 0.5
+      axis1: {trajectory: 0.1, goal: 0.01}
+      axis2: {trajectory: 0.1, goal: 0.01}
+      axis3: {trajectory: 0.1, goal: 0.01}
+      axis4: {trajectory: 0.1, goal: 0.01}
+      axis5: {trajectory: 0.1, goal: 0.01}
+      axis6: {trajectory: 0.1, goal: 0.01}
diff --git a/src/robotarm_bringup/launch/controllers.launch.py b/src/robotarm_bringup/launch/controllers.launch.py
index bd7f9f5..240abdf 100644
--- a/src/robotarm_bringup/launch/controllers.launch.py
+++ b/src/robotarm_bringup/launch/controllers.launch.py
@@ -16,6 +16,8 @@ INACTIVE_CONTROLLERS = [
     # upstream controllers chained to cartesian_impedance_controller (chain_to in controllers.yaml)
     'joint_stream_controller_impedance',
     'cartesian_jog_controller_impedance',
+    # executes trajectories planned by MoveIt on the high-level PC, activated by control_center
+    'joint_trajectory_controller',
 ]
 
 
diff --git a/src/robotarm_bringup/package.xml b/src/robotarm_bringup/package.xml
index 3ee8baf..ea74542 100644
--- a/src/robotarm_bringup/package.xml
+++ b/src/robotarm_bringup/package.xml
@@ -12,6 +12,7 @@
   <depend>rviz2</depend>
   
   <exec_depend>joint_state_broadcaster</exec_depend>
+  <exec_depend>joint_trajectory_controller</exec_depend>
   <exec_depend>moteus_interface</exec_depend>
   <exec_depend>robotarm_controllers</exec_depend>
   <exec_depend>robotarm_description</exec_depend>
````
