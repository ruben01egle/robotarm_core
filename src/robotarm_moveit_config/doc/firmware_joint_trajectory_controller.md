# Firmware patch: joint_trajectory_controller for MoveIt

This file contains a patch for **robotarm_firmware_lnx** (the RT PC repository). It adds the
`joint_trajectory_controller` that executes the trajectories planned by MoveIt. Without it, MoveIt
can plan for the real robot (and the firmware mock) but cannot execute anything.

It is stored here because the change could not be pushed to the firmware repository (no write
access). Once it has been applied there, this file can be deleted.

| | |
|---|---|
| Target repository | `robotarm_firmware_lnx` |
| Base commit | `e5bfe6a` ("test updated, cleanup and readme", firmware `main` on 2026-10-10) |
| Tested | Applied to `e5bfe6a`; built; tested end to end with `robotarm_mock.launch.py`, control_center and MoveIt (see the README of `robotarm_moveit_config`, section 9) |

## What the patch changes

| File | Change |
|---|---|
| `src/robotarm_bringup/config/controllers.yaml` | Registers `joint_trajectory_controller`: joints `axis1..axis6`, command interfaces `position` + `velocity` (PVC mode of `moteus_interface`, like the jog controllers), tracking tolerances that abort a goal if a joint deviates more than 0.1 rad while moving or 0.01 rad at the goal (PRELIMINARY, tune on hardware) |
| `src/robotarm_bringup/launch/robotarm.launch.py` | Spawns it `--inactive`; control_center activates it for a mission (`RequestAction`, `ACTION_MISSION`, `controller_names: joint_trajectory_controller`) |
| `src/robotarm_bringup/package.xml` | `exec_depend` on `joint_trajectory_controller` |
| `README.md` | Documents the new controller |

The controller name and joints must match `config/moveit_controllers.yaml` in
`robotarm_moveit_config`.

## How to apply

In a clone of `robotarm_firmware_lnx` at `e5bfe6a` or later (it still applies if the four files
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
index 2ab02c4..8dee029 100644
--- a/README.md
+++ b/README.md
@@ -27,8 +27,7 @@ This repo is for anyone building or maintaining the actual hardware-interfacing
 firmware: the `ros2_control` hardware plugin, the motion controllers, and the
 launch/config that brings them up on the real-time PC.
 
-**Current state:** the arm currently has 3 configured joints (`axis1`/`axis2`/
-`axis3`), but nothing here is hardcoded to that number — `moteus_interface`,
+**Current state:** the arm has 6 configured joints (`axis1` … `axis6`), but nothing here is hardcoded to that number — `moteus_interface`,
 `robotarm_controllers`, and `robotarm_bringup` are all written generically
 against whatever joints are declared in the URDF and config. Adding an axis is
 meant to be a matter of adding a joint to `robotarm_description`'s URDF/Xacro and
@@ -148,10 +147,16 @@ graph TD
 - `joint_state_broadcaster` (stock `ros2_controllers` package) — publishes
   `/joint_states` at a configurable rate.
 
-There is currently no trajectory-following controller (e.g. a standard
-`joint_trajectory_controller`) wired up in this repo — motion today is driven
-through the reactive `TeleopController` and `CartesianJogController` only. More controller options are
-expected as the project develops.
+- `joint_trajectory_controller` (stock `ros2_controllers` package) — executes
+  joint trajectories received on the `~/follow_joint_trajectory` action
+  (`control_msgs/action/FollowJointTrajectory`), e.g. planned by MoveIt on the
+  high-level PC. Claims `position`+`velocity` like `TeleopController` and
+  `CartesianJogController`, so only one of the three can be active. Aborts a
+  goal if the tracking error exceeds the `constraints` in `controllers.yaml`.
+
+`TeleopController`, `CartesianJogController` and `joint_trajectory_controller`
+are spawned inactive and are activated by the high-level PC (control_center)
+when a mission starts.
 
 ## Usage / Getting Started
 
@@ -204,8 +209,8 @@ ros2 launch robotarm_bringup robotarm_hardware.launch.py
 
 Both are thin wrappers around `robotarm.launch.py` (only `use_hardware`
 differs) and start `ros2_control_node`, `robot_state_publisher`, and spawn
-`joint_state_broadcaster`, `teleop_controller` and `cartesian_jog_controller`
-(the latter two inactive — activate one of them explicitly once you're ready to
+`joint_state_broadcaster`, `teleop_controller`, `cartesian_jog_controller` and
+`joint_trajectory_controller` (the latter three inactive — activate one of them explicitly once you're ready to
 send motion commands, e.g.
 `ros2 control switch_controllers --activate cartesian_jog_controller`). The hardware
 component (real or mock) boots disarmed and has to be activated
@@ -249,10 +254,10 @@ carries several honest rough edges:
   (official mjbots fdcanusb only, no SocketCAN), no retry/checksum logic, and a
   project-specific secondary-encoder homing workaround. Details and caveats are
   in its own README.
-- **Only reactive controllers exist today** (`TeleopController`,
-  `CartesianJogController`); there's no trajectory-following controller in this
-  repo yet, and more controller options
-  are planned as the project develops.
+- **Three motion controllers exist today**: `TeleopController` and
+  `CartesianJogController` (reactive) and the stock `joint_trajectory_controller`
+  (MoveIt trajectories); more controller options are planned as the project
+  develops.
 - **Joint count is a current-state fact, not a design limit.** The arm has 6
   configured joints today; `moteus_interface`, `robotarm_controllers`, and
   `robotarm_bringup` are all written to be driven by whatever's declared in the
diff --git a/src/robotarm_bringup/config/controllers.yaml b/src/robotarm_bringup/config/controllers.yaml
index fcc518b..ba98e8e 100644
--- a/src/robotarm_bringup/config/controllers.yaml
+++ b/src/robotarm_bringup/config/controllers.yaml
@@ -69,6 +69,11 @@ controller_manager:
     cartesian_jog_controller:
       type: robotarm_controllers/CartesianJogController
 
+    # Executes trajectories planned by MoveIt on the high-level PC (FollowJointTrajectory action
+    # ~/follow_joint_trajectory). Spawned inactive; activated by control_center for a mission.
+    joint_trajectory_controller:
+      type: joint_trajectory_controller/JointTrajectoryController
+
 moteus_hardware_system:
   ros__parameters:
     execution_mode: "pipelined"  # pipelined or strict_sequential
@@ -125,3 +130,39 @@ cartesian_jog_controller:
 
     max_increment_linear: 0.003     # m, per axis clamp of one increment step
     max_increment_angular: 0.0524   # rad (3°), per axis clamp of one increment step
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
+    # position + velocity: interpolated position plus velocity feedforward, selects the PVC mode
+    # of moteus_interface (same as teleop_controller, so both cannot be active at the same time)
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
diff --git a/src/robotarm_bringup/launch/robotarm.launch.py b/src/robotarm_bringup/launch/robotarm.launch.py
index 2001219..9f6f233 100644
--- a/src/robotarm_bringup/launch/robotarm.launch.py
+++ b/src/robotarm_bringup/launch/robotarm.launch.py
@@ -77,10 +77,19 @@ def generate_launch_description():
         output='screen'
     )
 
+    # MoveIt trajectory execution; inactive until control_center activates it for a mission
+    joint_trajectory_controller_spawner = Node(
+        package='controller_manager',
+        executable='spawner',
+        arguments=['joint_trajectory_controller', '--inactive'],
+        output='screen'
+    )
+
     return LaunchDescription(declared_arguments + [
         control_node,
         robot_state_pub_node,
         joint_state_broadcaster_spawner,
         teleop_controller_spawner,
         cartesian_jog_controller_spawner,
+        joint_trajectory_controller_spawner,
     ])
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
