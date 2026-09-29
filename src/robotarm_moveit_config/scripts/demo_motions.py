#!/usr/bin/env python3
"""Plan and execute test motions through move_group (verification of this package).

Runs against demo.launch.py (mock hardware) or any other running move_group:
  (a) OMPL:      plan + execute to the named state "ready"
  (b) Pilz PTP:  plan + execute back to "home"
  (c) Pilz LIN:  plan a straight TCP motion of +5 cm in z (Base_1 frame), starting at "ready".
                 Plan only by default (start state given explicitly, the robot does not move);
                 with --execute-lin the robot moves PTP to "ready", executes the LIN and returns
                 home.

Only plain rclpy and moveit_msgs are used (MoveGroup action /move_action, FK service /compute_fk),
so the script also shows what the RViz panel does under the hood.

Usage:
  ros2 run robotarm_moveit_config demo_motions.py [--scaling 0.5] [--execute-lin]
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import yaml

GROUP = 'arm'
BASE_FRAME = 'Base_1'
TCP_LINK = 'tcp'
OMPL = 'ompl'
PILZ = 'pilz_industrial_motion_planner'

# MoveItErrorCodes value -> name, for readable output
ERROR_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}


def load_named_states():
    """Read the group states of GROUP from the installed SRDF: {name: {joint: value}}."""
    srdf = os.path.join(
        get_package_share_directory('robotarm_moveit_config'), 'config', 'robotarm.srdf')
    states = {}
    for state in ET.parse(srdf).getroot().iter('group_state'):
        if state.get('group') == GROUP:
            states[state.get('name')] = {
                j.get('name'): float(j.get('value')) for j in state.iter('joint')}
    return states


def duration_s(duration):
    return duration.sec + duration.nanosec * 1e-9


class DemoMotions(Node):

    def __init__(self, scaling):
        super().__init__('robotarm_demo_motions')
        self.scaling = scaling
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        self.joint_state = None
        self.create_subscription(JointState, 'joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def wait_for_servers(self, timeout=30.0):
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('move_group action /move_action not available')
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('service /compute_fk not available')

    # ----- request building -------------------------------------------------------------------

    def request(self, pipeline, planner, goal, start_state=None):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = pipeline
        req.planner_id = planner          # '' = default planner of the pipeline
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.scaling
        req.max_acceleration_scaling_factor = self.scaling
        req.goal_constraints = [goal]
        # box in which the planner samples; empty = warning + default of +-1 m
        req.workspace_parameters.header.frame_id = BASE_FRAME
        req.workspace_parameters.min_corner.x = -1.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = -1.5
        req.workspace_parameters.max_corner.x = 1.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 1.5
        if start_state is None:
            req.start_state.is_diff = True  # start at the current robot state
        else:
            req.start_state = start_state
        return req

    @staticmethod
    def joint_goal(values):
        goal = Constraints()
        for name, value in values.items():
            goal.joint_constraints.append(JointConstraint(
                joint_name=name, position=value,
                tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0))
        return goal

    @staticmethod
    def pose_goal(pose):
        """TCP pose goal in BASE_FRAME. Pilz takes the target from the region/orientation."""
        goal = Constraints()
        pos = PositionConstraint()
        pos.header.frame_id = BASE_FRAME
        pos.link_name = TCP_LINK
        pos.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4]))
        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0
        pos.constraint_region.primitive_poses.append(region_pose)
        pos.weight = 1.0
        ori = OrientationConstraint()
        ori.header.frame_id = BASE_FRAME
        ori.link_name = TCP_LINK
        ori.orientation = pose.orientation
        ori.absolute_x_axis_tolerance = 1e-3
        ori.absolute_y_axis_tolerance = 1e-3
        ori.absolute_z_axis_tolerance = 1e-3
        ori.weight = 1.0
        goal.position_constraints.append(pos)
        goal.orientation_constraints.append(ori)
        return goal

    @staticmethod
    def robot_state(values):
        state = RobotState()
        state.joint_state.name = list(values.keys())
        state.joint_state.position = list(values.values())
        state.is_diff = False
        return state

    # ----- calls ------------------------------------------------------------------------------

    def run(self, request, plan_only):
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions(plan_only=plan_only)
        # no changes to the planning scene (an empty, non-diff scene would mean "replace it")
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError('MoveGroup goal rejected')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def fk(self, names, positions):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [TCP_LINK]
        req.robot_state.joint_state.name = list(names)
        req.robot_state.joint_state.position = list(positions)
        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'FK failed: {ERROR_NAMES.get(res.error_code.val)}')
        return res.pose_stamped[0].pose

    def current_joints(self, timeout=5.0):
        """Fresh /joint_states as {name: position}."""
        self.joint_state = None
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.joint_state is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            raise RuntimeError('no /joint_states received')
        return dict(zip(self.joint_state.name, self.joint_state.position))


def load_velocity_limits():
    """Read max_velocity per joint from the installed joint_limits.yaml: {joint: rad/s}."""
    path = os.path.join(
        get_package_share_directory('robotarm_moveit_config'), 'config', 'joint_limits.yaml')
    with open(path) as f:
        limits = yaml.safe_load(f)['joint_limits']
    return {joint: values['max_velocity'] for joint, values in limits.items()}


def summarize(label, result, scaling):
    """Print the result and check that no joint is faster than scaling * joint_limits.yaml."""
    code = result.error_code.val
    traj = result.planned_trajectory.joint_trajectory
    duration = duration_s(traj.points[-1].time_from_start) if traj.points else 0.0
    ok = code == MoveItErrorCodes.SUCCESS
    # move_group reports the planning time only for plan-only requests
    planning = f'planning time {result.planning_time:.3f} s, ' if result.planning_time > 0 else ''
    print(f'[{"OK" if ok else "FAIL"}] {label}: {ERROR_NAMES.get(code, code)}, {planning}'
          f'{len(traj.points)} points, trajectory duration {duration:.2f} s')
    if ok and traj.points:
        limits = load_velocity_limits()
        # highest |velocity| / allowed velocity over all joints and points (1.0 = at the limit)
        ratio = max(
            abs(v) / (scaling * limits[name])
            for point in traj.points for name, v in zip(traj.joint_names, point.velocities))
        vel_ok = ratio <= 1.0 + 1e-3
        print(f'[{"OK" if vel_ok else "FAIL"}] {label}: peak joint velocity = {ratio * 100:.1f} % '
              f'of scaling ({scaling}) x joint_limits.yaml')
        ok &= vel_ok
    return ok


def check_reached(node, label, target, tol=1e-3):
    actual = node.current_joints()
    err = max(abs(actual[j] - v) for j, v in target.items())
    ok = err < tol
    print(f'[{"OK" if ok else "FAIL"}] {label}: max joint error to target {err:.2e} rad')
    return ok


def check_lin(node, result, start_pose, dz):
    """Check that the planned LIN trajectory is a straight line of length dz along z."""
    traj = result.planned_trajectory.joint_trajectory
    poses = [node.fk(traj.joint_names, p.positions) for p in traj.points]
    p0 = start_pose.position
    end = poses[-1].position
    # distance of every waypoint from the ideal line x = x0, y = y0
    max_dev = max(math.hypot(p.position.x - p0.x, p.position.y - p0.y) for p in poses)
    moved = end.z - p0.z
    ok = abs(moved - dz) < 2e-4 and max_dev < 2e-4  # 0.2 mm
    print(f'[{"OK" if ok else "FAIL"}] LIN geometry: TCP z {p0.z:.4f} -> {end.z:.4f} m '
          f'(dz = {moved * 100:.2f} cm), max. deviation from straight line {max_dev * 1000:.3f} mm')
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scaling', type=float, default=0.5,
                        help='velocity and acceleration scaling factor (0..1], default 0.5')
    parser.add_argument('--execute-lin', action='store_true',
                        help='also execute the LIN motion (default: plan only)')
    parser.add_argument('--dz', type=float, default=0.05, help='LIN distance in z [m]')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = DemoMotions(args.scaling)
    states = load_named_states()
    all_ok = True
    try:
        node.wait_for_servers()

        print('\n(a) OMPL (RRTConnect): plan + execute to "ready"')
        res = node.run(node.request(OMPL, '', node.joint_goal(states['ready'])), plan_only=False)
        all_ok &= summarize('OMPL -> ready', res, args.scaling)
        all_ok &= check_reached(node, 'reached "ready"', states['ready'])

        print('\n(b) Pilz PTP: plan + execute to "home"')
        res = node.run(node.request(PILZ, 'PTP', node.joint_goal(states['home'])), plan_only=False)
        all_ok &= summarize('PTP -> home', res, args.scaling)
        all_ok &= check_reached(node, 'reached "home"', states['home'])

        print(f'\n(c) Pilz LIN: +{args.dz * 100:.1f} cm in z from "ready"')
        ready = states['ready']
        start_pose = node.fk(ready.keys(), ready.values())
        goal_pose = Pose()
        goal_pose.position.x = start_pose.position.x
        goal_pose.position.y = start_pose.position.y
        goal_pose.position.z = start_pose.position.z + args.dz
        goal_pose.orientation = start_pose.orientation
        p = start_pose.position
        print(f'    TCP at "ready": x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} m (frame {BASE_FRAME})')

        if args.execute_lin:
            res = node.run(node.request(PILZ, 'PTP', node.joint_goal(ready)), plan_only=False)
            all_ok &= summarize('PTP -> ready', res, args.scaling)
            res = node.run(node.request(PILZ, 'LIN', node.pose_goal(goal_pose)), plan_only=False)
        else:
            res = node.run(
                node.request(PILZ, 'LIN', node.pose_goal(goal_pose), node.robot_state(ready)),
                plan_only=True)
        label = 'LIN plan' + (' + execute' if args.execute_lin else ' (plan only)')
        lin_ok = summarize(label, res, args.scaling)
        all_ok &= lin_ok
        if lin_ok:
            all_ok &= check_lin(node, res, start_pose, args.dz)
        if args.execute_lin:
            res = node.run(node.request(PILZ, 'PTP', node.joint_goal(states['home'])),
                           plan_only=False)
            all_ok &= summarize('PTP -> home', res, args.scaling)
    except RuntimeError as e:
        print(f'[FAIL] {e}')
        all_ok = False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print('\nRESULT:', 'all checks passed' if all_ok else 'SOME CHECKS FAILED')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
