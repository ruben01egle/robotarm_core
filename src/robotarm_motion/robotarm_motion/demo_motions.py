"""
Plan and execute test motions through move_group.

  (a) OMPL:      plan + execute to the named state "ready"
  (b) Pilz PTP:  plan + execute back to "home"
  (c) Pilz LIN:  plan a straight TCP motion of +5 cm in z (base frame), starting at "ready".
                 Plan only by default (start state given explicitly, the robot does not move);
                 with --execute-lin the robot moves PTP to "ready", executes the LIN and returns
                 home.

Every motion is checked: result code, peak joint velocity <= scaling x joint_limits.yaml, goal
reached (/joint_states) and, for LIN, a straight TCP path of the requested length.

Without --mission the motions are sent directly (robotarm_moveit_config demo.launch.py, where
joint_trajectory_controller is always active). With --mission they run inside a control_center
mission, as needed on the real robot and the firmware mock.

Usage:
  ros2 run robotarm_motion demo_motions [--scaling 0.5] [--execute-lin] [--mission [--arm]]
"""

import argparse
import contextlib
import math
import sys

from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.node import Node

from robotarm_motion.mission import ControlCenterMission, MissionError
from robotarm_motion.moveit_client import (
    BASE_FRAME,
    duration_s,
    ERROR_NAMES,
    load_named_states,
    load_velocity_limits,
    MotionError,
    MoveItClient,
    OMPL,
    PILZ,
    spin_in_background,
)


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


def check_reached(moveit, label, target, tol=1e-3):
    """Compare /joint_states with the target joint values."""
    actual = moveit.current_joints()
    err = max(abs(actual[j] - v) for j, v in target.items())
    ok = err < tol
    print(f'[{"OK" if ok else "FAIL"}] {label}: max joint error to target {err:.2e} rad')
    return ok


def check_lin(moveit, result, start_pose, dz):
    """Check that the planned LIN trajectory is a straight line of length dz along z."""
    traj = result.planned_trajectory.joint_trajectory
    poses = [moveit.fk(traj.joint_names, p.positions) for p in traj.points]
    p0 = start_pose.position
    end = poses[-1].position
    # distance of every waypoint from the ideal line x = x0, y = y0
    max_dev = max(math.hypot(p.position.x - p0.x, p.position.y - p0.y) for p in poses)
    moved = end.z - p0.z
    ok = abs(moved - dz) < 2e-4 and max_dev < 2e-4  # 0.2 mm
    print(f'[{"OK" if ok else "FAIL"}] LIN geometry: TCP z {p0.z:.4f} -> {end.z:.4f} m '
          f'(dz = {moved * 100:.2f} cm), '
          f'max. deviation from straight line {max_dev * 1000:.3f} mm')
    return ok


def run_motions(moveit, runner, args):
    """Run (a)-(c); runner.run is MoveItClient.run or ControlCenterMission.run."""
    states = load_named_states()
    s = args.scaling
    ok = True

    print('\n(a) OMPL (RRTConnect): plan + execute to "ready"')
    res = runner.run(moveit.request(OMPL, '', moveit.joint_goal(states['ready'])), False)
    ok &= summarize('OMPL -> ready', res, s)
    ok &= check_reached(moveit, 'reached "ready"', states['ready'])

    print('\n(b) Pilz PTP: plan + execute to "home"')
    res = runner.run(moveit.request(PILZ, 'PTP', moveit.joint_goal(states['home'])), False)
    ok &= summarize('PTP -> home', res, s)
    ok &= check_reached(moveit, 'reached "home"', states['home'])

    print(f'\n(c) Pilz LIN: +{args.dz * 100:.1f} cm in z from "ready"')
    ready = states['ready']
    start_pose = moveit.fk(ready.keys(), ready.values())
    goal_pose = Pose()
    goal_pose.position.x = start_pose.position.x
    goal_pose.position.y = start_pose.position.y
    goal_pose.position.z = start_pose.position.z + args.dz
    goal_pose.orientation = start_pose.orientation
    p = start_pose.position
    print(f'    TCP at "ready": x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} m (frame {BASE_FRAME})')

    if args.execute_lin:
        res = runner.run(moveit.request(PILZ, 'PTP', moveit.joint_goal(ready)), False)
        ok &= summarize('PTP -> ready', res, s)
        res = runner.run(moveit.request(PILZ, 'LIN', moveit.pose_goal(goal_pose)), False)
    else:
        start = moveit.robot_state(ready)
        res = runner.run(moveit.request(PILZ, 'LIN', moveit.pose_goal(goal_pose), start), True)
    label = 'LIN plan' + (' + execute' if args.execute_lin else ' (plan only)')
    lin_ok = summarize(label, res, s)
    ok &= lin_ok
    if lin_ok:
        ok &= check_lin(moveit, res, start_pose, args.dz)
    if args.execute_lin:
        res = runner.run(moveit.request(PILZ, 'PTP', moveit.joint_goal(states['home'])), False)
        ok &= summarize('PTP -> home', res, s)
    return ok


def main():
    """Entry point of ros2 run robotarm_motion demo_motions."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scaling', type=float, default=0.5,
                        help='velocity and acceleration scaling factor (0..1], default 0.5')
    parser.add_argument('--execute-lin', action='store_true',
                        help='also execute the LIN motion (default: plan only)')
    parser.add_argument('--dz', type=float, default=0.05, help='LIN distance in z [m]')
    parser.add_argument('--mission', action='store_true',
                        help='run inside a control_center mission (real robot, firmware mock)')
    parser.add_argument('--arm', action='store_true',
                        help='with --mission: arm the hardware first if it is only CONNECTED')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node('robotarm_demo_motions')
    executor = spin_in_background(node)
    moveit = MoveItClient(node, scaling=args.scaling)
    mission = ControlCenterMission(node, moveit, arm=args.arm) if args.mission else None
    all_ok = False
    try:
        moveit.wait_for_servers()
        with mission if mission else contextlib.nullcontext():
            all_ok = run_motions(moveit, mission or moveit, args)
    except (MissionError, MotionError, TimeoutError) as e:
        print(f'[FAIL] {type(e).__name__}: {e}')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    print('\nRESULT:', 'all checks passed' if all_ok else 'SOME CHECKS FAILED')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
