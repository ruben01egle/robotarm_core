"""
Command line for goals, programs and motions: ros2 run robotarm_motion motion <command>.

  list                           goals (incl. SRDF named states) and programs of the goals file
  teach NAME [--cartesian] [--force]
                                 save the current robot pose as goal NAME (joint goal, or tcp pose
                                 with the current joints as IK seed)
  remove NAME                    delete goal NAME from the goals file
  run PROGRAM [--plan-only] [--scaling S]
  move GOAL [--motion PTP|LIN|OMPL] [--plan-only] [--scaling S]
                                 plan (and execute)

Every command goes through the motion_server, which owns the goals file (its parameter
goals_file).
"""

import argparse
import math
import sys

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robotarm_interface.action import RunMotion
from robotarm_interface.srv import GetGoals, RemoveGoal, TeachGoal
from robotarm_motion.goal_msgs import goal_from_msg, steps_from_msg
from robotarm_motion.goals import GoalFileError, JointGoal, JOINTS, MOTIONS, rpy_from_quat
from robotarm_motion.moveit_client import MotionError, spin_in_background, wait_for

SERVER = 'motion_server'


def deg(values):
    return '[' + ', '.join(f'{math.degrees(v):7.2f}' for v in values) + ']'


def with_node(fn):
    """Run fn(node) with a node spinning in the background."""
    # no rclpy SIGINT handler: Ctrl-C has to reach run() as KeyboardInterrupt, so the job can still
    # be cancelled on the motion_server before rclpy shuts down
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('robotarm_motion_cli')
    executor = spin_in_background(node)
    try:
        return fn(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def call(srv_type, name, request):
    """Call the motion_server service name; returns the response, MotionError if it failed."""
    def run(node):
        client = node.create_client(srv_type, f'/{SERVER}/{name}')
        if not client.wait_for_service(timeout_sec=10.0):
            raise MotionError(f'motion_server not running (/{SERVER}/{name})')
        return wait_for(client.call_async(request), timeout=30.0)
    response = with_node(run)
    if not response.success:
        raise MotionError(response.message)
    return response


def print_goal(goal, indent='  '):
    src = ' (SRDF)' if goal.source == 'srdf' else ''
    if isinstance(goal, JointGoal):
        print(f'{indent}{goal.name:20s} joint  deg {deg(goal.joints[j] for j in JOINTS)}{src}')
    else:
        seed = '' if not goal.seed else f'  seed {deg(goal.seed[j] for j in JOINTS)}'
        print(f'{indent}{goal.name:20s} cart   xyz [{", ".join(f"{v:.4f}" for v in goal.xyz)}] '
              f'm  rpy {deg(rpy_from_quat(*goal.quat))} deg{seed}')


def cmd_list(args):
    res = call(GetGoals, 'get_goals', GetGoals.Request())
    print(f'goals file: {res.goals_file}')
    print('\ngoals:')
    for msg in res.goals:
        print_goal(goal_from_msg(msg))
    print('\nprograms:')
    for program in res.programs:
        print(f'  {program.name}:')
        for s in steps_from_msg(program):
            extra = ''.join([f'  blend {s.blend} m' if s.blend else '',
                             f'  scaling {s.scaling}' if s.scaling else '',
                             f'  offset {list(s.offset_xyz)} m' if s.offset_xyz else ''])
            print(f'    {s.motion:4s} -> {s.goal}{extra}')
    if not res.programs:
        print('  (none)')
    return 0


def cmd_teach(args):
    req = TeachGoal.Request(name=args.name, cartesian=args.cartesian, overwrite=args.force)
    res = call(TeachGoal, 'teach_goal', req)
    print(res.message)
    print_goal(goal_from_msg(res.goal))
    return 0


def cmd_remove(args):
    print(call(RemoveGoal, 'remove_goal', RemoveGoal.Request(name=args.name)).message)
    return 0


def cmd_motion(args):
    goal = RunMotion.Goal()
    if args.command == 'run':
        goal.program = args.program
    else:
        goal.goal = args.goal
        goal.motion = args.motion
    goal.scaling = args.scaling
    goal.plan_only = args.plan_only

    def run(node):
        client = ActionClient(node, RunMotion, f'/{SERVER}/run')
        if not client.wait_for_server(timeout_sec=10.0):
            print('motion_server not running (/motion_server/run)')
            return 1
        last = [None]

        def feedback(msg):
            fb = msg.feedback
            text = fb.phase + (f' {fb.segment}/{fb.segments}' if fb.segment else '')
            if text != last[0]:
                print(f'  {text}')
                last[0] = text

        handle = wait_for(client.send_goal_async(goal, feedback_callback=feedback), 10.0)
        if not handle.accepted:
            print('rejected by the motion_server (another job running? see its log)')
            return 1
        try:
            response = wait_for(handle.get_result_async())
        except KeyboardInterrupt:
            print('cancelling ...')
            wait_for(handle.cancel_goal_async(), 5.0)
            response = wait_for(handle.get_result_async(), 30.0)
        result = response.result
        ok = response.status == GoalStatus.STATUS_SUCCEEDED and result.success
        print(('OK: ' if ok else 'FAILED: ') + result.message)
        return 0 if ok else 1

    return with_node(run)


def main():
    """Entry point of ros2 run robotarm_motion motion."""
    parser = argparse.ArgumentParser(prog='motion', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('list')
    p = sub.add_parser('teach')
    p.add_argument('name')
    p.add_argument('--cartesian', action='store_true',
                   help='save the tcp pose (with the current joints as IK seed)')
    p.add_argument('--force', action='store_true', help='replace an existing goal')
    p = sub.add_parser('remove')
    p.add_argument('name')
    for name, target in (('run', 'program'), ('move', 'goal')):
        p = sub.add_parser(name)
        p.add_argument(target)
        if name == 'move':
            p.add_argument('--motion', default='PTP', type=str.upper, choices=MOTIONS)
        p.add_argument('--plan-only', action='store_true', help='plan and show in RViz only')
        p.add_argument('--scaling', type=float, default=0.0,
                       help='velocity/acceleration scaling (0..1], default: the server default')
    args, _ = parser.parse_known_args()
    commands = {'list': cmd_list, 'teach': cmd_teach, 'remove': cmd_remove,
                'run': cmd_motion, 'move': cmd_motion}
    try:
        return commands[args.command](args)
    except (GoalFileError, MotionError, TimeoutError) as e:
        print(f'error: {e}')
        return 1


if __name__ == '__main__':
    sys.exit(main())
