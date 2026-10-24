"""
Command line for goals, programs and motions: ros2 run robotarm_motion motion <command>.

  list                           goals (incl. SRDF named states) and programs of the goals file
  init [--force]                 create the goals file from the installed example
  teach NAME [--cartesian] [--force]
                                 save the current robot pose as goal NAME (joint goal, or tcp pose
                                 with the current joints as IK seed)
  remove NAME                    delete goal NAME from the goals file
  run PROGRAM [--plan-only] [--scaling S]
  move GOAL [--motion PTP|LIN|OMPL] [--plan-only] [--scaling S]
                                 plan (and execute) through the motion_server, the execution gate

--goals FILE selects another goals file (for run/move also passed to the motion_server).
"""

import argparse
import math
import os
import shutil
import sys

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robotarm_motion.goals import (
    CartesianGoal, default_goals_file, GoalFile, GoalFileError, JointGoal, JOINTS, MOTIONS,
    rpy_from_quat)
from robotarm_motion.moveit_client import (
    load_named_states, MotionError, MoveItClient, spin_in_background, wait_for)
from robotarm_motion_msgs.action import RunMotion

EXAMPLE = os.path.join(get_package_share_directory('robotarm_motion'), 'config',
                       'goals_example.yaml')


def deg(values):
    return '[' + ', '.join(f'{math.degrees(v):7.2f}' for v in values) + ']'


def cmd_list(args):
    f = GoalFile(args.goals, load_named_states())
    exists = os.path.exists(args.goals)
    print(f'goals file: {args.goals}' + ('' if exists else ' (does not exist)'))
    print('\ngoals:')
    for name, g in sorted(f.goals.items()):
        src = ' (SRDF)' if g.source == 'srdf' else ''
        if isinstance(g, JointGoal):
            print(f'  {name:20s} joint  deg {deg(g.joints[j] for j in JOINTS)}{src}')
        else:
            rpy = rpy_from_quat(*g.quat)
            seed = '' if not g.seed else f'  seed {deg(g.seed[j] for j in JOINTS)}'
            print(f'  {name:20s} cart   xyz [{", ".join(f"{v:.4f}" for v in g.xyz)}] m  '
                  f'rpy {deg(rpy)} deg{seed}')
    print('\nprograms:')
    for name, steps in sorted(f.programs.items()):
        print(f'  {name}:')
        for s in steps:
            extra = ''.join([f'  blend {s.blend} m' if s.blend else '',
                             f'  scaling {s.scaling}' if s.scaling else '',
                             f'  offset {list(s.offset_xyz)} m' if s.offset_xyz else ''])
            print(f'    {s.motion:4s} -> {s.goal}{extra}')
    if not f.programs:
        print('  (none)')
    return 0


def cmd_init(args):
    if os.path.exists(args.goals) and not args.force:
        print(f'{args.goals} exists, use --force to overwrite')
        return 1
    os.makedirs(os.path.dirname(os.path.abspath(args.goals)), exist_ok=True)
    shutil.copyfile(EXAMPLE, args.goals)
    print(f'created {args.goals} from {EXAMPLE}')
    return 0


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


def cmd_teach(args):
    f = GoalFile(args.goals, load_named_states())
    if args.name in f.goals and not args.force:
        print(f'goal {args.name!r} exists ({f.goals[args.name].source}), '
              'use --force to replace it')
        return 1

    def teach(node):
        moveit = MoveItClient(node)
        joints = {j: v for j, v in moveit.current_joints().items() if j in JOINTS}
        if args.cartesian:
            moveit.wait_for_servers(timeout=10.0, execute=False)
            p = moveit.fk(JOINTS, [joints[j] for j in JOINTS])
            o = p.orientation
            return CartesianGoal(args.name, (p.position.x, p.position.y, p.position.z),
                                 (o.x, o.y, o.z, o.w), seed=joints)
        return JointGoal(args.name, joints)

    goal = with_node(teach)
    f.set_goal(goal)
    f.save()
    print(f'saved {"Cartesian" if args.cartesian else "joint"} goal {args.name!r} to {args.goals}')
    return cmd_list_one(goal)


def cmd_list_one(goal):
    if isinstance(goal, JointGoal):
        print(f'  joints deg {deg(goal.joints[j] for j in JOINTS)}')
    else:
        print(f'  xyz {[round(v, 4) for v in goal.xyz]} m, '
              f'rpy deg {deg(rpy_from_quat(*goal.quat))}, '
              f'seed deg {deg(goal.seed[j] for j in JOINTS)}')
    return 0


def cmd_remove(args):
    f = GoalFile(args.goals, load_named_states())
    f.remove_goal(args.name)
    f.save()
    print(f'removed goal {args.name!r} from {args.goals}')
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
    goal.goals_file = args.goals if args.goals_given else ''

    def run(node):
        client = ActionClient(node, RunMotion, '/motion_server/run')
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
    parser.add_argument('--goals', help=f'goals file (default {default_goals_file()})')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('list')
    p = sub.add_parser('init')
    p.add_argument('--force', action='store_true')
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
    args.goals_given = args.goals is not None
    args.goals = args.goals or default_goals_file()
    commands = {'list': cmd_list, 'init': cmd_init, 'teach': cmd_teach, 'remove': cmd_remove,
                'run': cmd_motion, 'move': cmd_motion}
    try:
        return commands[args.command](args)
    except (GoalFileError, MotionError, TimeoutError) as e:
        print(f'error: {e}')
        return 1


if __name__ == '__main__':
    sys.exit(main())
