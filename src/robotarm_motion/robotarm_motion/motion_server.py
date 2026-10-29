"""
motion_server: runs MoveIt motions inside a control_center mission and owns the goals file.

Jobs arrive on the action ~/run (robotarm_interface/action/RunMotion): a program of the goals file,
a single goal, or the steps of an unsaved program (the GUI's draft). For each job the server

1. loads the goals file, checks the steps (goals exist, blend radii Pilz accepts), splits them into
   segments and plans ALL of them (plan only) from the current state; if one step cannot be
   planned, nothing moves,
2. shows the plan in RViz (display_planned_path),
3. unless plan_only: starts a control_center mission in the TRAJECTORY mode, executes
   the planned segments one by one through move_group (/execute_trajectory), stops the mission.

Only one job runs at a time; further goals are rejected. A STOP, ERROR, EMERGENCY or lost
controller during the job aborts it (the firmware stops the motion, the server cancels
the running execution). Feedback and result name the steps of the executing or failed segment.

The GUI and the motion command line read and change the goals file only through these services:
  ~/get_goals       GetGoals       goals (incl. SRDF named states), programs
  ~/teach_goal      TeachGoal      save the current pose as a goal (auto-named if no name given);
                                   refused while a job runs or while the robot moves
  ~/set_goal        SetGoal        create or change a goal from given values
  ~/remove_goal     RemoveGoal     delete a goal (refused while a program uses it)
  ~/set_program     SetProgram     create or replace a program (blend problems as warnings)
  ~/remove_program  RemoveProgram  delete a program (optionally its unused auto-named goals)
The file is read again for every request, so it can also be edited by hand.

ROS parameters:
  goals_file          goals file (default $ROS_HOME/robotarm_motion/goals.yaml)
  default_scaling     velocity/acceleration scaling when a job does not set one (default 0.2)
  arm                 true: arm the hardware if control_center is only CONNECTED (default false)
  teach_max_velocity  [rad/s] teaching is refused while a joint moves faster (default 0.02)
"""

from collections import deque
import re
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from robotarm_interface.action import RunMotion
from robotarm_interface.srv import (
    GetGoals, RemoveGoal, RemoveProgram, SetGoal, SetProgram, TeachGoal)
from robotarm_motion.goal_msgs import goal_from_msg, goal_to_msg, program_to_msg, steps_from_msg
from robotarm_motion.goals import (
    CartesianGoal, check_goal, default_goals_file, GoalFile, GoalFileError, JointGoal, JOINTS,
    MOTIONS, NAME, segments, Step)
from robotarm_motion.mission import ControlCenterMission, MissionAborted, MissionError
from robotarm_motion.moveit_client import load_named_states, MotionError, MoveItClient
from robotarm_motion.planner import check_blends, Planner
from sensor_msgs.msg import JointState

STILL_WINDOW = 0.3   # [s] window of the stillness check before teaching


class JobError(RuntimeError):
    """A job or request was refused or failed; steps = (from, to), 1-based, (0, 0) = none."""

    def __init__(self, message, steps=(0, 0)):
        super().__init__(message)
        self.steps = steps


class MotionServer(Node):
    """Plans and executes RunMotion jobs, one at a time, inside a control_center mission."""

    def __init__(self):
        """Declare parameters, create clients, the ~/run action server and the services."""
        super().__init__('motion_server')
        self.declare_parameter('goals_file', default_goals_file())
        self.declare_parameter('default_scaling', 0.2)
        self.declare_parameter('arm', False)
        self.declare_parameter('teach_max_velocity', 0.02)
        param = self.get_parameter

        self.moveit = MoveItClient(self)
        self.planner = Planner(self, self.moveit)
        # STOP etc. cancel the running MoveIt goal
        self.mission = ControlCenterMission(self, self.moveit, arm=param('arm').value)
        self._busy = threading.Lock()
        self._goals_lock = threading.Lock()   # around every read and write of the goals file
        # (receive time, {joint: position}, max |velocity| or None) of the last STILL_WINDOW
        self._recent_states = deque()
        self._states_lock = threading.Lock()
        self.create_subscription(JointState, 'joint_states', self._joint_state_cb, 10)

        group = ReentrantCallbackGroup()
        self._action = ActionServer(
            self, RunMotion, '~/run', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=group)
        services = [
            (GetGoals, 'get_goals', self._get_goals),
            (TeachGoal, 'teach_goal', self._teach_goal),
            (SetGoal, 'set_goal', self._set_goal),
            (RemoveGoal, 'remove_goal', self._remove_goal),
            (SetProgram, 'set_program', self._set_program),
            (RemoveProgram, 'remove_program', self._remove_program),
        ]
        for srv_type, name, callback in services:
            self.create_service(srv_type, f'~/{name}', callback, callback_group=group)
        self.get_logger().info(
            f'motion_server ready, goals file {param("goals_file").value}, '
            'executing in the TRAJECTORY mode')

    # ----- action callbacks ------------------------------------------------------------------

    def _on_goal(self, goal):
        if sum(map(bool, (goal.program, goal.goal, len(goal.steps)))) != 1:
            self.get_logger().warn('rejected: set exactly one of program, goal and steps')
            return GoalResponse.REJECT
        if self._busy.locked():
            self.get_logger().warn('rejected: another job is running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self.moveit.cancel()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = RunMotion.Result()
        if not self._busy.acquire(blocking=False):   # a second goal accepted at the same moment
            result.message = 'another job is running'
            goal_handle.abort()
            return result
        try:
            result.duration, result.segments, notes = self._run(goal_handle)
            result.success = True
            what = 'planned' if goal_handle.request.plan_only else 'done'
            result.message = '; '.join([f'{what}: {result.segments} segment(s), '
                                        f'{result.duration:.2f} s'] + notes)
            goal_handle.succeed()
        except (JobError, GoalFileError, MotionError, MissionError, TimeoutError) as e:
            result.message = f'{type(e).__name__}: {e}'
            result.failed_step_from, result.failed_step_to = getattr(e, 'steps', (0, 0))
            self.get_logger().error(result.message)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self._busy.release()
        return result

    # ----- goals -----------------------------------------------------------------------------

    def _goal_file(self):
        return GoalFile(self.get_parameter('goals_file').value, load_named_states())

    def _respond(self, response, fn):
        """Fill success/message of response from fn() -> message; errors become the message."""
        try:
            response.message = fn()
            response.success = True
            self.get_logger().info(response.message)
        except (JobError, GoalFileError, MotionError, OSError, TimeoutError) as e:
            response.message = str(e)
        return response

    def _get_goals(self, _request, response):
        response.goals_file = self.get_parameter('goals_file').value

        def get():
            with self._goals_lock:
                goal_file = self._goal_file()
            response.goals = [goal_to_msg(g) for _, g in sorted(goal_file.goals.items())]
            response.programs = [program_to_msg(n, s)
                                 for n, s in sorted(goal_file.programs.items())]
            return f'{len(response.goals)} goals, {len(response.programs)} programs'
        try:
            response.message = get()
            response.success = True
        except (GoalFileError, OSError) as e:
            response.message = str(e)
        return response

    def _store_goal(self, goal, overwrite, prefix=None):
        """Save goal (auto-named with prefix if goal.name is empty); returns 'saved'/'replaced'."""
        with self._goals_lock:
            goal_file = self._goal_file()
            if not goal.name:
                goal.name = goal_file.next_free_name(prefix or 'p')
            check_goal(goal)
            existing = goal_file.goals.get(goal.name)
            if existing is not None and not overwrite:
                raise JobError(f'goal {goal.name!r} exists ({existing.source}), '
                               'set overwrite to replace it')
            goal.source = 'file'
            goal_file.set_goal(goal)
            goal_file.save()
        return 'replaced' if existing is not None else 'saved'

    def _teach_goal(self, request, response):
        def teach():
            goal = self._current_pose_goal(request.name, request.cartesian)
            verb = self._store_goal(goal, request.overwrite, request.prefix)
            response.goal = goal_to_msg(goal)
            return f'{verb} {"Cartesian" if request.cartesian else "joint"} goal {goal.name!r}'
        return self._respond(response, teach)

    def _current_pose_goal(self, name, cartesian):
        """Goal of the current pose; refused during a job or while the robot moves."""
        if name and not NAME.match(name):
            raise JobError(f'goal name {name!r}: letters, digits and _ only, '
                           'starting with a letter')
        if self._busy.locked():
            raise JobError('a job is running')
        self._check_still()
        current = self.moveit.current_joints()
        missing = [j for j in JOINTS if j not in current]
        if missing:
            raise JobError(f'no joint state for {missing}')
        joints = {j: current[j] for j in JOINTS}
        if not cartesian:
            return JointGoal(name, joints)
        p = self.moveit.fk(JOINTS, [joints[j] for j in JOINTS])
        o = p.orientation
        return CartesianGoal(name, (p.position.x, p.position.y, p.position.z),
                             (o.x, o.y, o.z, o.w), seed=joints)

    def _set_goal(self, request, response):
        def set_goal():
            goal = goal_from_msg(request.goal)
            verb = self._store_goal(goal, request.overwrite)
            response.goal = goal_to_msg(goal)
            return f'{verb} goal {goal.name!r}'
        return self._respond(response, set_goal)

    def _remove_goal(self, request, response):
        def remove():
            with self._goals_lock:
                goal_file = self._goal_file()
                goal_file.remove_goal(request.name)
                goal_file.save()
            return f'removed goal {request.name!r}'
        return self._respond(response, remove)

    def _set_program(self, request, response):
        def set_program():
            try:
                steps = steps_from_msg(request.steps)
                with self._goals_lock:
                    goal_file = self._goal_file()
                    if request.name in goal_file.programs and not request.overwrite:
                        raise JobError(f'program {request.name!r} exists, '
                                       'set overwrite to replace it')
                    goal_file.set_program(request.name, steps)
                    goal_file.save()
            except GoalFileError as e:
                response.bad_step = e.step
                raise
            try:
                problems = check_blends(self.planner, goal_file, steps)
            except (MotionError, TimeoutError) as e:
                problems = [(0, f'blend radii not checked: {e}')]
            response.warnings = [text for _, text in problems]
            response.bad_step = next((n for n, _ in problems if n), 0)
            return (f'saved program {request.name!r} ({len(steps)} steps)'
                    + (f', {len(problems)} warning(s)' if problems else ''))
        return self._respond(response, set_program)

    def _remove_program(self, request, response):
        def remove():
            with self._goals_lock:
                goal_file = self._goal_file()
                goal_file.remove_program(request.name)
                if request.remove_unused_goals:
                    own = re.compile(rf'^{re.escape(request.name)}_p\d+$')
                    for name, goal in list(goal_file.goals.items()):
                        if own.match(name) and goal.source == 'file' and not goal_file.users(name):
                            goal_file.remove_goal(name)
                            response.removed_goals.append(name)
                goal_file.save()
            removed = (f' and {len(response.removed_goals)} goal(s)'
                       if response.removed_goals else '')
            return f'removed program {request.name!r}{removed}'
        return self._respond(response, remove)

    # ----- stillness check -------------------------------------------------------------------

    def _joint_state_cb(self, msg):
        now = time.monotonic()
        speed = max((abs(v) for v in msg.velocity), default=None)
        with self._states_lock:
            self._recent_states.append((now, dict(zip(msg.name, msg.position)), speed))
            while self._recent_states and now - self._recent_states[0][0] > STILL_WINDOW:
                self._recent_states.popleft()

    def _check_still(self):
        """Raise JobError if a joint moved faster than teach_max_velocity in the last window."""
        limit = self.get_parameter('teach_max_velocity').value
        with self._states_lock:
            states = list(self._recent_states)
        if not states:
            return   # current_joints() reports missing joint states
        speeds = [s for _, _, s in states if s is not None]
        if speeds:
            speed = max(speeds)
        elif len(states) > 1:   # no velocities in /joint_states: from the positions
            dt = states[-1][0] - states[0][0]
            first, last = states[0][1], states[-1][1]
            speed = max(abs(last[j] - first[j]) for j in first if j in last) / dt if dt else 0.0
        else:
            speed = 0.0
        if speed > limit:
            raise JobError(f'robot is moving ({speed:.3f} rad/s > {limit} rad/s), hold still')

    # ----- job -------------------------------------------------------------------------------

    def _feedback(self, goal_handle, phase, segment=0, segments_=0, steps=(0, 0)):
        fb = RunMotion.Feedback()
        fb.phase, fb.segment, fb.segments = phase, segment, segments_
        fb.step_from, fb.step_to = steps
        goal_handle.publish_feedback(fb)

    def _job_steps(self, req, goal_file):
        """Return the steps of the job and its description."""
        if req.program:
            return goal_file.program(req.program), f'program {req.program!r}'
        if len(req.steps):
            return steps_from_msg(req.steps), f'{len(req.steps)} steps'
        motion = (req.motion or 'PTP').upper()
        if motion not in MOTIONS:
            raise JobError(f'motion must be one of {MOTIONS}, got {motion!r}')
        return [Step(req.goal, motion)], f'{motion} to {req.goal!r}'

    def _run(self, goal_handle):
        req = goal_handle.request
        scaling = req.scaling or self.get_parameter('default_scaling').value
        if not 0 < scaling <= 1:
            raise JobError(f'scaling must be in (0, 1], got {scaling}')
        with self._goals_lock:
            goal_file = self._goal_file()
        try:
            steps, what = self._job_steps(req, goal_file)
            goal_file.check_steps(steps)
        except GoalFileError as e:
            raise JobError(str(e), (e.step, e.step))
        problems = check_blends(self.planner, goal_file, steps)
        if problems:
            n = problems[0][0]
            raise JobError('; '.join(text for _, text in problems), (n, n))
        segs, notes = segments(steps)
        ranges, first = [], 1   # 1-based (from, to) step range of every segment
        for seg in segs:
            ranges.append((first, first + len(seg.steps) - 1))
            first += len(seg.steps)

        self._feedback(goal_handle, 'PLANNING', 0, len(segs))
        start = self.moveit.start_joints()
        try:
            planned = self.planner.plan(goal_file, segs, scaling, start)
        except MotionError as e:
            raise JobError(str(e), ranges[getattr(e, 'segment', 0)])
        self.planner.display(planned, start)
        duration = sum(p.duration for p in planned)
        self.get_logger().info(f'{what}: planned {len(planned)} segment(s), {duration:.2f} s, '
                               f'scaling {scaling}')
        if req.plan_only:
            return duration, len(planned), notes

        self._feedback(goal_handle, 'STARTING_MISSION', 0, len(planned))
        self.mission.start()
        try:
            for i, seg in enumerate(planned, 1):
                if goal_handle.is_cancel_requested:
                    raise JobError('cancelled by the client')
                self._feedback(goal_handle, 'EXECUTING', i, len(planned), ranges[i - 1])
                self.mission.check()
                try:
                    self.moveit.execute_trajectory(seg.trajectory)
                except MotionError as e:
                    if goal_handle.is_cancel_requested:
                        raise JobError('cancelled by the client', ranges[i - 1])
                    self.mission.check_after_failure()   # report a STOP rather than the error
                    raise JobError(str(e), ranges[i - 1])
            self.mission.check()
        except MissionAborted as e:
            raise JobError(f'aborted, {e}')
        finally:
            self._feedback(goal_handle, 'STOPPING_MISSION', 0, len(planned))
            self.mission.stop()
        return duration, len(planned), notes


def main(args=None):
    """Run the motion_server node."""
    rclpy.init(args=args)
    node = MotionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # wait for move_group in the background, the node spins meanwhile
    def wait():
        try:
            node.moveit.wait_for_servers(timeout=60.0)
            node.planner.wait_for_services(timeout=60.0)
            node.get_logger().info('move_group found')
        except MotionError as e:
            node.get_logger().error(f'{e}; jobs will fail until it is available')
    threading.Thread(target=wait, daemon=True).start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
