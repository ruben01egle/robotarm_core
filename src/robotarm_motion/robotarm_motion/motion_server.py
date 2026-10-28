"""
motion_server: runs MoveIt motions inside a control_center mission.

Jobs (a program or a single goal from the goals file) arrive on the action ~/run
(robotarm_interface/action/RunMotion). For each job the server

1. loads the goals file, splits the program into segments and plans ALL of them (plan only) from
   the current state; if one step cannot be planned, nothing moves,
2. shows the plan in RViz (display_planned_path),
3. unless plan_only: starts a control_center mission for the joint_trajectory_controller, executes
   the planned segments one by one through move_group (/execute_trajectory), stops the mission.

Only one job runs at a time; further goals are rejected. A STOP, ERROR, EMERGENCY or lost
controller during the job aborts it (control_center deactivates the controller, the server cancels
the running execution).

The server owns the goals file; the GUI and the motion command line use its services:
  ~/get_goals    (robotarm_interface/srv/GetGoals)    goals (incl. SRDF named states), programs
  ~/teach_goal   (robotarm_interface/srv/TeachGoal)   save the current pose as a goal
  ~/remove_goal  (robotarm_interface/srv/RemoveGoal)  delete a goal
The file is read again for every request, so it can also be edited by hand.

ROS parameters:
  goals_file        goals file (default $ROS_HOME/robotarm_motion/goals.yaml)
  default_scaling   velocity/acceleration scaling when a job does not set one (default 0.2)
  controller        trajectory controller (default joint_trajectory_controller)
  arm               true: arm the hardware if control_center is only CONNECTED (default false)
"""

import re
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from robotarm_interface.action import RunMotion
from robotarm_interface.srv import GetGoals, RemoveGoal, TeachGoal
from robotarm_motion.goal_msgs import goal_to_msg, program_to_msg
from robotarm_motion.goals import (
    CartesianGoal, default_goals_file, GoalFile, GoalFileError, JointGoal, JOINTS, MOTIONS,
    segments, Step)
from robotarm_motion.mission import ControlCenterMission, MissionAborted, MissionError
from robotarm_motion.moveit_client import load_named_states, MotionError, MoveItClient
from robotarm_motion.planner import check_goal_file_steps, Planner

GOAL_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


class JobError(RuntimeError):
    """A job was rejected or failed; the message goes to the client."""


class MotionServer(Node):
    """Plans and executes RunMotion jobs, one at a time, inside a control_center mission."""

    def __init__(self):
        """Declare parameters, create clients and the ~/run action server."""
        super().__init__('motion_server')
        self.declare_parameter('goals_file', default_goals_file())
        self.declare_parameter('default_scaling', 0.2)
        self.declare_parameter('controller', 'joint_trajectory_controller')
        self.declare_parameter('arm', False)
        param = self.get_parameter
        self.controller = param('controller').value

        self.moveit = MoveItClient(self)
        self.planner = Planner(self, self.moveit)
        # STOP etc. cancel the running MoveIt goal
        self.mission = ControlCenterMission(self, self.moveit, controllers=[self.controller],
                                            arm=param('arm').value)
        self._busy = threading.Lock()
        self._goals_lock = threading.Lock()   # around every read and write of the goals file
        group = ReentrantCallbackGroup()
        self._action = ActionServer(
            self, RunMotion, '~/run', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=group)
        self.create_service(GetGoals, '~/get_goals', self._get_goals, callback_group=group)
        self.create_service(TeachGoal, '~/teach_goal', self._teach_goal, callback_group=group)
        self.create_service(RemoveGoal, '~/remove_goal', self._remove_goal, callback_group=group)
        self.get_logger().info(
            f'motion_server ready, goals file {param("goals_file").value}, '
            f'executing on {self.controller}')

    # ----- action callbacks ------------------------------------------------------------------

    def _on_goal(self, goal):
        if bool(goal.program) == bool(goal.goal):
            self.get_logger().warn('rejected: set exactly one of program and goal')
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

    def _get_goals(self, _request, response):
        response.goals_file = self.get_parameter('goals_file').value
        try:
            with self._goals_lock:
                goal_file = self._goal_file()
        except (GoalFileError, OSError) as e:
            response.message = str(e)
            return response
        response.goals = [goal_to_msg(g) for _, g in sorted(goal_file.goals.items())]
        response.programs = [program_to_msg(n, s) for n, s in sorted(goal_file.programs.items())]
        response.success = True
        return response

    def _teach_goal(self, request, response):
        try:
            goal, replaced = self._teach(request.name, request.cartesian, request.overwrite)
        except (JobError, GoalFileError, MotionError, OSError, TimeoutError) as e:
            response.message = str(e)
            return response
        kind = 'Cartesian' if request.cartesian else 'joint'
        response.message = f'{"replaced" if replaced else "saved"} {kind} goal {goal.name!r}'
        response.goal = goal_to_msg(goal)
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _teach(self, name, cartesian, overwrite):
        """Save the current pose as goal name; returns (goal, replaced an existing one)."""
        if not GOAL_NAME.match(name):
            raise JobError(f'goal name {name!r}: letters, digits and _ only, '
                           'starting with a letter')
        if self._busy.locked():
            raise JobError('a job is running')
        current = self.moveit.current_joints()
        missing = [j for j in JOINTS if j not in current]
        if missing:
            raise JobError(f'no joint state for {missing}')
        joints = {j: current[j] for j in JOINTS}
        if cartesian:
            p = self.moveit.fk(JOINTS, [joints[j] for j in JOINTS])
            o = p.orientation
            goal = CartesianGoal(name, (p.position.x, p.position.y, p.position.z),
                                 (o.x, o.y, o.z, o.w), seed=joints)
        else:
            goal = JointGoal(name, joints)
        with self._goals_lock:
            goal_file = self._goal_file()
            existing = goal_file.goals.get(name)
            if existing is not None and not overwrite:
                raise JobError(f'goal {name!r} exists ({existing.source}), '
                               'set overwrite to replace it')
            goal_file.set_goal(goal)
            goal_file.save()
        return goal, existing is not None

    def _remove_goal(self, request, response):
        try:
            with self._goals_lock:
                goal_file = self._goal_file()
                goal_file.remove_goal(request.name)
                goal_file.save()
        except (GoalFileError, OSError) as e:
            response.message = str(e)
            return response
        response.message = f'removed goal {request.name!r}'
        response.success = True
        self.get_logger().info(response.message)
        return response

    # ----- job -------------------------------------------------------------------------------

    def _feedback(self, goal_handle, phase, segment=0, segments_=0):
        fb = RunMotion.Feedback()
        fb.phase, fb.segment, fb.segments = phase, segment, segments_
        goal_handle.publish_feedback(fb)

    def _run(self, goal_handle):
        req = goal_handle.request
        scaling = req.scaling or self.get_parameter('default_scaling').value
        if not 0 < scaling <= 1:
            raise JobError(f'scaling must be in (0, 1], got {scaling}')
        with self._goals_lock:
            goal_file = self._goal_file()
        if req.program:
            steps = goal_file.program(req.program)
            what = f'program {req.program!r}'
        else:
            motion = (req.motion or 'PTP').upper()
            if motion not in MOTIONS:
                raise JobError(f'motion must be one of {MOTIONS}, got {motion!r}')
            steps = [Step(req.goal, motion)]
            what = f'{motion} to {req.goal!r}'
        check_goal_file_steps(goal_file, steps)
        segs, notes = segments(steps)

        self._feedback(goal_handle, 'PLANNING', 0, len(segs))
        start = self.moveit.start_joints()
        planned = self.planner.plan(goal_file, segs, scaling, start)
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
                self._feedback(goal_handle, 'EXECUTING', i, len(planned))
                self.mission.check()
                try:
                    self.moveit.execute_trajectory(seg.trajectory)
                except MotionError:
                    self.mission.check_after_failure()   # report a STOP rather than the error
                    raise
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
