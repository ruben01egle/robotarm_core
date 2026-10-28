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

ROS parameters:
  goals_file        goals file (default $ROS_HOME/robotarm_motion/goals.yaml)
  default_scaling   velocity/acceleration scaling when a job does not set one (default 0.2)
  controller        trajectory controller (default joint_trajectory_controller)
  arm               true: arm the hardware if control_center is only CONNECTED (default false)
"""

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from robotarm_interface.action import RunMotion
from robotarm_motion.goals import (
    default_goals_file, GoalFile, GoalFileError, MOTIONS, segments, Step)
from robotarm_motion.mission import ControlCenterMission, MissionAborted, MissionError
from robotarm_motion.moveit_client import load_named_states, MotionError, MoveItClient
from robotarm_motion.planner import check_goal_file_steps, Planner


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
        self._action = ActionServer(
            self, RunMotion, '~/run', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())
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
        path = self.get_parameter('goals_file').value
        goal_file = GoalFile(path, load_named_states())
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
