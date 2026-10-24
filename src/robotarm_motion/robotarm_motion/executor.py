"""
Execute planned joint trajectories on the joint_trajectory_controller (FollowJointTrajectory).

Used by the motion_server, the execution gate: move_group only plans there
(allow_trajectory_execution: false), trajectories reach the controller only through this class.
Before sending, a trajectory is checked:

- its first point matches the current joint state within start_tolerance (as MoveIt's
  trajectory_execution.allowed_start_tolerance),
- every point is inside the joint limits of /robot_description (+ the same tolerance, a joint may
  start on its limit).

While the controller runs it, the result is awaited with a timeout of
duration * duration_scaling + duration_margin (as MoveIt's allowed_execution_duration_scaling and
allowed_goal_duration_margin); on timeout the goal is cancelled and the controller holds.
"""

import threading

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from robotarm_motion.moveit_client import duration_s, MotionError, wait_for

ERROR_TEXT = {
    FollowJointTrajectory.Result.INVALID_GOAL: 'INVALID_GOAL',
    FollowJointTrajectory.Result.INVALID_JOINTS: 'INVALID_JOINTS',
    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: 'OLD_HEADER_TIMESTAMP',
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: 'PATH_TOLERANCE_VIOLATED',
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: 'GOAL_TOLERANCE_VIOLATED',
}


class TrajectoryExecutor:
    """Sends checked trajectories to <controller>/follow_joint_trajectory, one at a time."""

    def __init__(self, node, controller='joint_trajectory_controller', start_tolerance=0.01,
                 duration_scaling=1.2, duration_margin=1.0):
        """Create the action client for controller."""
        self.node = node
        self.controller = controller
        self.start_tolerance = start_tolerance
        self.duration_scaling = duration_scaling
        self.duration_margin = duration_margin
        self._client = ActionClient(node, FollowJointTrajectory,
                                    f'{controller}/follow_joint_trajectory')
        self._lock = threading.Lock()
        self._handle = None
        self._cancelled = False

    def wait_for_server(self, timeout=10.0):
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise MotionError(f'action /{self.controller}/follow_joint_trajectory not available')

    def check(self, trajectory, current, limits):
        """Raise MotionError if the trajectory does not start at current or leaves limits."""
        if not trajectory.points:
            raise MotionError('empty trajectory')
        unknown = [j for j in trajectory.joint_names if j not in current]
        if unknown:
            raise MotionError(f'no joint state for {unknown}')
        first = dict(zip(trajectory.joint_names, trajectory.points[0].positions))
        name, error = max(((j, abs(first[j] - current[j])) for j in first), key=lambda e: e[1])
        if error > self.start_tolerance:
            raise MotionError(f'trajectory does not start at the current state: {name} differs '
                              f'by {error:.4f} rad (> {self.start_tolerance} rad); replan')
        for point in trajectory.points:
            for j, q in zip(trajectory.joint_names, point.positions):
                if j in limits:
                    lower, upper = limits[j]
                    if not lower - self.start_tolerance <= q <= upper + self.start_tolerance:
                        raise MotionError(f'trajectory leaves the joint limits: {j} = {q:.4f} rad')

    def execute(self, trajectory, current, limits):
        """Check and run trajectory (JointTrajectory); blocks until done, raises MotionError."""
        self.check(trajectory, current, limits)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.trajectory.header.stamp.sec = 0      # zero stamp: start now
        goal.trajectory.header.stamp.nanosec = 0
        with self._lock:
            self._cancelled = False
        handle = wait_for(self._client.send_goal_async(goal), timeout=5.0)
        if not handle.accepted:
            raise MotionError(f'{self.controller} rejected the trajectory '
                              '(is it active, i.e. is a mission running?)')
        with self._lock:
            self._handle = handle
            cancelled = self._cancelled
        if cancelled:
            handle.cancel_goal_async()
        duration = duration_s(trajectory.points[-1].time_from_start)
        timeout = duration * self.duration_scaling + self.duration_margin
        try:
            response = wait_for(handle.get_result_async(), timeout=timeout)
        except TimeoutError:
            handle.cancel_goal_async()
            raise MotionError(f'execution took longer than {timeout:.1f} s, cancelled')
        finally:
            with self._lock:
                self._handle = None
        result = response.result
        if self._cancelled:
            raise MotionError('execution cancelled')
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            reason = ERROR_TEXT.get(result.error_code, result.error_code)
            raise MotionError(f'{self.controller}: {reason} {result.error_string}'.strip())

    def cancel(self):
        """Stop the running trajectory (the controller holds); safe from any thread."""
        with self._lock:
            self._cancelled = True
            handle = self._handle
        if handle is not None:
            handle.cancel_goal_async()
