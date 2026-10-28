"""
Blocking Python client for MoveIt's move_group (MoveGroup action, FK service).

Only rclpy and moveit_msgs are used, so the code also shows what the RViz MotionPlanning panel
does under the hood. All calls block the calling thread; the node has to be spun by an executor in
another thread (see spin_in_background).
"""

import os
import threading
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import yaml

GROUP = 'arm'
BASE_FRAME = 'base'
TCP_LINK = 'tcp'
OMPL = 'ompl'
PILZ = 'pilz_industrial_motion_planner'
CONFIG_PACKAGE = 'robotarm_moveit_config'
# [rad] a current position at most this far outside a joint limit is clamped onto the limit before
# planning (a joint resting on its limit). Same value as
# trajectory_execution.allowed_start_tolerance in moveit_controllers.yaml, so execution accepts
# the clamped start; larger deviations are left to MoveIt, which refuses (START_STATE_INVALID).
START_STATE_CLAMP_TOLERANCE = 0.01
# [rad] clamped positions are put this far inside the limit. A joint held on its limit jitters
# around it by a few µrad (seen in the MuJoCo sim): without the margin, move_group can read a
# value just outside while the client saw one just inside.
START_STATE_CLAMP_MARGIN = 1e-6

# MoveItErrorCodes value -> name, for readable output
ERROR_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}


class MotionError(RuntimeError):
    """Planning or execution failed; error_code is the MoveItErrorCodes value."""

    def __init__(self, message, error_code=None):
        super().__init__(message)
        self.error_code = error_code


def spin_in_background(node):
    """Spin node in a daemon thread; returns the executor (call shutdown() at the end)."""
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return executor


def wait_for(future, timeout=None):
    """Block until an rclpy future is done and return its result (node spun elsewhere)."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise TimeoutError('timed out waiting for a ROS response')
    return future.result()


def load_named_states():
    """Read the named states of GROUP from the installed SRDF: {name: {joint: value}}."""
    srdf = os.path.join(get_package_share_directory(CONFIG_PACKAGE), 'config', 'robotarm.srdf')
    states = {}
    for state in ET.parse(srdf).getroot().iter('group_state'):
        if state.get('group') == GROUP:
            states[state.get('name')] = {
                j.get('name'): float(j.get('value')) for j in state.iter('joint')}
    return states


def load_velocity_limits():
    """Read max_velocity per joint from the installed joint_limits.yaml: {joint: rad/s}."""
    path = os.path.join(get_package_share_directory(CONFIG_PACKAGE), 'config', 'joint_limits.yaml')
    with open(path) as f:
        limits = yaml.safe_load(f)['joint_limits']
    return {joint: values['max_velocity'] for joint, values in limits.items()}


def parse_position_limits(urdf):
    """Position limits of the revolute/prismatic joints of a URDF: {joint: (lower, upper)}."""
    limits = {}
    for joint in ET.fromstring(urdf).iter('joint'):
        limit = joint.find('limit')
        if joint.get('type') in ('revolute', 'prismatic') and limit is not None:
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    return limits


def clamp_to_limits(positions, limits, tolerance=START_STATE_CLAMP_TOLERANCE,
                    margin=START_STATE_CLAMP_MARGIN):
    """
    Clamp positions {joint: rad} that are at most tolerance outside their limits, margin inside.

    Returns (clamped positions, {joint: clamped amount}); joints further outside are kept as they
    are, so MoveIt reports them.
    """
    clamped, changes = dict(positions), {}
    for name, value in positions.items():
        if name not in limits:
            continue
        lower, upper = limits[name]
        target = min(max(value, lower + margin), upper - margin)
        if target != value and abs(target - value) <= tolerance:
            clamped[name] = target
            changes[name] = target - value
    return clamped, changes


def duration_s(duration):
    """Convert a builtin_interfaces/Duration to seconds."""
    return duration.sec + duration.nanosec * 1e-9


class MoveItClient:
    """Plan and execute motions of the planning group GROUP through move_group."""

    def __init__(self, node, scaling=0.1):
        """Create the clients on node; scaling is the velocity/acceleration scaling (0..1]."""
        self.node = node
        self.scaling = scaling
        self._move_client = ActionClient(node, MoveGroup, 'move_action')
        self._execute_client = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
        self._fk_client = node.create_client(GetPositionFK, 'compute_fk')
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._joint_state = None
        self._joint_state_event = threading.Event()
        self._position_limits = {}
        node.create_subscription(JointState, 'joint_states', self._joint_state_cb, 10)
        # the URDF the robot runs with (robot_state_publisher, latched), for the position limits
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, 'robot_description', self._description_cb, latched)

    def _joint_state_cb(self, msg):
        self._joint_state = msg
        self._joint_state_event.set()

    def _description_cb(self, msg):
        self._position_limits = parse_position_limits(msg.data)

    @property
    def position_limits(self):
        """Joint position limits {joint: (lower, upper)} of /robot_description ({} before)."""
        return self._position_limits

    def _clamped(self, positions):
        clamped, changes = clamp_to_limits(positions, self._position_limits)
        if changes:
            self.node.get_logger().info(
                'start state clamped onto joint limits: ' +
                ', '.join(f'{name} {delta:+.6f} rad' for name, delta in changes.items()))
        return clamped, changes

    def start_joints(self, timeout=5.0):
        """Fresh current joint positions, clamped onto the joint limits (see clamp_to_limits)."""
        return self._clamped(self.current_joints(timeout))[0]

    def _start_state(self):
        """
        Start state for a request without an explicit one.

        Normally "the current state" (is_diff), which move_group takes from /joint_states. If a
        joint rests slightly outside its limit, MoveIt would refuse to plan; then the current state
        clamped onto the limit is given explicitly.
        """
        msg = self._joint_state
        current = RobotState()
        current.is_diff = True
        if msg is None or not self._position_limits:
            return current
        clamped, changes = self._clamped(dict(zip(msg.name, msg.position)))
        return self.robot_state(clamped) if changes else current

    def wait_for_servers(self, timeout=30.0, execute=True):
        """Wait for move_group's /move_action, /compute_fk (and /execute_trajectory)."""
        if not self._move_client.wait_for_server(timeout_sec=timeout):
            raise MotionError('move_group action /move_action not available')
        if not self._fk_client.wait_for_service(timeout_sec=timeout):
            raise MotionError('service /compute_fk not available')
        if execute and not self._execute_client.wait_for_server(timeout_sec=timeout):
            raise MotionError('move_group action /execute_trajectory not available')

    # ----- request building -------------------------------------------------------------------

    def request(self, pipeline, planner, goal, start_state=None):
        """Build a MotionPlanRequest; planner '' = default planner of the pipeline."""
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = pipeline
        req.planner_id = planner
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
        # start at the current robot state, clamped onto the joint limits if it is just outside
        req.start_state = self._start_state() if start_state is None else start_state
        return req

    @staticmethod
    def joint_goal(values):
        """Goal constraints for joint values {joint: rad}."""
        goal = Constraints()
        for name, value in values.items():
            goal.joint_constraints.append(JointConstraint(
                joint_name=name, position=value,
                tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0))
        return goal

    @staticmethod
    def pose_goal(pose):
        """Goal constraints for a TCP pose in BASE_FRAME (Pilz uses region pose + orientation)."""
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
        """Absolute RobotState from joint values {joint: rad}, e.g. as planning start state."""
        state = RobotState()
        state.joint_state.name = list(values.keys())
        state.joint_state.position = list(values.values())
        state.is_diff = False
        return state

    # ----- calls ------------------------------------------------------------------------------

    def run(self, request, plan_only):
        """
        Send request to move_group; returns the MoveGroup result (check result.error_code).

        For execution move_group always plans from the current state and ignores an explicit
        start state (e.g. one clamped onto the joint limits by _start_state). Such a request is
        therefore planned first and the planned trajectory executed afterwards; move_group's
        allowed_start_tolerance (0.01 rad) still rejects a start that does not match the robot.
        """
        if not plan_only and not request.start_state.is_diff:
            planned = self._send(self._move_client, self._move_goal(request, plan_only=True))
            if planned.error_code.val != MoveItErrorCodes.SUCCESS:
                return planned
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = planned.planned_trajectory
            executed = self._send(self._execute_client, goal)
            planned.error_code = executed.error_code
            planned.executed_trajectory = planned.planned_trajectory
            return planned
        return self._send(self._move_client, self._move_goal(request, plan_only))

    @staticmethod
    def _move_goal(request, plan_only):
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions(plan_only=plan_only)
        # no changes to the planning scene (an empty, non-diff scene would mean "replace it")
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _send(self, client, goal):
        """Send an action goal, remember it for cancel(), wait for and return its result."""
        handle = wait_for(client.send_goal_async(goal), timeout=10.0)
        if not handle.accepted:
            raise MotionError(f'{type(goal).__name__} rejected')
        with self._goal_lock:
            self._goal_handle = handle
        try:
            return wait_for(handle.get_result_async()).result
        finally:
            with self._goal_lock:
                self._goal_handle = None

    def execute(self, request):
        """Plan and execute request; raise MotionError unless it succeeded."""
        result = self.run(request, plan_only=False)
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'motion failed: {ERROR_NAMES.get(code, code)}', code)
        return result

    def execute_trajectory(self, trajectory):
        """
        Execute a planned JointTrajectory through move_group; MotionError unless it succeeded.

        move_group checks the start against the current state (allowed_start_tolerance) and
        monitors the duration (moveit_controllers.yaml, trajectory_execution).
        """
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = RobotTrajectory(joint_trajectory=trajectory)
        code = self._send(self._execute_client, goal).error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'execution failed: {ERROR_NAMES.get(code, code)}', code)

    def move_to_named(self, name, pipeline=PILZ, planner='PTP'):
        """Move to a named state of the SRDF (default: Pilz PTP)."""
        states = load_named_states()
        return self.execute(self.request(pipeline, planner, self.joint_goal(states[name])))

    def cancel(self):
        """Cancel the running MoveGroup goal, if any (safe to call from any thread)."""
        with self._goal_lock:
            handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    def fk(self, names, positions):
        """TCP pose in BASE_FRAME for the given joint positions."""
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [TCP_LINK]
        req.robot_state.joint_state.name = list(names)
        req.robot_state.joint_state.position = list(positions)
        res = wait_for(self._fk_client.call_async(req), timeout=5.0)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'FK failed: {ERROR_NAMES.get(res.error_code.val)}',
                              res.error_code.val)
        return res.pose_stamped[0].pose

    def current_joints(self, timeout=5.0):
        """Wait for the next /joint_states message and return it as {name: position}."""
        self._joint_state_event.clear()
        if not self._joint_state_event.wait(timeout):
            raise MotionError('no /joint_states received')
        msg = self._joint_state
        return dict(zip(msg.name, msg.position))
