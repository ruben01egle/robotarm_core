"""
control_center mission handshake for MoveIt motions.

On the real robot (and the firmware mock) the joint_trajectory_controller is inactive until
control_center activates it. A MoveIt client therefore has to:

  1. request ACTION_MISSION / TYPE_START with controller_names=["joint_trajectory_controller"]
     (only possible while control_center is ARMED),
  2. send its MoveIt goals,
  3. request ACTION_MISSION / TYPE_STOP with the same node_name.

control_center can also end the mission by itself: STOP by the operator, ERROR, EMERGENCY, or
controller_lost (the controller left the active state on its own; back to ARMED). In every case
the state leaves MISSION. ControlCenterMission notices this on /system_status, cancels the running
MoveIt goal and raises MissionAborted for every further motion.

Usage (node spun in another thread, see moveit_client.spin_in_background):

    moveit = MoveItClient(node)
    with ControlCenterMission(node, moveit) as mission:
        mission.execute(moveit.request(...))
"""

import threading

from moveit_msgs.msg import MoveItErrorCodes
from robotarm_interface.msg import SystemStatus
from robotarm_interface.srv import RequestAction

from robotarm_motion.moveit_client import MotionError, wait_for


class MissionError(RuntimeError):
    """A mission could not be started or stopped."""


class MissionAborted(MissionError):
    """control_center ended the mission (STOP, ERROR, EMERGENCY or controller lost)."""


class ControlCenterMission:
    """Context manager for a control_center mission with a set of ros2_control controllers."""

    def __init__(self, node, moveit, controllers=('joint_trajectory_controller',), arm=False,
                 timeout=5.0):
        """
        Prepare the mission; arm=True arms the hardware first if control_center is CONNECTED.

        arm only arms, it never disarms: disarming deactivates the hardware, which has to be a
        conscious operator decision on the real robot.
        """
        self.node = node
        self.moveit = moveit
        # control_center activates the whole set in one switch request (string[] in the service)
        self.controllers = [controllers] if isinstance(controllers, str) else list(controllers)
        self.arm = arm
        self.timeout = timeout
        self.state = None
        self.aborted = None  # reason, once control_center left the mission on its own
        self.active = False
        # set once /system_status has reported MISSION; before that, a status message published
        # just before the transition may still say ARMED and must not count as an abort
        self._mission_seen = False
        self._status_event = threading.Event()
        self._client = node.create_client(RequestAction, 'request_action')
        node.create_subscription(SystemStatus, 'system_status', self._status_cb, 10)

    def _status_cb(self, msg):
        self.state = msg.state
        self._status_event.set()
        if not self.active or self.aborted is not None:
            return
        if msg.state == 'MISSION':
            self._mission_seen = True
        elif self._mission_seen:
            self.aborted = f'control_center switched to {msg.state}'
            self.node.get_logger().error(f'{self.aborted}: cancelling the running motion')
            self.moveit.cancel()

    def _wait_for_state(self, states, timeout):
        """Wait until /system_status reports one of states; returns the state reached."""
        end = self.node.get_clock().now().nanoseconds + timeout * 1e9
        while self.node.get_clock().now().nanoseconds < end:
            self._status_event.clear()
            if self.state in states:
                return self.state
            self._status_event.wait(0.5)
        raise MissionError(f'control_center is {self.state}, expected one of {states}')

    def _request(self, action_id, request_type):
        if not self._client.wait_for_service(timeout_sec=self.timeout):
            raise MissionError('control_center service /request_action not available')
        req = RequestAction.Request()
        req.node_name = self.node.get_name()
        req.action_id = action_id
        req.request_type = request_type
        req.controller_names = self.controllers
        res = wait_for(self._client.call_async(req), timeout=self.timeout)
        if not res.success:
            raise MissionError(res.message)

    def start(self):
        """Arm if requested, then start the mission (activates the controller)."""
        state = self._wait_for_state(('CONNECTED', 'ARMED'), self.timeout)
        if state == 'CONNECTED':
            if not self.arm:
                raise MissionError('control_center is CONNECTED, arm the robot first')
            self._request(RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_START)
            self._wait_for_state(('ARMED',), self.timeout)
        self._request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_START)
        self.aborted = None
        self._mission_seen = False
        self.active = True
        self._wait_for_state(('MISSION',), self.timeout)
        self.node.get_logger().info(f'mission started, {", ".join(self.controllers)} active')

    def stop(self):
        """Stop the mission (deactivates the controller); no-op if control_center ended it."""
        if not self.active:
            return
        self.active = False
        if self.aborted is not None:
            self.node.get_logger().warn(
                f'mission was ended by control_center ({self.aborted}); nothing to stop')
            return
        self._request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP)
        self.node.get_logger().info(f'mission stopped, {", ".join(self.controllers)} inactive')

    def check(self):
        """Raise MissionAborted if control_center has ended the mission."""
        if self.aborted is not None:
            raise MissionAborted(self.aborted)

    def check_after_failure(self, settle=1.0):
        """
        After a failed motion: wait for control_center's view, then raise MissionAborted if any.

        MoveIt reports a failed execution within milliseconds after the controller is
        deactivated, but control_center notices a lost controller only on its next check (0.2 s)
        and publishes /system_status at 10 Hz. Waiting up to settle seconds for a state other
        than MISSION reports the real cause (STOP, controller lost) instead of the execution error.
        """
        end = self.node.get_clock().now().nanoseconds + settle * 1e9
        while self.aborted is None and self.node.get_clock().now().nanoseconds < end:
            self._status_event.clear()
            self._status_event.wait(0.1)
        self.check()

    def run(self, request, plan_only):
        """MoveItClient.run inside the mission; raises MissionAborted after a STOP."""
        self.check()
        result = self.moveit.run(request, plan_only)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.check_after_failure()
        self.check()
        return result

    def execute(self, request):
        """MoveItClient.execute inside the mission; raises MissionAborted after a STOP."""
        self.check()
        try:
            return self.moveit.execute(request)
        except MotionError:
            self.check_after_failure()  # report the STOP rather than the execution error
            raise

    def __enter__(self):
        """Start the mission."""
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        """Stop the mission, also after an exception."""
        self.stop()
        return False
