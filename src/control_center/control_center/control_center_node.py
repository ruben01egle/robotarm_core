import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from transitions import Machine
from contextlib import nullcontext
from enum import Enum
import threading
import time

from robotarm_interface.srv import RequestAction, SetMode
from robotarm_interface.msg import SystemStatus, RobotarmMode, ModeState

# Values are the SystemStatus.msg constants, so the state is published without conversion
class SystemState(Enum):
        IDLE            = SystemStatus.IDLE
        CONNECTED       = SystemStatus.CONNECTED
        ARMED           = SystemStatus.ARMED
        MISSION         = SystemStatus.MISSION
        CONFIG          = SystemStatus.CONFIG
        STOP            = SystemStatus.STOP
        ERROR           = SystemStatus.ERROR
        EMERGENCY       = SystemStatus.EMERGENCY

# Modes a mission may run in. LIMP, ARMED and STOPPED are reached through arm/disarm, mission end
# and stop; EMERGENCY_OFF only through the emergency.
MISSION_MODES = {
    RobotarmMode.JOINT_STREAM, RobotarmMode.CARTESIAN_JOG, RobotarmMode.TRAJECTORY,
    RobotarmMode.GRAVITY_COMP, RobotarmMode.IMPEDANCE, RobotarmMode.IMPEDANCE_JOINT_STREAM,
    RobotarmMode.IMPEDANCE_JOG,
}

# Firmware modes in which no controller moves the arm: STOP and ERROR leave them as they are
# (requesting STOPPED from LIMP would power the drives)
AT_REST_MODES = {
    RobotarmMode.LIMP, RobotarmMode.STOPPED, RobotarmMode.FAULT, RobotarmMode.EMERGENCY_OFF,
    RobotarmMode.UNKNOWN,
}

# Retry time for STOPPED and EMERGENCY_OFF, which must not fail because of a running switch
SAFETY_RETRY_TIMEOUT = 2.0

MODE_NAMES = {
    getattr(RobotarmMode, name): name
    for name in dir(RobotarmMode) if name.isupper() and isinstance(getattr(RobotarmMode, name), int)
}


def mode_name(mode):
    return MODE_NAMES.get(mode, str(mode))


class ControlCenterNode(Node):

    state: SystemState

    def __init__(self):
        super().__init__('control_center_node')

        states = [
            {
                'name': SystemState.IDLE,
                'on_enter': 'on_enter_idle'
            },
            {'name': SystemState.CONNECTED},
            {'name': SystemState.ARMED},
            {'name': SystemState.MISSION},
            {'name': SystemState.CONFIG},
            {
                'name': SystemState.STOP,
                'on_enter': 'on_enter_stop',
            },
            {
                'name': SystemState.ERROR,
                'on_enter': 'on_enter_error',
            },
            {
                'name': SystemState.EMERGENCY,
                'on_enter': 'on_enter_emergency',
            }
        ]

        transitions = [
            # --- Standard Workflow ---
            {'trigger': 'enter_connect',      'source': SystemState.IDLE,      'dest': SystemState.CONNECTED,   'after': 'on_enter_connected_after'},
            {'trigger': 'exit_connect',       'source': SystemState.CONNECTED, 'dest': SystemState.IDLE},
            {'trigger': 'enter_config',       'source': SystemState.CONNECTED, 'dest': SystemState.CONFIG},
            {'trigger': 'exit_config',        'source': SystemState.CONFIG,    'dest': SystemState.CONNECTED},
            {'trigger': 'enter_arm',          'source': SystemState.CONNECTED, 'dest': SystemState.ARMED,       'before': 'on_enter_arm_before'},
            {'trigger': 'exit_arm',           'source': [SystemState.ARMED, SystemState.STOP], 'dest': SystemState.CONNECTED, 'before': 'on_exit_arm_before'},
            {'trigger': 'enter_mission',      'source': SystemState.ARMED,     'dest': SystemState.MISSION,     'before': 'on_enter_mission_before'},
            {'trigger': 'exit_mission',       'source': SystemState.MISSION,   'dest': SystemState.ARMED,       'before': 'on_exit_mission_before'},
            {'trigger': 'controller_lost',    'source': SystemState.MISSION,   'dest': SystemState.STOP,        'before': 'on_controller_lost_before'},

            # --- STOPS & Recovery ---
            {'trigger': 'enter_stop',         'source': SystemState.MISSION,   'dest': SystemState.STOP},
            {'trigger': 'exit_stop',          'source': SystemState.STOP,      'dest': SystemState.ARMED,       'before': 'on_exit_stop_before'},

            # --- Global Error ---
            {'trigger': 'enter_error',     'source': '*', 'dest': SystemState.ERROR},

            # --- Global Emergency ---
            {'trigger': 'enter_emergency', 'source': '*', 'dest': SystemState.EMERGENCY}
        ]
        
        self.machine = Machine(model=self, states=states, transitions=transitions, initial=SystemState.IDLE, send_event=True)

        self.multithread_cb_group = ReentrantCallbackGroup()      

        # The firmware mode manager does all controller and hardware switching
        self.set_mode_client = self.create_client(
            SetMode,
            '/mode_manager/set_mode',
            callback_group=self.multithread_cb_group
        )
        # The firmware rejects a request while another one is running (except EMERGENCY_OFF)
        self.set_mode_lock = threading.Lock()

        # Actual mode of the firmware, latched: reliable + transient local to get it on connect
        self.mode_state = None
        self.mode_state_sub = self.create_subscription(
            ModeState,
            '/mode_manager/state',
            self.mode_state_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.multithread_cb_group
        )

        self.startup_timer = self.create_timer(
            1.0, 
            self.check_hardware_connection, 
            callback_group=self.multithread_cb_group
        )

        self.watchdog_timer = self.create_timer(
            0.2, 
            self.watchdog_cb, 
            callback_group=self.multithread_cb_group
        )
        self.last_heartbeat_time = None
        
        self.joint_state_sub = None

        self.request_action_srv = self.create_service(
            RequestAction, 
            'request_action', 
            self.request_action_cb, 
            callback_group=self.multithread_cb_group
        )

        # Mode of the running mission (RobotarmMode value), None = no mission
        self.active_mode = None
        self.active_node = None

        # Mission monitoring on /mode_manager/state: detects when the firmware leaves the mission
        # mode on its own (e.g. after a controller error it goes to STOPPED)
        self.switching_mode = False
        self.mission_mode_seen = False

        self.status_pub = self.create_publisher(
            SystemStatus,
            '/system_status',
            1,
            callback_group=self.multithread_cb_group
        )

        self.status_timer = self.create_timer(
            0.1,
            self.publish_status_cb,
            callback_group=self.multithread_cb_group
        )

        self.get_logger().info('Control center node running')

    def check_hardware_connection(self):
        if self.state != SystemState.IDLE:
            return
        self.get_logger().info("Checking for firmware mode manager")
        if self.mode_state is not None and self.mode_state.mode.mode != RobotarmMode.UNKNOWN:
            self.get_logger().info(f"Detected firmware, mode {mode_name(self.mode_state.mode.mode)}")
            self.startup_timer.cancel()
            self.enter_connect() # type: ignore

    def watchdog_cb(self):
        if self.state in [SystemState.CONNECTED, SystemState.ARMED, SystemState.MISSION]:
            if not self.last_heartbeat_time:
                return
            if (time.time() - self.last_heartbeat_time) > 0.5:
                self.get_logger().error("Watchdog: Heartbeat lost!")
                self.enter_error() # type: ignore

    def on_enter_idle(self, event_data=None):
        self.get_logger().info("State: IDLE. Waiting for hardware...")
        if self.joint_state_sub:
            self.destroy_subscription(self.joint_state_sub)
            self.joint_state_sub = None
        self.startup_timer.reset()

    def on_enter_connected_after(self, event_data=None):
        self.get_logger().info("State: CONNECTED. Starting heartbeat watchod via joint state broadcaster")
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1,
            callback_group=self.multithread_cb_group
        )

    def joint_state_callback(self, msg):
        self.last_heartbeat_time = time.time()

    def mode_state_cb(self, msg: ModeState):
        self.mode_state = msg
        mode = msg.mode.mode

        # Someone else requested EMERGENCY_OFF: the hardware is finalized anyway
        if mode == RobotarmMode.EMERGENCY_OFF and self.state != SystemState.EMERGENCY:
            self.get_logger().fatal("Firmware reports EMERGENCY_OFF")
            self.enter_emergency() # type: ignore
            return

        if self.state != SystemState.MISSION or self.active_mode is None:
            return
        if self.switching_mode or msg.switching:
            return
        if mode == self.active_mode:
            self.mission_mode_seen = True
        # Before the mission mode has been seen, an ARMED may still be the stale state from
        # before the switch. Any other mode means the mission was left (e.g. FAULT, STOPPED).
        elif self.mission_mode_seen or mode != RobotarmMode.ARMED:
            self.controller_lost(mode=mode) # type: ignore

    def set_mode(self, target, retry_timeout=0.0):
        """
        Request a mode from the firmware mode manager and wait for the result.

        Rejected requests are repeated until retry_timeout. Raises RuntimeError on failure.
        """
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Service '/mode_manager/set_mode' not active!")
        request = SetMode.Request()
        request.target.mode = target

        # EMERGENCY_OFF is never rejected by the firmware, it aborts a running request
        emergency = target == RobotarmMode.EMERGENCY_OFF
        deadline = time.time() + retry_timeout
        with nullcontext() if emergency else self.set_mode_lock:
            if not emergency:
                self.switching_mode = True
            try:
                rate = self.create_rate(10)
                try:
                    while True:
                        future = self.set_mode_client.call_async(request)
                        while not future.done(): rate.sleep()
                        response = future.result()
                        if response is not None and response.success:
                            break
                        reason = response.message if response is not None else "no response"
                        if time.time() >= deadline:
                            actual = mode_name(response.mode.mode) if response is not None else "unknown"
                            raise RuntimeError(f"Mode {mode_name(target)} rejected: {reason} (mode is {actual})")
                        self.get_logger().warn(f"Mode {mode_name(target)} rejected ({reason}), retrying...")
                        rate.sleep()
                finally:
                    self.destroy_rate(rate)
            finally:
                if not emergency:
                    self.switching_mode = False
        self.get_logger().info(f"Firmware mode {mode_name(target)}")

    def stop_motion(self):
        """Switch the firmware to STOPPED if a controller moves the arm. Never raises."""
        mode = self.mode_state.mode.mode if self.mode_state is not None else RobotarmMode.UNKNOWN
        if mode in AT_REST_MODES:
            self.get_logger().info(f"Firmware is {mode_name(mode)}. System is already stationary.")
        else:
            self.get_logger().info(f"Safety stop: requesting STOPPED (firmware was {mode_name(mode)})...")
            try:
                self.set_mode(RobotarmMode.STOPPED, retry_timeout=SAFETY_RETRY_TIMEOUT)
            except Exception as e:
                self.get_logger().error(f"Failed to stop: {e}")
        self.active_node = None
        self.active_mode = None

    def on_enter_arm_before(self, event_data=None):
        # On failure the firmware returns to LIMP
        self.set_mode(RobotarmMode.ARMED)
        self.get_logger().info("Robot armed")

    def on_exit_arm_before(self, event_data=None):
        self.set_mode(RobotarmMode.LIMP)
        self.active_node = None
        self.active_mode = None
        self.get_logger().info("Robot disarmed")

    def on_enter_mission_before(self, event_data):
        node_name = event_data.kwargs.get('node_name', 'unknown_node')
        mode = event_data.kwargs.get('mode')

        if mode not in MISSION_MODES:
            raise RuntimeError(f"Rejected: node '{node_name}' requested a mission in mode {mode_name(mode)}.")

        self.get_logger().info(f"Preparing mission for node '{node_name}' in mode {mode_name(mode)}...")
        self.mission_mode_seen = False
        self.set_mode(mode)

        self.active_node = node_name
        self.active_mode = mode
        self.get_logger().info(f"Mission of '{node_name}' running in mode {mode_name(mode)}.")

    def on_exit_mission_before(self, event_data):
        node_name = event_data.kwargs.get('node_name', 'unknown_node')
        
        if self.active_node != node_name:
            raise RuntimeError(f"Rejected: active node is {self.active_node}. You ({node_name}) cannot stop this mission.")

        self.get_logger().info(f"Stopping mission ({mode_name(self.active_mode)})...")
        self.set_mode(RobotarmMode.ARMED)

        self.active_node = None
        self.active_mode = None
        self.get_logger().info("Mission successfully stopped.")

    def on_controller_lost_before(self, event_data):
        mode = event_data.kwargs.get('mode')
        self.get_logger().error(
            f"Firmware left mission mode {mode_name(self.active_mode)} unexpectedly (now {mode_name(mode)}). "
            f"Mission of '{self.active_node}' aborted. See controller_manager log for reason.")
        # Nothing to request: after a controller error the firmware cleans up to STOPPED itself.
        # Resuming (ARMED) or disarming (LIMP) is the operator's decision.
        self.active_node = None
        self.active_mode = None

    def on_enter_stop(self, event_data=None):
        if event_data is not None and event_data.event.name == 'controller_lost':
            return
        self.get_logger().warn("STOP INVOKED: Stopping all motion for safety...")
        self.stop_motion()

    def on_exit_stop_before(self, event_data=None):
        # The drives held on their own PID; with the feedforward back, the arm may lift slightly
        self.set_mode(RobotarmMode.ARMED)
        self.get_logger().info("Resumed, robot armed")

    def on_enter_error(self, event_data=None):
        self.get_logger().fatal("ERROR STATE INVOKED: Hard-stopping robot by stopping all motion...")
        self.stop_motion()

    def on_enter_emergency(self, event_data=None):
        self.get_logger().fatal("EMERGENCY STATE INVOKED")
        self.active_node = None
        self.active_mode = None
        self.set_mode(RobotarmMode.EMERGENCY_OFF, retry_timeout=SAFETY_RETRY_TIMEOUT)

    def request_action_cb(self, request: RequestAction.Request, response: RequestAction.Response):
        action_map = {
            (RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_START): "enter_arm",
            (RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_STOP):  "exit_arm",
            
            (RequestAction.Request.ACTION_CONFIGURE, RequestAction.Request.TYPE_START): "enter_config",
            (RequestAction.Request.ACTION_CONFIGURE, RequestAction.Request.TYPE_STOP):  "exit_config",
            
            (RequestAction.Request.ACTION_MISSION,   RequestAction.Request.TYPE_START): "enter_mission",
            (RequestAction.Request.ACTION_MISSION,   RequestAction.Request.TYPE_STOP):  "exit_mission",

            (RequestAction.Request.INVOKE_STOP,      RequestAction.Request.TYPE_START): "enter_stop",
            (RequestAction.Request.INVOKE_STOP,      RequestAction.Request.TYPE_STOP):  "exit_stop",

            (RequestAction.Request.INVOKE_EMERGENCY, RequestAction.Request.TYPE_START): "enter_emergency",
            (RequestAction.Request.INVOKE_EMERGENCY, RequestAction.Request.TYPE_STOP):  "enter_emergency"
        }

        key = (request.action_id, request.request_type)
        
        if key not in action_map:
            response.message = f"Invalid Action/Type Combination: {key}"
            response.success = False
            return response

        trigger_fctn = action_map[key]

        try:
            if (request.action_id == RequestAction.Request.ACTION_MISSION):
                getattr(self, trigger_fctn)(node_name=request.node_name, mode=request.mode.mode)
            else:
                getattr(self, trigger_fctn)()

            response.success = True
            response.message = f"Successfully triggered {trigger_fctn}. Current State: {self.state}"
            self.get_logger().debug(f"Triggered {trigger_fctn}")

        except Exception as e:
            response.message = f"TRANSITION FAILED: {str(e)}"
            response.success = False

        return response
    
    def publish_status_cb(self):
        try:
            msg = SystemStatus()
            msg.state = self.state.value
            
            msg.connected = self.state != SystemState.IDLE and self.state != SystemState.ERROR
            # STOP is powered too (drives hold), disarming is possible from there
            msg.armed = self.state in [SystemState.ARMED, SystemState.MISSION, SystemState.STOP]
            msg.stopped = self.state in [SystemState.STOP]
            
            msg.mode.mode = self.mode_state.mode.mode if self.mode_state is not None else RobotarmMode.UNKNOWN
            msg.active_node = self.active_node if self.active_node else ""

            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing system state: {e}", throttle_duration_sec=2.0)

    def perform_cleanup(self):
        """
        Safely secures the robot hardware on node shutdown, BUT ONLY if it was
        running or armed. Prevents bypassing EMERGENCY (EMERGENCY_OFF) or ERROR states.
        """
        if self.state in [SystemState.IDLE, SystemState.ERROR, SystemState.EMERGENCY]:
            self.get_logger().info(f"Shutdown: System already in safe state ({self.state.name}). Skipping cleanup.")
            return

        self.get_logger().warn(f"Unexpected shutdown from state {self.state.name}! Securing hardware...")

        try:
            if self.set_mode_client.service_is_ready():
                self.get_logger().info("Requesting LIMP...")
                request = SetMode.Request()
                request.target.mode = RobotarmMode.LIMP
                future = self.set_mode_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                self.get_logger().info("Robot set to LIMP.")
        except Exception as e:
            print(f"[Shutdown Cleanup] Failed to disarm: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    ros_node = ControlCenterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()