import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from controller_manager_msgs.srv import ListHardwareInterfaces
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchController_Request as SwitchReq
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State as LifecycleState

from transitions import Machine
from enum import Enum
import time

from robotarm_interface.srv import RequestAction
from robotarm_interface.msg import SystemStatus

from .switching import (FEEDFORWARD_CONTROLLER, FEEDFORWARD_UPSTREAMS, switch_request,
                        with_feedforward)

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

class ControlCenterNode(Node):

    state: SystemState

    def __init__(self):
        super().__init__('control_center_node')

        self.declare_parameter('hardware_component_name', 'test') 
        # self.declare_parameter('hold_position_controller_name', 'my_hold_position_controller')

        self.hardware_name = self.get_parameter('hardware_component_name').get_parameter_value().string_value
        # self.hold_controller_name = self.get_parameter('hold_position_controller_name').get_parameter_value().string_value

        self.get_logger().info(f"Loaded Hardware Component Name: '{self.hardware_name}'")

        # The position-controlled controllers are chained onto the feedforward controller;
        # control_center switches it together with them (see switching.py)
        self.declare_parameter('feedforward_controller', FEEDFORWARD_CONTROLLER)
        self.declare_parameter('feedforward_upstreams', list(FEEDFORWARD_UPSTREAMS))
        self.ff_controller = self.get_parameter('feedforward_controller').value
        self.ff_upstreams = tuple(self.get_parameter('feedforward_upstreams').value)
        # self.get_logger().info(f"Loaded Hold-Position Controller: '{self.hold_controller_name}'")

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
            {'trigger': 'exit_arm',           'source': SystemState.ARMED,     'dest': SystemState.CONNECTED,   'after': 'on_exit_arm_after'},
            {'trigger': 'enter_mission',      'source': SystemState.ARMED,     'dest': SystemState.MISSION,     'before': 'on_enter_mission_before'},
            {'trigger': 'exit_mission',       'source': SystemState.MISSION,   'dest': SystemState.ARMED,       'before': 'on_exit_mission_before'},
            {'trigger': 'controller_lost',    'source': SystemState.MISSION,   'dest': SystemState.ARMED,       'before': 'on_controller_lost_before'},

            # --- STOPS & Recovery ---
            {'trigger': 'enter_stop',         'source': SystemState.MISSION,   'dest': SystemState.STOP},
            {'trigger': 'exit_stop',          'source': SystemState.STOP,      'dest': SystemState.ARMED},

            # --- Global Error ---
            {'trigger': 'enter_error',     'source': '*', 'dest': SystemState.ERROR},

            # --- Global Emergency ---
            {'trigger': 'enter_emergency', 'source': '*', 'dest': SystemState.EMERGENCY}
        ]
        
        self.machine = Machine(model=self, states=states, transitions=transitions, initial=SystemState.IDLE, send_event=True)

        self.multithread_cb_group = ReentrantCallbackGroup()      

        self.cm_client = self.create_client(
            ListHardwareInterfaces, 
            '/controller_manager/list_hardware_interfaces', 
            callback_group=self.multithread_cb_group
        )
        self.switch_ctrl_client = self.create_client(
            SwitchController, 
            '/controller_manager/switch_controller', 
            callback_group=self.multithread_cb_group
        )
        self.hw_state_client = self.create_client(
            SetHardwareComponentState,
            '/controller_manager/set_hardware_component_state',
            callback_group=self.multithread_cb_group
        )
        self.list_ctrl_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
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

        # Controller set of the mission, (de)activated in a single switch request,
        # e.g. [cartesian_impedance_controller, joint_stream_controller_impedance] when chained
        self.active_controllers = []
        self.active_node = None

        # Controller monitoring: detects when the controller_manager deactivates a mission controller
        # on its own (e.g. after an error in the controller)
        self.switching_controller = False
        self.controller_check_pending = False
        self.controller_monitor_timer = self.create_timer(
            0.2,
            self.controller_monitor_cb,
            callback_group=self.multithread_cb_group
        )

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
        self.get_logger().info("Checking for hw")
        if self.cm_client.service_is_ready():
            self.get_logger().info("Detected hardware")
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

    def on_enter_arm_before(self, event_data=None):
        if not self.hw_state_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Service 'set_hardware_component_state' not active!")
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_name
        request.target_state.id = LifecycleState.PRIMARY_STATE_ACTIVE

        rate = self.create_rate(10)
        future = self.hw_state_client.call_async(request)
        while not future.done(): rate.sleep()
        response = future.result()
        if response is None or not response.ok:
            raise RuntimeError(f"Failed to arm hardware.")
            
        self.get_logger().info("Hardware-Interface armed")

    def on_exit_arm_after(self, event_data=None):
        if not self.hw_state_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Service 'set_hardware_component_state' not active!")
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_name
        request.target_state.id = LifecycleState.PRIMARY_STATE_INACTIVE

        rate = self.create_rate(10)
        future = self.hw_state_client.call_async(request)
        while not future.done(): rate.sleep()
        response = future.result()
        if response is None or not response.ok:
            raise RuntimeError(f"Failed to disarm hardware.")
            
        self.get_logger().info("Hardware-Interface disarmed")

    def on_enter_mission_before(self, event_data):
        node_name = event_data.kwargs.get('node_name', 'unknown_node')
        requested = list(event_data.kwargs.get('controller_names', []))

        if not requested:
            raise RuntimeError(f"Rejected: node '{node_name}' requested a mission without controllers.")

        # Position-controlled controllers only run together with the feedforward controller
        controller_names = with_feedforward(requested, self.ff_controller, self.ff_upstreams)
        if self.ff_controller in controller_names and self.ff_controller not in requested:
            self.check_loaded(self.ff_controller, needed_for=requested)

        self.get_logger().info(f"Preparing mission for node '{node_name}'. Activating controllers {controller_names}...")

        if not self.switch_ctrl_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Service '/controller_manager/switch_controller' not active!")

        # All controllers of the set in ONE request (STRICT: all or nothing), downstream first
        request = SwitchController.Request()
        request.activate_controllers, request.deactivate_controllers = \
            switch_request(self.active_controllers, controller_names)
        request.strictness = SwitchReq.STRICT
        request.activate_asap = True

        self.switching_controller = True
        try:
            rate = self.create_rate(10)
            future = self.switch_ctrl_client.call_async(request)
            while not future.done():
                rate.sleep()

            response = future.result()
            if response is None or not response.ok:
                raise RuntimeError(f"Failed to activate controllers: {controller_names}")

            self.active_node = node_name
            self.active_controllers = controller_names
        finally:
            self.switching_controller = False
        self.get_logger().info(f"Successfully activated controllers {controller_names} for mission.")

    def on_exit_mission_before(self, event_data):
        node_name = event_data.kwargs.get('node_name', 'unknown_node')
        
        if self.active_node != node_name:
            raise RuntimeError(f"Rejected: active node is {self.active_node}. You ({node_name}) cannot stop this mission.")
            
        if self.active_controllers:
            self.get_logger().info(f"Stopping active mission controllers {self.active_controllers}...")
            
            if not self.switch_ctrl_client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service '/controller_manager/switch_controller' not active!")

            # Whole set in one request: chaining does not allow deactivating the downstream controller alone
            request = SwitchController.Request()
            request.activate_controllers, request.deactivate_controllers = \
                switch_request(self.active_controllers, [])
            request.strictness = SwitchReq.STRICT

            self.switching_controller = True
            try:
                rate = self.create_rate(20)
                future = self.switch_ctrl_client.call_async(request)
                while not future.done():
                    rate.sleep()

                response = future.result()
                if response is None or not response.ok:
                    raise RuntimeError(f"Failed to deactivate controllers: {self.active_controllers}")

                # Reset while the flag is still set, so the monitor does not raise a false alarm
                self.active_controllers = []
            finally:
                self.switching_controller = False

        self.active_node = None
        self.active_controllers = []
        self.get_logger().info("Mission controllers successfully stopped.")

    def check_loaded(self, controller, needed_for):
        """Raise RuntimeError unless controller is loaded in the controller_manager."""
        if not self.list_ctrl_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Service '/controller_manager/list_controllers' not active!")
        rate = self.create_rate(20)
        future = self.list_ctrl_client.call_async(ListControllers.Request())
        while not future.done():
            rate.sleep()
        response = future.result()
        if response is None or controller not in [c.name for c in response.controller]:
            raise RuntimeError(f"Rejected: {controller} is not loaded on the controller_manager "
                               f"(needed for {', '.join(needed_for)}).")

    def controller_monitor_cb(self):
        """Check periodically whether all controllers of the mission are still active."""
        if self.state != SystemState.MISSION or not self.active_controllers:
            return
        if self.switching_controller or self.controller_check_pending:
            return
        if not self.list_ctrl_client.service_is_ready():
            return

        self.controller_check_pending = True
        expected_controllers = list(self.active_controllers)
        future = self.list_ctrl_client.call_async(ListControllers.Request())
        future.add_done_callback(lambda f: self.controller_monitor_response_cb(f, expected_controllers))

    def controller_monitor_response_cb(self, future, expected_controllers):
        try:
            response = future.result()
            if response is None:
                return

            # The state may have changed during the query (e.g. a regular disengage)
            if (self.state != SystemState.MISSION or self.switching_controller
                    or self.active_controllers != expected_controllers):
                return

            states = {ctrl.name: ctrl.state for ctrl in response.controller}
            lost = {name: states.get(name, "not loaded") for name in expected_controllers
                    if states.get(name) != "active"}

            if lost:
                still_active = [name for name in expected_controllers if name not in lost]
                self.controller_lost(lost=lost, still_active=still_active) # type: ignore
        except Exception as e:
            self.get_logger().error(f"Controller monitor failed: {e}", throttle_duration_sec=2.0)
        finally:
            self.controller_check_pending = False

    def on_controller_lost_before(self, event_data):
        lost = event_data.kwargs.get('lost', {})
        still_active = event_data.kwargs.get('still_active', [])
        lost_text = ", ".join(f"'{name}' ({state})" for name, state in lost.items())
        self.get_logger().error(
            f"Controller {lost_text} left active state unexpectedly. "
            f"Mission of '{self.active_node}' aborted. See controller_manager log for reason.")

        # Deactivate the rest of the set too, so the result is always a defined state (ARMED, drives hold)
        if still_active:
            self.active_controllers = still_active
            self.deactivate_active_controller()

        self.active_node = None
        self.active_controllers = []

    def on_enter_stop(self, event_data=None):
        self.get_logger().warn("STOP INVOKED: Deactivating all active controllers for safety...")
        self.deactivate_active_controller()

    def on_enter_error(self, event_data=None):
        self.get_logger().fatal("ERROR STATE INVOKED: Hard-stopping robot by dropping controllers...")
        self.deactivate_active_controller()

    def on_enter_emergency(self, event_data=None):
        self.get_logger().fatal("EMERGENCY STATE INVOKED")
        if not self.hw_state_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Service 'set_hardware_component_state' not active!")
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_name
        request.target_state.id = LifecycleState.PRIMARY_STATE_FINALIZED

        rate = self.create_rate(10)
        future = self.hw_state_client.call_async(request)
        while not future.done(): rate.sleep()
        response = future.result()
        if response is None or not response.ok:
            raise RuntimeError(f"Failed to deactivate hardware")

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
                getattr(self, trigger_fctn)(node_name=request.node_name, controller_names=list(request.controller_names))
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
            msg.armed = self.state in [SystemState.ARMED, SystemState.MISSION]
            msg.stopped = self.state in [SystemState.STOP]
            
            msg.active_controllers = list(self.active_controllers)
            msg.active_node = self.active_node if self.active_node else ""

            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing system state: {e}", throttle_duration_sec=2.0)

    def deactivate_active_controller(self, blocking=False):
        """Helper method to turn off the currently running controller set immediately."""
        if not self.active_controllers:
            self.get_logger().info("No active controller running. System is already stationary.")
            return

        self.get_logger().info(f"Safety shutdown: Deactivating controllers {self.active_controllers}...")

        if not self.switch_ctrl_client.service_is_ready():
            self.get_logger().error("Service '/controller_manager/switch_controller' not ready!")
            return

        request = SwitchController.Request()
        request.activate_controllers, request.deactivate_controllers = \
            switch_request(self.active_controllers, [])
        request.strictness = SwitchReq.BEST_EFFORT

        try:
            if blocking:
                response = self.switch_ctrl_client.call(request)
                if response is None or not response.ok:
                    self.get_logger().error(f"Failed to deactivate controllers {self.active_controllers}!")
                else:
                    self.get_logger().info(f"Successfully deactivated controllers {self.active_controllers}.")
            else:
                rate = self.create_rate(20)
                future = self.switch_ctrl_client.call_async(request)
                while not future.done(): 
                    rate.sleep()
                    
                response = future.result()
                if response is None or not response.ok:
                    self.get_logger().error(f"Failed to deactivate controllers {self.active_controllers}!")
                else:
                    self.get_logger().info(f"Successfully deactivated controllers {self.active_controllers}. Hardware stopped.")
                
        except Exception as e:
            self.get_logger().error(f"Exception during safety controller switch: {e}")

        self.active_node = None
        self.active_controllers = []

    def perform_cleanup(self):
        """
        Safely secures the robot hardware on node shutdown, BUT ONLY if it was
        running or armed. Prevents bypassing EMERGENCY (FINALIZED) or ERROR states.
        """
        if self.state in [SystemState.IDLE, SystemState.ERROR, SystemState.EMERGENCY]:
            self.get_logger().info(f"Shutdown: System already in safe state ({self.state.name}). Skipping cleanup.")
            return

        self.get_logger().warn(f"Unexpected shutdown from state {self.state.name}! Securing hardware...")
        
        self.deactivate_active_controller(blocking=True)

        try:
            if self.hw_state_client.service_is_ready():
                self.get_logger().info(f"Setting hardware '{self.hardware_name}' to INACTIVE...")
                request = SetHardwareComponentState.Request()
                request.name = self.hardware_name
                request.target_state.id = LifecycleState.PRIMARY_STATE_INACTIVE
                self.hw_state_client.call(request)
                self.get_logger().info("Hardware successfully set to INACTIVE.")
        except Exception as e:
            print(f"[Shutdown Cleanup] Failed to deactivate hardware: {e}")

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