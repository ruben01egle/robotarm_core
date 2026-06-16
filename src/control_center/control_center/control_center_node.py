import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from controller_manager_msgs.srv import ListHardwareInterfaces
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchController_Request as SwitchReq
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State as LifecycleState

from transitions import Machine
from enum import Enum
import time

from robotarm_interface.srv import RequestAction
from robotarm_interface.msg import SystemStatus

class SystemState(Enum):
        IDLE            = 0x0
        CONNECTED       = 0x1
        ARMED           = 0x2
        MISSION         = 0x3
        CONFIG          = 0x4
        STOP            = 0x5
        ERROR           = 0x66
        EMERGENCY       = 0xFF

class ControlCenterNode(Node):

    state: SystemState

    def __init__(self):
        super().__init__('control_center_node')

        self.declare_parameter('hardware_component_name', 'test') 
        # self.declare_parameter('hold_position_controller_name', 'my_hold_position_controller')

        self.hardware_name = self.get_parameter('hardware_component_name').get_parameter_value().string_value
        # self.hold_controller_name = self.get_parameter('hold_position_controller_name').get_parameter_value().string_value

        self.get_logger().info(f"Loaded Hardware Component Name: '{self.hardware_name}'")
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

        self.active_controller = None
        self.active_node = None

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
        controller_name = event_data.kwargs.get('controller_name', 'default')

        self.get_logger().info(f"Preparing mission for node '{node_name}'. Activating controller '{controller_name}'...")

        if not self.switch_ctrl_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Service '/controller_manager/switch_controller' not active!")

        request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.deactivate_controllers = []
        request.strictness = SwitchReq.STRICT
        request.activate_asap = True

        rate = self.create_rate(10)
        future = self.switch_ctrl_client.call_async(request)
        while not future.done(): 
            rate.sleep()
            
        response = future.result()
        if response is None or not response.ok:
            raise RuntimeError(f"Failed to activate controller: {controller_name}")
            
        self.active_node = node_name
        self.active_controller = controller_name
        self.get_logger().info(f"Successfully activated controller '{controller_name}' for mission.")

    def on_exit_mission_before(self, event_data):
        node_name = event_data.kwargs.get('node_name', 'unknown_node')
        controller_name = event_data.kwargs.get('controller_name', 'default')
        
        if self.active_node != node_name:
            raise RuntimeError(f"Rejected: active node is {self.active_node}. You ({node_name}) cannot stop this mission.")
            
        if self.active_controller:
            self.get_logger().info(f"Stopping active mission controller '{self.active_controller}'...")
            
            if not self.switch_ctrl_client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service '/controller_manager/switch_controller' not active!")

            request = SwitchController.Request()
            request.activate_controllers = []
            request.deactivate_controllers = [self.active_controller]
            request.strictness = SwitchReq.STRICT

            rate = self.create_rate(20)
            future = self.switch_ctrl_client.call_async(request)
            while not future.done(): 
                rate.sleep()
                
            response = future.result()
            if response is None or not response.ok:
                raise RuntimeError(f"Failed to deactivate controller: {self.active_controller}")

        self.active_node = None
        self.active_controller = None
        self.get_logger().info("Mission controller successfully stopped.")

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
                getattr(self, trigger_fctn)(node_name=request.node_name, controller_name=request.controller_names)
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
            msg.state = str(self.state.name)
            
            msg.connected = self.state != SystemState.IDLE and self.state != SystemState.ERROR
            msg.armed = self.state in [SystemState.ARMED, SystemState.MISSION]
            msg.stopped = self.state in [SystemState.STOP]
            
            msg.active_controllers = self.active_controller if self.active_controller else ""
            msg.active_nodes = self.active_node if self.active_node else ""

            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing system state: {e}", throttle_duration_sec=2.0)

    def deactivate_active_controller(self, blocking=False):
        """Helper method to turn off the currently running controller immediately."""
        if not self.active_controller:
            self.get_logger().info("No active controller running. System is already stationary.")
            return

        self.get_logger().info(f"Safety shutdown: Deactivating controller '{self.active_controller}'...")

        if not self.switch_ctrl_client.service_is_ready():
            self.get_logger().error("Service '/controller_manager/switch_controller' not ready!")
            return

        request = SwitchController.Request()
        request.activate_controllers = []
        request.deactivate_controllers = [self.active_controller]
        request.strictness = SwitchReq.BEST_EFFORT 

        try:
            if blocking:
                response = self.switch_ctrl_client.call(request)
                if response is None or not response.ok:
                    self.get_logger().error(f"Failed to deactivate controller '{self.active_controller}'!")
                else:
                    self.get_logger().info(f"Successfully deactivated controller '{self.active_controller}'.")
            else:
                rate = self.create_rate(20)
                future = self.switch_ctrl_client.call_async(request)
                while not future.done(): 
                    rate.sleep()
                    
                response = future.result()
                if response is None or not response.ok:
                    self.get_logger().error(f"Failed to deactivate controller '{self.active_controller}'!")
                else:
                    self.get_logger().info(f"Successfully deactivated controller '{self.active_controller}'. Hardware stopped.")
                
        except Exception as e:
            self.get_logger().error(f"Exception during safety controller switch: {e}")

        self.active_node = None
        self.active_controller = None

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