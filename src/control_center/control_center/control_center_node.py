import rclpy
from rclpy.time import Time
from transitions import Machine
from rclpy.node import Node
from enum import IntEnum
import socket

from interface.msg import SystemState, Heartbeat, HeartbeatQuery, HardwareActions, HardwareCommand, HardwareFeedback
from interface.srv import RequestAction

class ControlCenterNode(Node):
    class State(IntEnum):
        IDLE            = SystemState.IDLE
        CONNECTED       = SystemState.CONNECTED
        ARMED           = SystemState.ARMED
        MISSION         = SystemState.MISSION
        CONFIG          = SystemState.CONFIG
        SOFT_STOP       = SystemState.SOFT_STOP
        HARD_STOP       = SystemState.HARD_STOP
        ERROR           = SystemState.ERROR
        EMERGENCY       = SystemState.EMERGENCY

    state: State

    def __init__(self):
        super().__init__('control_center_node')

        self.heartbeat_query_id = 0
        self.LATENCY_WARNING_THRESHOLD = 0.1
        self.LATENCY_ERROR_THRESHOLD = 0.2
        self.MAX_MISSED_QUERIES = 5
        # Structure: { 'node_name': {'last_query_id': int, 'latency': float} }
        self.tracked_nodes = {}

        self.HADWARE_SIGNATURE = "stm32"

        self.command_id = 1             # Start with 1 to prevent accidents due to standard value=0
        self.pending_command = False

        transitions = [
            # --- Standard Workflow ---
            {'trigger': 'connect',          'source': self.State.IDLE,      'dest': self.State.CONNECTED},
            {'trigger': 'disconnect',       'source': self.State.CONNECTED, 'dest': self.State.IDLE},
            {'trigger': 'enter_config',     'source': self.State.CONNECTED, 'dest': self.State.CONFIG},
            {'trigger': 'exit_config',      'source': self.State.CONFIG,    'dest': self.State.CONNECTED},
            {'trigger': 'arm',              'source': self.State.CONNECTED, 'dest': self.State.ARMED},
            {'trigger': 'disarm',           'source': self.State.ARMED,     'dest': self.State.CONNECTED},
            {'trigger': 'start_mission',    'source': self.State.ARMED,     'dest': self.State.MISSION},
            {'trigger': 'end_mission',      'source': self.State.MISSION,   'dest': self.State.ARMED},

            # --- STOPS & Recovery ---
            {'trigger': 'soft_stop',          'source': self.State.MISSION,   'dest': self.State.SOFT_STOP},
            {'trigger': 'soft_stop_complete', 'source': self.State.SOFT_STOP, 'dest': self.State.ARMED},
            {'trigger': 'hard_stop',          'source': self.State.MISSION,   'dest': self.State.HARD_STOP},
            {'trigger': 'hard_stop_complete', 'source': self.State.HARD_STOP, 'dest': self.State.ARMED},

            # --- Global Error ---
            {'trigger': 'enter_error',     'source': '*', 'dest': self.State.ERROR},

            # --- Global Emergency ---
            {'trigger': 'enter_emergency', 'source': '*', 'dest': self.State.EMERGENCY}
        ]
        
        self.machine = Machine(model=self, states=self.State, transitions=transitions, initial=self.State.CONNECTED)

        self.create_subscription(Heartbeat, 'heartbeat/response', self.heartbeat_cb, 20)
        self.create_subscription(HardwareFeedback, 'hardware/feedback', self.hardware_feedback_cb, 3)

        self.state_pub = self.create_publisher(SystemState, 'system_state', 1)
        self.command_pub = self.create_publisher(HardwareCommand, 'hardware/command', 1)
        self.heartbeat_query_pub = self.create_publisher(HeartbeatQuery, 'heartbeat/query', 1)

        self.request_action_srv = self.create_service(
            RequestAction, 
            'request_action', 
            self.request_action_cb
        )

        self.timer_state_pub = self.create_timer(0.5, self.publish_state)
        self.timer_heartbeat_pub = self.create_timer(0.5, self.publish_heartbeat_query)
        self.timer_watchdog = self.create_timer(1.0, self.watchdog_cb)

        self.get_logger().info('Control center node running')

    def publish_state(self):
        msg = SystemState()
        msg.state = int(self.state)
        self.state_pub.publish(msg)

    def publish_heartbeat_query(self):
        self.heartbeat_query_id = (self.heartbeat_query_id + 1) % 4294967295
        msg = HeartbeatQuery()
        msg.query_id = self.heartbeat_query_id
        msg.sent_timestamp = self.get_clock().now().to_msg()
        self.heartbeat_query_pub.publish(msg)

    def heartbeat_cb(self, msg):
        try:
            node_name = bytes(msg.node_name).decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            node_name = "unknown_node"

        now = self.get_clock().now()
        sent_time = Time.from_msg(msg.original_sent_timestamp)
        latency = (now - sent_time).nanoseconds / 1e9

        if node_name not in self.tracked_nodes:
            self.get_logger().info(f"New node registered: {node_name}")
            self.tracked_nodes[node_name] = {
                'last_query_id': msg.query_id,
                'latency': latency
            }
        else:
            self.tracked_nodes[node_name]["last_query_id"] = msg.query_id
            self.tracked_nodes[node_name]["latency"] = latency

    def watchdog_cb(self):
        nodes_to_remove = []
        for node_name, node_data in self.tracked_nodes.items():
            if self.MAX_MISSED_QUERIES < (self.heartbeat_query_id - node_data["last_query_id"]) % 4294967295:
                self.get_logger().error(
                    f"NODE DISCONNECTED: {node_name} | "
                    f"Current ID: {self.heartbeat_query_id} | "
                    f"Last Node ID: {node_data['last_query_id']}"
                )
                nodes_to_remove.append(node_name)
                continue

            if node_data["latency"] > self.LATENCY_ERROR_THRESHOLD:
                self.get_logger().error(f"Node: {node_name} critical latency")
            elif node_data["latency"] > self.LATENCY_WARNING_THRESHOLD:
                self.get_logger().warn(f"Node: {node_name} high latency")

        for node_name in nodes_to_remove:
            del self.tracked_nodes[node_name]

        self.handle_hw_connection()

    def is_hw_connected(self):
        for node_name in self.tracked_nodes:
            if self.HADWARE_SIGNATURE in node_name.lower():
                return True
        return False

    def handle_hw_connection(self):
        if self.state == self.State.IDLE:
            if self.is_hw_connected():
                self.get_logger().info("Hardware node registered")
                self.connect() # type: ignore

        elif self.state == self.State.CONNECTED:
            if not self.is_hw_connected():
                if self.pending_command:
                    self.get_logger().error(f"HARDWARE DISCONNECTED UNEXPECTETLY")
                    self.enter_error() # type: ignore
                else:
                    self.disconnect() # type: ignore
        else:
            if not self.is_hw_connected():
                self.get_logger().error(f"HARDWARE DISCONNECTED UNEXPECTETLY")
                self.enter_error() # type: ignore

    def request_action_cb(self, request, response):
        action_map = {
            (RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_START): ("arm", HardwareActions.ARM),
            (RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_STOP):  ("disarm", HardwareActions.DISARM),
            
            (RequestAction.Request.ACTION_CONFIGURE, RequestAction.Request.TYPE_START): ("enter_config", HardwareActions.ENTER_CONFIG),
            (RequestAction.Request.ACTION_CONFIGURE, RequestAction.Request.TYPE_STOP):  ("exit_config", HardwareActions.EXIT_CONFIG),
            
            (RequestAction.Request.ACTION_MISSION,   RequestAction.Request.TYPE_START): ("start_mission", HardwareActions.START_MISSION),
            (RequestAction.Request.ACTION_MISSION,   RequestAction.Request.TYPE_STOP):  ("end_mission", HardwareActions.END_MISSION),
        }

        if self.pending_command:
            response.success = False
            self.get_logger().warn("Rejected request: Hardware busy.")
            return response

        # 2. Key aus dem Request extrahieren
        key = (request.action_id, request.request_type)
        
        if key not in action_map:
            self.get_logger().error(f"Invalid Action/Type Combination: {key}")
            response.success = False
            return response

        trigger, hw_cmd = action_map[key]
        check_method = f"may_{trigger}"

        # 3. Validierung und Ausführung
        if hasattr(self, check_method) and getattr(self, check_method)():
            # Hardware-Befehl senden
            msg = HardwareCommand()
            self.command_id += 1
            msg.command_id = self.command_id
            msg.action = hw_cmd
            self.command_pub.publish(msg)
            self.pending_command = True

            response.success = True
            self.get_logger().debug(f"Triggered {trigger} (HW Cmd: {hw_cmd})")
        else:
            self.get_logger().error(f"INVALID TRANSITION: {trigger} not allowed from {self.state}")
            response.success = False

        return response
    
    def hardware_feedback_cb(self, msg: HardwareFeedback):
        action_map = {
            # Triggers ros system
            HardwareActions.REBOOT: "reboot",
            HardwareActions.ARM: "arm",
            HardwareActions.DISARM: "disarm",
            HardwareActions.ENTER_CONFIG: "enter_config",
            HardwareActions.EXIT_CONFIG: "exit_config",
            HardwareActions.START_MISSION: "start_mission",
            HardwareActions.END_MISSION: "end_mission",

            # Triggers SMT
            HardwareActions.SOFT_STOP_COMPLETE: "soft_stop_complete",
            HardwareActions.HARD_STOP_COMPLETE: "hard_stop_complete",

            # Triggers common
            HardwareActions.SOFT_STOP: "soft_stop",
            HardwareActions.HARD_STOP: "hard_stop",
            HardwareActions.ENTER_ERROR: "enter_error",
            HardwareActions.ENTER_EMERGENCY: "enter_emergency",
        }

        if (msg.type == HardwareFeedback.RESPONSE):
            if msg.command_id == self.command_id:
                self.pending_command = False
                if not msg.success:
                    self.get_logger().error(f"TRANSITION FAILED")

        if (msg.current_state != self.state):
            trigger = action_map[msg.action]
            self.get_logger().info(f"Hardware triggered: {trigger}")
            try:
                getattr(self, trigger)()
                if self.state != msg.current_state:
                    self.get_logger().error(f"INVALID HARDWARE STATE")
                    self.enter_error() # type: ignore
            except Exception as e:
                self.get_logger().error(f"Failed to switch state: {e}")
                self.enter_error() # type: ignore

        self.publish_state()
    

def main(args=None):
    rclpy.init(args=args)
    
    ros_node = ControlCenterNode()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()