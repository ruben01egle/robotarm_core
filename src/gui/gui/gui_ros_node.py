import sys
import math
import threading
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log, Parameter, ParameterValue, ParameterType, ParameterEvent
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from tf2_ros import Buffer, TransformListener, TransformException

from PyQt6.QtWidgets import QApplication

from .robot_main_widget import RobotMainWindow
from .data_store import GuiDataStore
from .controller_modes import CARTESIAN_JOG_CONTROLLERS, IMPEDANCE_CONTROLLER, IMPEDANCE_MODES

from robotarm_interface.msg import SystemStatus, ModeState, RobotarmMode

# SystemStatus.state -> name for the display
STATE_NAMES = {
    getattr(SystemStatus, name): name
    for name in ('IDLE', 'CONNECTED', 'ARMED', 'MISSION', 'CONFIG', 'STOP', 'ERROR', 'EMERGENCY')
}

# The name of the TCP link is fixed in the URDF
TCP_FRAME = 'tcp'

# Stiffness parameters of the cartesian_impedance_controller -> keys in the data store
STIFFNESS_PARAMS = {
    'cartesian_stiffness_frame': 'frame',
    'cartesian_linear_stiffness': 'linear',
    'cartesian_angular_stiffness': 'angular',
    'max_linear_stiffness': 'max_linear',
    'max_angular_stiffness': 'max_angular',
}

class GuiRosNode(Node):
    def __init__(self, data_store):
        Node.__init__(self, 'gui_ros_node')
        self.store = data_store

        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_cb, 
            telemetry_qos
        )

        self.create_subscription(
            SystemStatus, 
            '/system_status', 
            self.system_status_cb, 
            telemetry_qos
        )

        # Firmware mode, requestable modes and switching (greys out mode buttons). Latched and
        # published on change only: reliable + transient local, or the current state never arrives
        self.create_subscription(
            ModeState,
            '/mode_manager/state',
            self.mode_state_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.last_mode = None

        self.create_subscription(Log, '/rosout', self.log_cb, 10)

        # TCP pose from TF (robot_state_publisher on the RT PC), same rate as /joint_states
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tcp_stamp = None
        self.tcp_timer = self.create_timer(1.0 / 50.0, self.tcp_pose_cb)

        self.urdf_timer = self.create_timer(1.0, self.fetch_urdf_limits_startup)

        # Velocity limits of both jog instances (direct and *_impedance); the simulation has no direct one
        self.cartesian_param_clients = {
            name: self.create_client(GetParameters, f'/{name}/get_parameters')
            for name in CARTESIAN_JOG_CONTROLLERS.values()
        }
        self.cartesian_limits_timer = self.create_timer(1.0, self.fetch_cartesian_limits_startup)

        # Stiffness of the cartesian_impedance_controller: read at start, on every activation and on request
        self.stiffness_get_client = self.create_client(
            GetParameters, f'/{IMPEDANCE_CONTROLLER}/get_parameters')
        self.stiffness_set_client = self.create_client(
            SetParametersAtomically, f'/{IMPEDANCE_CONTROLLER}/set_parameters_atomically')
        self.stiffness_timer = self.create_timer(1.0, self.fetch_stiffness_startup)
        self.create_subscription(ParameterEvent, '/parameter_events', self.parameter_event_cb, 10)

        self.store.set_status(state="UNKNOWN", connected=False, armed=False, stopped=False, active_node="")

    def joint_state_cb(self, msg: JointState):
        try:
            time_s = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
            
            num_joints = len(msg.position)
            
            joint_state_list = []
            for i in range(num_joints):
                v_val = msg.velocity[i] if i < len(msg.velocity) else 0.0
                t_val = msg.effort[i] if i < len(msg.effort) else 0.0
                
                joint_state_list.append({
                    'p': msg.position[i],
                    'v': v_val,
                    't': t_val
                })
            
            self.store.push_telemetry_frame(time_s, joint_state_list)
            
        except Exception as e:
            self.get_logger().error(f"Error while parsing joint states: {e}", throttle_duration_sec=2.0)

    def tcp_pose_cb(self):
        # Base frame = root link of the URDF, known only after the URDF has been read
        base_frame = self.store.get_base_frame()
        if base_frame is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(base_frame, TCP_FRAME, Time())
        except TransformException as e:
            self.get_logger().debug(f"TCP pose not available: {e}", throttle_duration_sec=10.0)
            return

        stamp = tf.header.stamp
        time_s = stamp.sec + (stamp.nanosec * 1e-9)
        if time_s == self.last_tcp_stamp:
            return
        self.last_tcp_stamp = time_s

        t = tf.transform.translation
        rx, ry, rz = quat_to_rpy(tf.transform.rotation)
        self.store.push_cartesian_frame(time_s, [t.x, t.y, t.z, rx, ry, rz])

    def log_cb(self, msg):
        levels = {20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = levels.get(msg.level, "DEBUG")
        self.store.add_log(level_str, msg.name, msg.msg)

    def system_status_cb(self, msg: SystemStatus):
        state = STATE_NAMES.get(msg.state, f"UNKNOWN ({msg.state})")
        self.store.set_status(state, msg.connected, msg.armed, msg.stopped, msg.active_node)

    def mode_state_cb(self, msg: ModeState):
        mode = msg.mode.mode
        self.store.set_mode_state(mode, [m.mode for m in msg.available], msg.switching)

        if mode != self.last_mode:
            # FAULT lasts only until the firmware has cleaned up to STOPPED, keep it in the log
            if mode == RobotarmMode.FAULT:
                self.store.add_log("ERROR", "mode_manager",
                                   "Controller error, the firmware stops the arm (FAULT -> STOPPED)")
            # Read again on every activation, the limits may have changed after a reconfigure
            if mode in IMPEDANCE_MODES and self.last_mode not in IMPEDANCE_MODES:
                self.request_stiffness_read()
        self.last_mode = mode

    def fetch_urdf_limits_startup(self):
        param_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        
        if not param_client.service_is_ready():
            self.get_logger().info("Waiting for'/robot_state_publisher' to read URDF...")
            return

        self.urdf_timer.cancel()

        # Query the parameter
        req = GetParameters.Request()
        req.names = ['robot_description']
        
        future = param_client.call_async(req)
        future.add_done_callback(self.urdf_response_cb)

    def urdf_response_cb(self, future):
        try:
            response = future.result()
            if not response or not response.values:
                self.get_logger().error("URDF-Parameter 'robot_description' is empty!")
                return

            # XML string of the ROS parameter
            urdf_string = response.values[0].string_value
            
            # Parse the URDF
            base_frame = self.parse_urdf_root_link(urdf_string)
            if base_frame:
                self.get_logger().info(f"URDF root link: '{base_frame}'")
                self.store.set_base_frame(base_frame)

            limits = self.parse_urdf_limits(urdf_string)
            
            if limits:
                self.get_logger().info(f"URDF parsed succesfully: {len(limits)} axis found!")
                self.store.set_joint_limits(limits)
            else:
                self.get_logger().warn("No axis found")

        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")

    def fetch_cartesian_limits_startup(self):
        pending = [name for name, client in self.cartesian_param_clients.items()
                   if name not in self.store.get_cartesian_limits()]
        if not pending:
            self.cartesian_limits_timer.cancel()
            return

        for name in pending:
            client = self.cartesian_param_clients[name]
            if not client.service_is_ready():
                self.get_logger().info(f"Waiting for '/{name}' to read velocity limits...", throttle_duration_sec=10.0)
                continue

            req = GetParameters.Request()
            req.names = ['max_linear_velocity', 'max_angular_velocity']

            future = client.call_async(req)
            future.add_done_callback(lambda f, n=name: self.cartesian_limits_response_cb(f, n))

    def cartesian_limits_response_cb(self, future, name):
        try:
            response = future.result()
            if not response or len(response.values) != 2:
                self.get_logger().error(f"Velocity limits of '{name}' could not be read!")
                return

            max_linear = response.values[0].double_value
            max_angular = response.values[1].double_value

            if not all(math.isfinite(v) and v > 0.0 for v in (max_linear, max_angular)):
                self.get_logger().error(f"Invalid velocity limits of '{name}': lin={max_linear}, ang={max_angular}")
                return

            if name not in self.store.get_cartesian_limits():
                self.get_logger().info(f"Velocity limits of '{name}': {max_linear} m/s, {max_angular} rad/s")
            self.store.set_cartesian_limits(name, max_linear, max_angular)

        except Exception as e:
            self.get_logger().error(f"Error reading velocity limits of '{name}': {e}")

    # --- Stiffness of the cartesian_impedance_controller ---
    def fetch_stiffness_startup(self):
        if not self.stiffness_get_client.service_is_ready():
            self.get_logger().info(f"Waiting for '/{IMPEDANCE_CONTROLLER}' to read stiffness...", throttle_duration_sec=10.0)
            return
        self.stiffness_timer.cancel()
        self.request_stiffness_read()

    def request_stiffness_read(self):
        """Read frame, values and limits (asynchronous, the result goes to the data store)."""
        if not self.stiffness_get_client.service_is_ready():
            self.get_logger().warn(f"Service '/{IMPEDANCE_CONTROLLER}/get_parameters' is not available!")
            return
        req = GetParameters.Request()
        req.names = list(STIFFNESS_PARAMS.keys())
        future = self.stiffness_get_client.call_async(req)
        future.add_done_callback(self.stiffness_read_response_cb)

    def stiffness_read_response_cb(self, future):
        try:
            response = future.result()
            if not response or len(response.values) != len(STIFFNESS_PARAMS):
                self.get_logger().error("Stiffness parameters could not be read!")
                return
            values = {}
            for name, value in zip(STIFFNESS_PARAMS.keys(), response.values):
                parsed = parse_stiffness_value(name, value)
                if parsed is None:
                    self.get_logger().error(f"Stiffness parameter '{name}' missing or of wrong type!")
                    return
                values[STIFFNESS_PARAMS[name]] = parsed
            self.store.update_stiffness(values)
        except Exception as e:
            self.get_logger().error(f"Error reading stiffness: {e}")

    def apply_stiffness(self, frame: str, linear: list, angular: list):
        """Frame and all six values in ONE set_parameters_atomically; the result goes to the data store."""
        if not self.stiffness_set_client.service_is_ready():
            self.store.set_stiffness_apply_result(False, f"Service '/{IMPEDANCE_CONTROLLER}/set_parameters_atomically' not available")
            return

        def param(name, value):
            p = Parameter(name=name)
            if isinstance(value, str):
                p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
            else:
                p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                         double_array_value=[float(v) for v in value])
            return p

        req = SetParametersAtomically.Request()
        req.parameters = [
            param('cartesian_stiffness_frame', frame),
            param('cartesian_linear_stiffness', linear),
            param('cartesian_angular_stiffness', angular),
        ]
        future = self.stiffness_set_client.call_async(req)
        future.add_done_callback(lambda f: self.stiffness_apply_response_cb(f, frame, linear, angular))

    def stiffness_apply_response_cb(self, future, frame, linear, angular):
        try:
            response = future.result()
            if response is None:
                self.store.set_stiffness_apply_result(False, "no response")
                return
            if response.result.successful:
                self.get_logger().info(f"Stiffness applied: [{frame}] lin={list(linear)} N/m, ang={list(angular)} Nm/rad")
                self.store.update_stiffness({'frame': frame, 'linear': list(linear), 'angular': list(angular)})
            else:
                self.get_logger().error(f"Stiffness rejected: {response.result.reason}")
            self.store.set_stiffness_apply_result(response.result.successful, response.result.reason)
        except Exception as e:
            self.store.set_stiffness_apply_result(False, str(e))

    def parameter_event_cb(self, msg: ParameterEvent):
        """Take over stiffness changes by other clients (and our own)."""
        if msg.node != f'/{IMPEDANCE_CONTROLLER}':
            return
        values = {}
        for p in list(msg.new_parameters) + list(msg.changed_parameters):
            if p.name in STIFFNESS_PARAMS:
                parsed = parse_stiffness_value(p.name, p.value)
                if parsed is not None:
                    values[STIFFNESS_PARAMS[p.name]] = parsed
        if values:
            self.store.update_stiffness(values)

    def parse_urdf_root_link(self, urdf_xml_str: str):
        """Root link = the only link that is no joint's 'child'."""
        try:
            root = ET.fromstring(urdf_xml_str)
            children = {joint.find('child').get('link') for joint in root.findall('joint')}
            roots = [link.get('name') for link in root.findall('link') if link.get('name') not in children]

            if len(roots) != 1:
                self.get_logger().error(f"URDF must have exactly one root link, found: {roots}")
                return None
            return roots[0]
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF root link: {e}")
            return None

    def parse_urdf_limits(self, urdf_xml_str: str) -> list:
        limits_list = []
        try:
            root = ET.fromstring(urdf_xml_str)
            
            # all 'joint' elements
            for joint in root.findall('joint'):
                joint_type = joint.get('type')
                
                # Only joints with limits (revolute = rotating, prismatic = linear)
                if joint_type in ['revolute', 'prismatic']:
                    limit_element = joint.find('limit')
                    if limit_element is not None:
                        lower = float(limit_element.get('lower', 0.0))
                        upper = float(limit_element.get('upper', 0.0))
                        velocity = float(limit_element.get('velocity', 1.0))
                        limits_list.append((lower, upper, velocity))
                        
            return limits_list
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            return []

def parse_stiffness_value(name, value):
    """ParameterValue -> str / [3 floats] / float, None for a wrong type."""
    if name == 'cartesian_stiffness_frame':
        return value.string_value if value.type == ParameterType.PARAMETER_STRING else None
    if name in ('cartesian_linear_stiffness', 'cartesian_angular_stiffness'):
        if value.type != ParameterType.PARAMETER_DOUBLE_ARRAY or len(value.double_array_value) != 3:
            return None
        return list(value.double_array_value)
    return value.double_value if value.type == ParameterType.PARAMETER_DOUBLE else None

def quat_to_rpy(q):
    """Quaternion -> roll/pitch/yaw (fixed axes x-y-z) in rad."""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw

def main(args=None):
    # 1. Initialise ROS
    rclpy.init(args=args)
    
    # 2. Shared Data Store
    data_store = GuiDataStore()
    
    # 3. ROS node
    ros_node = GuiRosNode(data_store)
    
    # 4. Spin ROS in its own thread,
    # so rclpy.spin() does not block the Qt event loop
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()
    
    # 5. Start the Qt application
    app = QApplication(sys.argv)

    # The window gets the ROS node, so buttons can call its methods directly
    window = RobotMainWindow(data_store, ros_node)
    window.show()
    
    try:
        exit_code = app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: zero the last twist command
        window.cartesian_page.shutdown()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()