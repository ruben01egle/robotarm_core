import sys
import threading
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import pyqtSignal, pyqtSlot, QObject

from .robot_main_widget import RobotMainWindow
from .data_store import GuiDataStore

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction
from robotarm_interface.msg import SystemStatus

class GuiRosNode(Node, QObject):
    set_manual_move = pyqtSignal(bool)
    set_axis_limits = pyqtSignal(list)

    def __init__(self, data_store):
        Node.__init__(self, 'gui_ros_node')
        QObject.__init__(self)
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

        self.create_subscription(Log, '/rosout', self.log_cb, 10)

        self.forward_command_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands', 
            1
        )

        self.request_action_client = RequestActionClient(self)

        self.urdf_timer = self.create_timer(1.0, self.fetch_urdf_limits_startup)

        self.store.set_status(state="UNKNOWN", connected=False, armed=False, stopped=False)

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

    def log_cb(self, msg):
        levels = {20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = levels.get(msg.level, "DEBUG")
        self.store.add_log(level_str, msg.name, msg.msg)

    def system_status_cb(self, msg: SystemStatus):
        self.store.set_status(msg.state, msg.connected, msg.armed, msg.stopped)

    # functions for gui to attach signals to
    @pyqtSlot(bool)
    def arm_command(self, arm: bool):
        if arm:
            self.get_logger().info('Arm robot requested')
            self.request_action_client.send_request(RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_START, False)
        else:
            self.get_logger().info('Disarm robot requested')
            self.request_action_client.send_request(RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_STOP, False)

    @pyqtSlot(bool)
    def req_manual_move(self, move: bool):
        controller_name = "forward_position_controller"
        if move:
            self.get_logger().info('Manual move requested')
            if self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_START, blocking=True, controller_names=controller_name):
                self.set_manual_move.emit(True)
        else:
            self.get_logger().info('Manual move stop requested')
            if self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP, blocking=True, controller_names=controller_name):
                self.set_manual_move.emit(False)

    @pyqtSlot(list)
    def stream_move(self, target: list):
        try:
            msg = Float64MultiArray()
            msg.data = target
            self.forward_command_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error while streaming manual move targets: {e}")

    @pyqtSlot(bool)
    def stop(self, stop: bool):
        if stop:
            self.get_logger().info('Stop robot requested')
            self.request_action_client.send_request(RequestAction.Request.INVOKE_STOP, RequestAction.Request.TYPE_START, False)
        else:
            self.get_logger().info('Release robot requested')
            self.request_action_client.send_request(RequestAction.Request.INVOKE_STOP, RequestAction.Request.TYPE_STOP, False)

    @pyqtSlot()
    def emergency(self):
        self.get_logger().error('EMERGENCY STOP INWOKED')
        self.request_action_client.send_request(RequestAction.Request.INVOKE_EMERGENCY, RequestAction.Request.TYPE_START, False)

    def fetch_urdf_limits_startup(self):
        """Versucht beim Startup die URDF zu laden. Stoppt sich selbst bei Erfolg."""
        # Service-Client für die Parameter des robot_state_publisher erstellen
        param_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        
        if not param_client.service_is_ready():
            self.get_logger().info("Warte auf '/robot_state_publisher' um URDF-Limits zu lesen...")
            return

        # Wenn der Service da ist, Timer stoppen, damit wir das nur EINMAL machen
        self.urdf_timer.cancel()

        # Parameter abfragen
        req = GetParameters.Request()
        req.names = ['robot_description']
        
        future = param_client.call_async(req)
        # Wir hängen einen Callback an das Future, sobald die Antwort da ist
        future.add_done_callback(self.urdf_response_cb)

    def urdf_response_cb(self, future):
        try:
            response = future.result()
            if not response or not response.values:
                self.get_logger().error("URDF-Parameter 'robot_description' ist leer!")
                return

            # Den XML-String aus dem ROS-Parameter extrahieren
            urdf_string = response.values[0].string_value
            
            # URDF parsen
            limits = self.parse_urdf_limits(urdf_string)
            
            if limits:
                self.get_logger().info(f"URDF parsed succesfully: {len(limits)} axis found!")
                # --- HIER FEUERT DAS SIGNAL ---
                # Qt reicht diese Liste jetzt an dein ManualControl und TelemetryDashboard weiter
                self.set_axis_limits.emit(limits)
            else:
                self.get_logger().warn("No axis found")

        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")

    def parse_urdf_limits(self, urdf_xml_str: str) -> list:
        """Parst den URDF-String und gibt eine Liste von (min, max) in RADIAN zurück."""
        limits_list = []
        try:
            root = ET.fromstring(urdf_xml_str)
            
            # Wir suchen nach allen 'joint'-Elementen
            for joint in root.findall('joint'):
                joint_type = joint.get('type')
                
                # Nur Gelenke, die Limits besitzen (revolute = rotierend, prismatic = linear)
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

def main(args=None):
    # 1. ROS initialisieren
    rclpy.init(args=args)
    
    # 2. Shared Data Store
    data_store = GuiDataStore()
    
    # 3. ROS Node erstellen
    ros_node = GuiRosNode(data_store)
    
    # 4. ROS in einem eigenen Thread spinnen
    # Damit rclpy.spin() nicht den Qt-Event-Loop blockiert
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()
    
    # 5. Qt Application starten
    app = QApplication(sys.argv)
    
    # Hier übergibst du die ros_node an das Widget, 
    # damit Buttons direkt Methoden der Node aufrufen können
    window = RobotMainWindow(data_store, ros_node)
    window.show()
    
    try:
        exit_code = app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()