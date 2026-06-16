import sys
import threading
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters

from PyQt6.QtWidgets import QApplication

from .robot_main_widget import RobotMainWindow
from .data_store import GuiDataStore

from robotarm_interface.msg import SystemStatus

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

        self.create_subscription(Log, '/rosout', self.log_cb, 10)

        self.urdf_timer = self.create_timer(1.0, self.fetch_urdf_limits_startup)

        self.store.set_status(state="UNKNOWN", connected=False, armed=False, stopped=False, active_nodes="", active_controllers="")

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
        self.store.set_status(msg.state, msg.connected, msg.armed, msg.stopped, msg.active_nodes, msg.active_controllers)

    def fetch_urdf_limits_startup(self):
        param_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        
        if not param_client.service_is_ready():
            self.get_logger().info("Waiting for'/robot_state_publisher' to read URDF...")
            return

        self.urdf_timer.cancel()

        # Parameter abfragen
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

            # Den XML-String aus dem ROS-Parameter extrahieren
            urdf_string = response.values[0].string_value
            
            # URDF parsen
            limits = self.parse_urdf_limits(urdf_string)
            
            if limits:
                self.get_logger().info(f"URDF parsed succesfully: {len(limits)} axis found!")
                self.store.set_joint_limits(limits)
            else:
                self.get_logger().warn("No axis found")

        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")

    def parse_urdf_limits(self, urdf_xml_str: str) -> list:
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