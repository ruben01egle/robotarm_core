import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication

from .robot_main_widget import RobotMainWindow
from .data_store import GuiDataStore
from.ReadMotorConfigClient import ReadMotorActionClient
from .WriteConfigMotorClient import WriteMotorActionClient

from rcl_interfaces.msg import Log
from std_msgs.msg import Empty
from rclpy.action import ActionClient
from interface.msg import TelemetryBatch, TrajectoryBatch, HeartbeatQuery, Heartbeat, SystemState, MotorParameter
from interface.srv import RequestAction
from interface.action import Mission, ConfigMotor, ReadMotorConfigs
from utility.RequestActionClient import RequestActionClient

class GuiRosNode(Node):
    STATE_MAP = {
        SystemState.IDLE: ("IDLE"),
        SystemState.CONNECTED: ("CONNECTED"),
        SystemState.ARMED: ("ARMED"),
        SystemState.MOTION: ("MOTION"),
        SystemState.CONFIG: ("CONFIG"),
        SystemState.ERROR: ("ERROR"),
        SystemState.EMERGENCY_HALT: ("EMERGENCY")
    }

    def __init__(self, data_store):
        super().__init__('robot_gui_node')
        self.store = data_store
        self.state = SystemState.IDLE

        self.create_subscription(HeartbeatQuery, 'heartbeat/query', self.heartbeat_cb, 10)
        self.create_subscription(SystemState, 'system_state', self.system_state_cb, 1)
        self.create_subscription(HeartbeatQuery, 'heartbeat/query', self.heartbeat_cb, 10)
        self.create_subscription(Log, '/rosout', self.log_cb, 50)
        self.create_subscription(TelemetryBatch, 'telemetry', self.telemetry_cb, 100)
        self.create_subscription(TrajectoryBatch, 'trajectory/data', self.trajectory_cb, 100)

        self.heartbeat_pub = self.create_publisher(Heartbeat, 'heartbeat/response', 5)
        self.emergency_pub = self.create_publisher(Empty, 'system_error', 5)

        self.mission_client = ActionClient(self, Mission, 'move_robot')
        self.read_motor_config_client = ReadMotorActionClient(self, self.read_motor_config_cb)
        self.write_motor_config_client = WriteMotorActionClient(self, self.read_motor_config_cb)

        self.request_action_client = RequestActionClient(self)

        state_str = self.STATE_MAP.get(self.state, ("UNKNOWN"))
        self.store.set_status(state=state_str, connected=False, armed=False)


    def heartbeat_cb(self, msg):
        pass

    def system_state_cb(self, msg):
        new_state = msg.state
        new_state_str = self.STATE_MAP.get(new_state, ("UNKNOWN"))
        if self.state != new_state:
            if new_state == SystemState.IDLE:
                self.store.set_status(state=new_state_str, connected=False, armed=False)
            elif new_state == SystemState.CONNECTED:
                self.store.clear_params()
                self.read_motor_config_client.request_action()
                self.store.set_status(state=new_state_str, connected=True, armed=False)
            elif new_state == SystemState.ARMED:
                self.store.set_status(state=new_state_str, connected=True, armed=True)
            elif new_state == SystemState.CONFIG:
                self.store.set_status(state=new_state_str, connected=True, armed=False)
            elif new_state == SystemState.MOTION:
                self.store.set_status(state=new_state_str, connected=True, armed=True)
            elif new_state == SystemState.ERROR:
                pass
        
            self.state = new_state

    def trajectory_cb(self, msg):
        pass

    def telemetry_cb(self, msg):
        pass

    def log_cb(self, msg):
        levels = {20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = levels.get(msg.level, "DEBUG")
        self.store.add_log(level_str, msg.name, msg.msg)

    def mission_goal(self):
        pass

    def mission_feedback_cb(self, msg):
        pass

    def mission_result_cb(self, msg):
        pass

    def motor_config_goal(self):
        pass

    def motor_config_feedback_cb(self, msg):
        pass

    def motor_config_result_cb(self, msg):
        pass

    def read_motor_config_cb(self, params, success):
        if not success:
            self.get_logger().error("Read motor config failed")
            return

        new_parameters = {}
        
        for index, p in enumerate(params):
            axis_id = index + 1  # Index 0 wird Achse 1, Index 1 wird Achse 2, etc.
            
            new_parameters[axis_id] = {
                "p_gain": p.p_gain,
                "i_gain": p.i_gain,
                "d_gain": p.d_gain,
                "limit_v": p.limit_v
            }
        self.store.update_axis_config(new_parameters)

    def write_motor_config_cb(self, axis_id, params, success):
        if not success:
            self.get_logger().error("Read motor config failed")
            return

        new_parameters = {
            axis_id: {
                "p_gain": params.p_gain,
                "i_gain": params.i_gain,
                "d_gain": params.d_gain,
                "limit_v": params.limit_v
            }
        }
        self.store.update_axis_config(new_parameters)

    def arm_command(self, arm):
        if arm:
            self.get_logger().info('Arm robot requested')
            self.request_action_client.send_request(RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_START, False)
        else:
            self.get_logger().info('Disarm robot requested')
            self.request_action_client.send_request(RequestAction.Request.ACTION_DISARM_ROBOT, RequestAction.Request.TYPE_START, False)


    def start_motion_jointangles(self, angles):
        print('start motion joint angles')

    def start_motion_csv(self, path):
        print('start motion csv')

    def stop_motion(self):
        print('self motion')

    def request_read_motor_config(self):
        self.read_motor_config_client.request_action()

    def request_write_motor_config(self, axis, params):
        msg = MotorParameter()
        msg.p_gain = int(params.get("p_gain", 0))
        msg.d_gain = int(params.get("d_gain", 0))
        msg.i_gain = int(params.get("i_gain", 0))
        msg.limit_v = int(params.get("limit_v", 0))
        self.write_motor_config_client.request_action(axis, msg)

    def emergency_stop(self):
        msg = Empty()
        self.emergency_pub.publish(msg)
        self.get_logger().error('EMERGENCY STOP INWOKED')

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
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()