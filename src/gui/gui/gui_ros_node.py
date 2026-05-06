import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PyQt6.QtWidgets import QApplication
import numpy as np

from .robot_main_widget import RobotMainWindow
from .data_store import GuiDataStore
from .ReadMotorConfigClient import ReadMotorActionClient
from .WriteConfigMotorClient import WriteMotorActionClient
from .MissionClient import MissionClient

from rcl_interfaces.msg import Log
from interface.msg import TelemetryBatch, TrajectoryBatch, SystemState, MotorParameter, StopCommand
from interface.srv import RequestAction
from interface.action import Mission
from utility.RequestActionClient import RequestActionClient
from utility.HeartbeatClient import HeartbeatClient
from utility.state_string_map import STATE_MAP

class GuiRosNode(Node):
    def __init__(self, data_store):
        super().__init__('robot_gui_node')
        self.store = data_store
        self.state = SystemState.IDLE
        self.config_requested = False
        self.hold_joint_angles = []
        self.trajectory_buffer = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        self.create_subscription(SystemState, 'system_state', self.system_state_cb, 1)
        self.create_subscription(Log, '/rosout', self.log_cb, 10)
        self.create_subscription(TelemetryBatch, 'telemetry', self.telemetry_cb, 50)
        self.create_subscription(TrajectoryBatch, 'trajectory/data', self.trajectory_cb, qos_profile)

        self.stop_pub = self.create_publisher(StopCommand, 'system_stop', 1)

        self.read_motor_config_client = ReadMotorActionClient(self, self.read_motor_config_cb)
        self.write_motor_config_client = WriteMotorActionClient(self, self.write_motor_config_cb)
        self.mission_client = MissionClient(self, self.mission_feedback_cb)

        self.request_action_client = RequestActionClient(self)
        self.heartbeat_client = HeartbeatClient(self)

        state_str = STATE_MAP.get(self.state, ("UNKNOWN"))
        self.store.set_status(state=state_str, connected=False, armed=False)


    def system_state_cb(self, msg):
        new_state = msg.state
        new_state_str = STATE_MAP.get(new_state, ("UNKNOWN"))
        if self.state != new_state:
            if new_state == SystemState.IDLE:
                self.store.clear_store()
                self.config_requested = False
                self.store.set_status(state=new_state_str, connected=False, armed=False)
            elif new_state == SystemState.CONNECTED:
                if not self.config_requested:
                    self.read_motor_config_client.request_read_config()
                    self.config_requested = True
                self.store.set_status(state=new_state_str, connected=True, armed=False)
            elif new_state == SystemState.ARMED:
                self.store.set_status(state=new_state_str, connected=True, armed=True)
            elif new_state == SystemState.CONFIG:
                self.store.set_status(state=new_state_str, connected=True, armed=False)
            elif new_state == SystemState.MISSION:
                self.store.set_status(state=new_state_str, connected=True, armed=True)
            elif new_state == SystemState.ERROR:
                pass
        
            self.state = new_state

    def trajectory_cb(self, msg):
        t_id = msg.trajectory_id
        if msg.trajectory_status == TrajectoryBatch.START:
            self.trajectory_buffer.clear()
            self.hold_joint_angles = []
        
        for frame in msg.data:
            key = (t_id, frame.idx)
            
            # Erstelle eine Liste von Dictionaries (eines pro Achse)
            # Format: [{'p':.., 'v':.., 't':..}, {...}, ...]
            target_list = []
            for axis in [frame.axis1, frame.axis2, frame.axis3, frame.axis4, frame.axis5, frame.axis6]:
                target_list.append({
                    'p': axis.position,
                    'v': axis.velocity,
                    't': axis.torque
                })
                
            self.trajectory_buffer[key] = target_list
            if msg.trajectory_status == TrajectoryBatch.END:
                self.hold_joint_angles = target_list

    def telemetry_cb(self, msg):
        t_id = msg.trajectory_id
        
        for frame in msg.data:
            key = (t_id, frame.idx)
            time_s = frame.time_us / 1_000_000.0 # Umrechnung in Sekunden
            
            # Ist-Werte der 6 Achsen aufbereiten
            actual_list = []
            for axis in [frame.axis1, frame.axis2, frame.axis3, frame.axis4, frame.axis5, frame.axis6]:
                actual_list.append({
                    'p': axis.position,
                    'v': axis.velocity,
                    't': axis.torque
                })

            if self.state == SystemState.CONNECTED:
                self.hold_joint_angles = actual_list
                target_list = self.hold_joint_angles
            elif self.state == SystemState.ARMED:
                target_list = self.hold_joint_angles
            elif self.state == SystemState.MISSION:
                if key in self.trajectory_buffer:
                    target_list = self.trajectory_buffer.pop(key)
                else:
                    continue
            else:
                continue
            self.store.push_telemetry_frame(time_s, actual_list, target_list)

    def log_cb(self, msg):
        levels = {20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = levels.get(msg.level, "DEBUG")
        self.store.add_log(level_str, msg.name, msg.msg)


    def read_motor_config_cb(self, params, success):
        if not success:
            self.config_requested = False
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

    def mission_feedback_cb(self, feedback_msg):
        self.store.set_progress(feedback_msg.feedback.planning_progress, feedback_msg.feedback.execution_progress)

    # functions for gui to attach signals to
    def arm_command(self, arm):
        if arm:
            self.get_logger().info('Arm robot requested')
            self.request_action_client.send_request(RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_START, False)
        else:
            self.get_logger().info('Disarm robot requested')
            self.request_action_client.send_request(RequestAction.Request.ACTION_ARM_ROBOT, RequestAction.Request.TYPE_STOP, False)

    def start_motion_jointangles(self, angles, scale):
        self.store.set_progress(0, 0)
        self.get_logger().info('Start joint angle mission')
        joint_angle_arr = np.array(angles, dtype=np.float32)
        self.mission_client.request_mission(Mission.Goal.OPTION_SET_JOINT_ANGLES, None, joint_angle_arr, scale)

    def start_motion_csv(self, path):
        self.store.set_progress(0, 0)
        self.get_logger().info('Start csv mission')
        self.mission_client.request_mission(Mission.Goal.OPTION_CSV, path, None, None)

    def request_read_motor_config(self):
        self.read_motor_config_client.request_read_config()

    def request_write_motor_config(self, axis, params):
        msg = MotorParameter()
        msg.p_gain = int(params.get("p_gain", 0))
        msg.d_gain = int(params.get("d_gain", 0))
        msg.i_gain = int(params.get("i_gain", 0))
        msg.limit_v = int(params.get("limit_v", 0))
        self.write_motor_config_client.request_write_config(axis, msg)

    def stop_motion(self):
        msg = StopCommand()
        msg.stop_type = StopCommand.TYPE_SOFT_STOP
        self.stop_pub.publish(msg)
        self.get_logger().warn('SOFT STOP INWOKED')
        self.mission_client.cancel_current_goal()

    def hard_stop(self):
        msg = StopCommand()
        msg.stop_type = StopCommand.TYPE_HARD_STOP
        self.stop_pub.publish(msg)
        self.get_logger().warn('HARD STOP INWOKED')

    def emergency_stop(self):
        msg = StopCommand()
        msg.stop_type = StopCommand.TYPE_EMERGENCY
        self.stop_pub.publish(msg)
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
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()