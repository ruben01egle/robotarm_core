import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication

# Deine bestehenden Klassen
from .robot_main_widget import RobotMainWindow
from .data_store import GuiDataStore

class GuiRosNode(Node):
    def __init__(self, data_store):
        super().__init__('robot_gui_node')
        self.store = data_store
        self.max_seq = -1
        self.soll_dict = {} # (traj_id, seq_num) -> target_data

        # Subscriber für Telemetrie (STM32 -> PC)
        self.create_subscription(MyTelemetryMsg, 'telemetry', self.telemetry_cb, 10)
        # Subscriber für Sollwerte (MotionNode -> PC)
        self.create_subscription(MySetpointMsg, 'setpoints', self.setpoint_cb, 10)

    def setpoint_cb(self, msg):
        # Sollwert puffern für das Matching
        self.soll_dict[(msg.trajectory_id, msg.seq_num)] = msg.target_list

    def telemetry_cb(self, msg):
        # 1. Matching
        soll = self.soll_dict.pop((msg.trajectory_id, msg.seq_num), None)
        
        # 2. Nur-Vorwärts-Filter & Push in Store
        if soll and msg.seq_num > self.max_seq:
            self.max_seq = msg.seq_num
            self.store.push_telemetry_frame(
                msg.time_s, 
                msg.actual_list, 
                soll, 
                msg.trajectory_prog
            )

    # Methoden, die von der GUI (Buttons) aufgerufen werden
    def send_arm_command(self):
        # Hier z.B. einen Service-Call oder Publisher nutzen
        self.get_logger().info("Sende Arm-Kommando...")

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