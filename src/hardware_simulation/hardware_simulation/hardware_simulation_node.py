import rclpy
from rclpy.time import Time
from transitions import Machine
from rclpy.node import Node
from enum import IntEnum
import math
import socket
import threading


from interface.msg import SystemState, HardwareActions, HardwareCommand, HardwareFeedback, TelemetryBatch, TelemetryFrame, AxisData

from utility.HeartbeatClient import HeartbeatClient

class HardwareSimulationNode(Node):
    class State(IntEnum):
        IDLE = SystemState.IDLE
        CONNECTED = SystemState.CONNECTED
        ARMED = SystemState.ARMED
        MISSION = SystemState.MOTION
        CONFIG = SystemState.CONFIG
        ERROR = SystemState.ERROR
        EMERGENCY_HALT = SystemState.EMERGENCY_HALT

    state: State

    def __init__(self):
        super().__init__('stm32_sim_node')

        self.heartbeat_client = None

        self.BEACON_PORT = 6666
        self.BEACON_SIGNATURE = b"ROS2"
        self.TIMEOUT_LIMIT = 3.0
        self.udp_thread = None
        self.stop_udp_event = threading.Event()

        transitions = [
            {'trigger': 'connect', 'source': self.State.IDLE, 'dest': self.State.CONNECTED},
            {'trigger': 'disconnect', 'source': self.State.CONNECTED, 'dest': self.State.IDLE},
            {'trigger': 'enter_config', 'source': self.State.CONNECTED, 'dest': self.State.CONFIG},
            {'trigger': 'exit_config', 'source': self.State.CONFIG, 'dest': self.State.CONNECTED},
            {'trigger': 'arm', 'source': self.State.CONNECTED, 'dest': self.State.ARMED},
            {'trigger': 'disarm', 'source': self.State.ARMED, 'dest': self.State.CONNECTED},
            {'trigger': 'start_mission', 'source': self.State.ARMED, 'dest': self.State.MISSION},
            {'trigger': 'end_mission', 'source': self.State.MISSION, 'dest': self.State.ARMED},
            {'trigger': 'error', 'source': '*', 'dest': self.State.ERROR},
            {'trigger': 'emergency', 'source': '*', 'dest': self.State.EMERGENCY_HALT},
        ]
        self.machine = Machine(model=self, states=self.State, transitions=transitions, initial=self.State.IDLE)

        self.hardware_feedbaack_pub = None
        self.telemetry_pub = None

        self.telemetry_packet_count = 0

        self.timer_watchdog = self.create_timer(1.0, self.watchdog_cb)
        self.timer_telemetry = self.create_timer(0.1, self.telemetry_cb)

        self.start_time = self.get_clock().now()

        self.get_logger().info('Hardware sim node running')

    def watchdog_cb(self):
        # 1. Discovery Logik
        if self.state == self.State.IDLE:
            self._start_discovery()
            return 

        now = self.get_clock().now()
        if not self.heartbeat_client:
            return
        elapsed = (now - self.heartbeat_client.get_last_query_time()).nanoseconds / 1e9
        self.TIMEOUT_LIMIT = 3.0

        if elapsed > self.TIMEOUT_LIMIT:
            self.get_logger().error(f"CONNECTION LOST: No Heartbeat for {elapsed:.2f}s")
            
            if self.state == self.State.CONNECTED:
                self.get_logger().warn("Returning to IDLE...")
                self._deinitialize_ros_interface()
                self.disconnect() # type: ignore
            else:
                self.get_logger().error("Emergency Transition to ERROR state!")
                self.error() # type: ignore
                self._deinitialize_ros_interface()

    def telemetry_cb(self):
        if not self.telemetry_pub:
            return
        # 1. Die Nachricht (Batch) initialisieren
        batch = TelemetryBatch()
        batch.packet_num = self.telemetry_packet_count
        batch.trajectory_id = 0
        
        # 2. Einen einzelnen TelemetryFrame erstellen
        frame = TelemetryFrame()
        
        # Zeitstempel in Mikrosekunden (simuliert)
        diff_time = self.get_clock().now() - self.start_time
        # Umrechnung in Mikrosekunden
        frame.time_us = int(diff_time.nanoseconds / 1000)
        frame.idx = batch.packet_num # Index innerhalb der Sequenz
        
        # 3. Signale generieren (Beispiel: Sinus-Wellen für alle Achsen)
        # Nutze die Zeit für eine flüssige Bewegung
        t = diff_time.nanoseconds / 1e9 
        
        def generate_axis(offset):
            axis = AxisData()
            # Beispielwerte: Position schwingt, Velocity ist Ableitung, Torque simuliert Last
            axis.position = math.sin(t + offset)
            axis.velocity = math.cos(t + offset)
            axis.torque = (math.sin(t * 2 + offset) * 0.5)
            return axis

        # Achsen 1 bis 6 befüllen
        frame.axis1 = generate_axis(0.0)
        frame.axis2 = generate_axis(0.5)
        frame.axis3 = generate_axis(1.0)
        frame.axis4 = generate_axis(1.5)
        frame.axis5 = generate_axis(2.0)
        frame.axis6 = generate_axis(2.5)
        
        frame.gripper = 0.0 # Simulierter Greiferwert (0 = offen, 1 = geschlossen)

        # 4. Den Frame in das Batch-Array einfügen
        # Da es ein 'Bounded Sequence' ist (<=10), nutzen wir eine Liste
        batch.data = [frame]

        # 5. Veröffentlichen
        self.telemetry_pub.publish(batch)
        
        # Zähler für das nächste Paket erhöhen
        self.telemetry_packet_count = batch.packet_num + 1

    def hardware_command_cb(self, msg):
        action_map = {
            (HardwareActions.ARM): ("arm"),
            (HardwareActions.DISARM):  ("disarm"),
            
            (HardwareActions.ENTER_CONFIG): ("enter_config"),
            (HardwareActions.EXIT_CONFIG):  ("exit_config"),
            
            (HardwareActions.START_MISSION): ("start_mission"),
            (HardwareActions.END_MISSION):  ("end_mission"),
        }
        response = HardwareFeedback()
        response.command_id = msg.command_id
        response.action = msg.action
        
        trigger = action_map[msg.action]
        try:
            getattr(self, trigger)()
            response.success = True
            response.current_state = self.state
        except:
            response.success = False
        if self.hardware_feedbaack_pub:
            self.hardware_feedbaack_pub.publish(response)
        else:
            self.get_logger().error("Ros init did not provide all publishers")


    def _start_discovery(self):
        """Startet den UDP-Listener Thread"""
        if self.udp_thread is not None and self.udp_thread.is_alive():
            return
            
        self.stop_udp_event.clear()
        self.udp_thread = threading.Thread(target=self._listen_for_beacon, daemon=True)
        self.udp_thread.start()
        self.get_logger().info("Discovery started: Listening for Beacon...")

    def _setup_ros_interface(self):
        """Erstellt Publisher/Subscriber"""
        self.start_time = self.get_clock().now()
        self.heartbeat_client = HeartbeatClient(self)
        self.feedback_pub = self.create_publisher(HardwareFeedback, 'hardware/feedback', 10)
        self.command_sub = self.create_subscription(
            HardwareCommand, 'hardware/command', self.hardware_command_cb, 10
        )
        self.hardware_feedbaack_pub = self.create_publisher(HardwareFeedback, 'hardware/feedback', 1)
        self.telemetry_pub = self.create_publisher(TelemetryBatch, 'telemetry', 1)
        self.connect() # type: ignore

    def _deinitialize_ros_interface(self):
        """Löscht alle ROS-Interfaces sauber"""
        self.get_logger().warn("De-initializing ROS interfaces...")
        
        if self.command_sub:
            self.destroy_subscription(self.command_sub)
        if self.feedback_pub:
            self.destroy_publisher(self.feedback_pub)
        if self.telemetry_pub:
            self.destroy_publisher(self.telemetry_pub)
        if self.heartbeat_client:
            self.heartbeat_client.destroy() 
            
        self.command_sub = None
        self.feedback_pub = None
        self.heartbeat_client = None
        self.disconnect() # type: ignore

    def _listen_for_beacon(self):
        """UDP-Thread: Beendet sich selbst bei Erfolg oder Stop-Event"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0) # Timeout für sauberes Beenden des Threads
        sock.bind(('', 6666))
        
        while not self.stop_udp_event.is_set() and rclpy.ok():
            try:
                data, addr = sock.recvfrom(1024)
                if data == b"ROS2":
                    self.get_logger().info(f"Beacon found! Initializing ROS...")
                    # ROS initialisieren (muss im Main-Context/Timer passieren oder Thread-safe sein)
                    self._setup_ros_interface()
                    break # Thread beenden
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP Socket Error: {e}")
                break
        
        sock.close()
        self.get_logger().info("UDP Discovery thread stopped.")

def main(args=None):
    rclpy.init(args=args)
    
    ros_node = HardwareSimulationNode()
    
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