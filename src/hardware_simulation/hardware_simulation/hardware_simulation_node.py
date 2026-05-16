import rclpy
from rclpy.time import Time
from transitions import Machine
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from enum import IntEnum
import math
import threading
import socket
import threading


from interface.msg import (
    SystemState, HardwareActions, HardwareCommand, HardwareFeedback, 
    TelemetryBatch, TelemetryFrame, AxisData, 
    TrajectoryBatch, TrajectoryFeedback
)
from utility.HeartbeatClient import HeartbeatClient

class HardwareSimulationNode(Node):
    class State(IntEnum):
        IDLE = SystemState.IDLE
        CONNECTED = SystemState.CONNECTED
        ARMED = SystemState.ARMED
        MISSION = SystemState.MISSION
        CONFIG = SystemState.CONFIG
        ERROR = SystemState.ERROR
        EMERGENCY_HALT = SystemState.EMERGENCY_HALT

    class StreamState(IntEnum):
        IDLE = 0
        INIT = 1
        STREAMING = 2
        FINISHED = 3

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

        self.hardware_feedback_pub = None
        self.telemetry_pub = None

        self.telemetry_packet_count = 0

        self.timer_watchdog = self.create_timer(1.0, self.watchdog_cb)
        self.timer_telemetry = self.create_timer(0.1, self.telemetry_cb)

        self.streaming_state = self.StreamState.IDLE
        self.controll_running = False
        self.trajectory_buffer = []
        self.current_trajectory_id = 0
        self.processed_idx = 0
        self.current_joint_angles = [0.0] * 6

        self.last_request_time = self.get_clock().now()
        self.REQUEST_COOLDOWN = 0.03  # 50ms Cooldown zwischen Requests
        
        # 1kHz Loop Simulation
        self.timer_loop = self.create_timer(0.001, self.control_loop_cb) # 1ms
        self.telemetry_accumulator = []

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
        if self.controll_running:
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
        frame.idx = 0 # Index innerhalb der Sequenz
        
        # 3. Signale generieren (Beispiel: Sinus-Wellen für alle Achsen)
        # Nutze die Zeit für eine flüssige Bewegung
        t = diff_time.nanoseconds / 1e9 
        
        for i in range(1, 7):
            axis = AxisData()
            axis.position = self.current_joint_angles[i-1]
            axis.velocity = 0.0
            axis.torque = 0.0
            setattr(frame, f"axis{i}", axis)
        
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
        if getattr(self, f"may_{trigger}")():
            getattr(self, trigger)()
            response.success = True
            response.current_state = self.state
        else:
            self.get_logger().error(
                f"INVALID TRANSITION: Cannot execute '{trigger}' while in state '{self.state.name}'"
            )
            response.success = False
        if self.hardware_feedback_pub:
            self.hardware_feedback_pub.publish(response)
        else:
            self.get_logger().error("Ros init did not provide all publishers")

    def trajectory_data_cb(self, msg):
        """Empfängt Batches vom TrajectoryExecutioner"""
        if msg.trajectory_status == TrajectoryBatch.START:
            if self.streaming_state != self.StreamState.IDLE:
                self.get_logger().error("Recieved trajectory while executing")
                self.controll_running = False
                self.error()    # type: ignore
            self.get_logger().info(f"New Trajectory START received: ID {msg.trajectory_id}")
            self.trajectory_buffer = []
            self.processed_idx = 0
            self.current_trajectory_id = msg.trajectory_id
            
            # START_ACK senden
            ack = TrajectoryFeedback()
            ack.trajectory_id = msg.trajectory_id
            ack.trajectory_status = TrajectoryFeedback.START_ACK
            self.traj_feedback_pub.publish(ack)
            
            self._request_more_data(150)
            self.streaming_state = self.StreamState.INIT
            return
        

        if msg.trajectory_id == self.current_trajectory_id:
            # Daten in den Puffer schieben
            self.trajectory_buffer.extend(msg.data)
            
            if msg.trajectory_status == TrajectoryBatch.END:
                self.get_logger().info("Full trajectory received.")
                self.streaming_state = self.StreamState.FINISHED
                self.controll_running = True
            
            if self.streaming_state == self.StreamState.INIT and len(self.trajectory_buffer) > 100:
                self.streaming_state = self.StreamState.STREAMING
                self.controll_running = True

    def control_loop_cb(self):
        """Die simulierten 1kHz (1ms) Hardware-Interrupt-Routine"""
        if not self.controll_running or self.state == SystemState.ERROR:
            return

        # 1. Sollwert aus Puffer holen
        if len(self.trajectory_buffer) > 0:
            target_frame = self.trajectory_buffer.pop(0)
            self.processed_idx = target_frame.idx
            
            # Simulation: Wir "regeln" auf den Zielwert (hier einfach Kopieren mit Rauschen)
            current_frame = self._simulate_hardware_behavior(target_frame)

            self.current_joint_angles = [
                current_frame.axis1.position,
                current_frame.axis2.position,
                current_frame.axis3.position,
                current_frame.axis4.position,
                current_frame.axis5.position,
                current_frame.axis6.position
            ]
        else:
            if self.streaming_state == self.StreamState.FINISHED:
                self._publish_telemetry_batch()
                ack = TrajectoryFeedback()
                ack.trajectory_id = self.current_trajectory_id
                self.get_logger().info(f"HARDWARE END REACHED: {self.processed_idx}")
                ack.trajectory_status = TrajectoryFeedback.END_REACHED
                ack.current_hardware_idx = self.processed_idx
                if self.traj_feedback_pub:
                    self.traj_feedback_pub.publish(ack)
                self.streaming_state = self.StreamState.IDLE
                self.controll_running = False
            else:
                self.get_logger().error("Trajectory ran dry")
                self.error() # type: ignore
            return
            

        # 2. Telemetrie sammeln
        self.telemetry_accumulator.append(current_frame)
        
        if len(self.telemetry_accumulator) >= 5:
            self._publish_telemetry_batch()

        # 3. Flow Control: Neue Daten anfordern wenn Puffer leerer wird
        # Wenn weniger als 100 Punkte im Puffer sind, fragen wir 50 neue an
        if self.streaming_state != self.StreamState.FINISHED:
            if len(self.trajectory_buffer) < 100:
                now = self.get_clock().now()
                # Prüfen, ob seit dem letzten Request genug Zeit vergangen ist (50ms)
                duration = (now - self.last_request_time).nanoseconds / 1e9
                if duration > self.REQUEST_COOLDOWN:
                    self._request_more_data(50)
                    self.last_request_time = now                


    def _simulate_hardware_behavior(self, target_frame):
        """Fügt den Sollwerten simuliertes Rauschen hinzu"""
        import random
        noise = lambda: random.uniform(-0.001, 0.001)
        
        # Wir klonen den Frame und verrauschen die Achsen
        f = TelemetryFrame()
        diff_time = self.get_clock().now() - self.start_time
        # Umrechnung in Mikrosekunden
        f.time_us = int(diff_time.nanoseconds / 1000)
        f.idx = target_frame.idx
        
        # Simulation der 6 Achsen (Beispielhaft für Achse 1)
        for i in range(1, 7):
            axis_name = f"axis{i}"
            target_axis = getattr(target_frame, axis_name)
            sim_axis = AxisData()
            sim_axis.position = target_axis.position + noise()
            sim_axis.velocity = target_axis.velocity + noise()
            sim_axis.torque = target_axis.torque + noise()
            setattr(f, axis_name, sim_axis)
        
        f.gripper = target_frame.gripper
        return f

    def _request_more_data(self, amount):
        """Sendet REQUEST_DATA an den Executioner"""
        self.get_logger().debug(f"Requested data: {amount} at buffer len: {len(self.trajectory_buffer)}")
        msg = TrajectoryFeedback()
        msg.trajectory_id = self.current_trajectory_id
        msg.trajectory_status = TrajectoryFeedback.REQUEST_DATA
        msg.request_next_count = amount 
        msg.current_hardware_idx = self.processed_idx
        self.traj_feedback_pub.publish(msg)

    def _publish_telemetry_batch(self):
        """Verschickt die gesammelten 5 Frames"""
        batch = TelemetryBatch()
        batch.trajectory_id = self.current_trajectory_id
        batch.packet_num = self.telemetry_packet_count
        batch.data = self.telemetry_accumulator
        
        if self.telemetry_pub:
            self.telemetry_pub.publish(batch)
        
        self.telemetry_packet_count += 1
        self.telemetry_accumulator = []


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
        self.command_sub = self.create_subscription(
            HardwareCommand, 'hardware/command', self.hardware_command_cb, 10, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.hardware_feedback_pub = self.create_publisher(HardwareFeedback, 'hardware/feedback', 1)
        qos_telemetry = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
        self.telemetry_pub = self.create_publisher(TelemetryBatch, 'telemetry', qos_telemetry)

        # Subscriber für Trajektorie-Daten vom Executioner
        qos_trajectory = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        self.traj_sub = self.create_subscription(
            TrajectoryBatch, 'trajectory/data', self.trajectory_data_cb, qos_trajectory, callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        # Publisher für Feedback zum Executioner
        self.traj_feedback_pub = self.create_publisher(
            TrajectoryFeedback, 'trajectory/feedback', 3
        )
        self.connect() # type: ignore

    def _deinitialize_ros_interface(self):
        """Löscht alle ROS-Interfaces sauber"""
        self.get_logger().warn("De-initializing ROS interfaces...")
        
        if self.command_sub:
            self.destroy_subscription(self.command_sub)
        if self.hardware_feedback_pub:
            self.destroy_publisher(self.hardware_feedback_pub)
        if self.telemetry_pub:
            self.destroy_publisher(self.telemetry_pub)
        if self.heartbeat_client:
            self.heartbeat_client.destroy() 
            
        self.command_sub = None
        self.hardware_feedback_pub = None
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