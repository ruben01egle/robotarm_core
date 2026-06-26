from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QFrame
from PyQt6.QtCore import Qt
import math

from std_msgs.msg import Float64MultiArray

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction, SetFloat64

class ManualControlWidget(QWidget):
    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node
        self.name = "ManualControlWidget"
        self.required_controller = "teleop_controller"

        self.request_action_client = RequestActionClient(self.node)
        self.joint_target_pub = self.node.create_publisher(
            Float64MultiArray,
            '/teleop_controller/commands', 
            1
        )

        self.speed_scale_client = self.node.create_client(
            SetFloat64, 
            '/teleop_controller/scale_speed'
        )
        
        # Interner Zustand
        self.limits = []
        self.num_joints = 0
        self.movement_enabled = False
                
        # Listen für dynamische UI-Elemente
        self.sliders = []
        self.target_labels = []
        self.actual_labels = []
        self.last_sent_positions = []

        # Haupt-Layout vorbereiten (bleibt leer, bis set_axis_limits aufgerufen wird)
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)
        
        # Initialer Platzhalter
        self.placeholder_label = QLabel("Waiting for URDF joint limits...")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.main_layout.addWidget(self.placeholder_label)

    def set_axis_limits(self, axis_data: list):
        """
        Erwartet Liste von Tuples: [(min, max, v_max_rad_s), ...]
        v_max_rad_s: Maximale Geschwindigkeit in Radiant pro Sekunde.
        """
        if not axis_data:
            return
            
        self.num_joints = len(axis_data)
        self.limits = []
        self.velocity_limits = []
        self.last_sent_positions = [0.0] * self.num_joints

        for min_rad, max_rad, _ in axis_data:
            # 1. Positionslimits für die Slider (in Grad)
            min_deg = math.degrees(min_rad)
            max_deg = math.degrees(max_rad)
            self.limits.append((min_deg, max_deg))

        # Platzhalter entfernen
        self.main_layout.removeWidget(self.placeholder_label)
        self.placeholder_label.deleteLater()

        # --- 1. HEADER & UNLOCK BUTTON ---
        header_layout = QHBoxLayout()
        header = QLabel("<b>Manual Axis Control</b>")
        header.setStyleSheet("font-size: 16px;")
        header_layout.addWidget(header)
        header_layout.addStretch()
        
        self.btn_toggle_lock = QPushButton("ENABLE MOVEMENT")
        self.btn_toggle_lock.setCheckable(True)
        self.btn_toggle_lock.setFixedWidth(150)
        self.btn_toggle_lock.setFixedHeight(30)
        self.btn_toggle_lock.clicked.connect(self.on_lock_toggle_clicked)
        header_layout.addWidget(self.btn_toggle_lock)
        
        self.main_layout.addLayout(header_layout)

        # --- 2. SPEED OVERRIDE SECTION ---
        self.setup_speed_slider_ui()

        # --- 3. DYNAMISCHE ACHSEN ---
        self.scroll_area = QFrame()
        self.scroll_area.setFrameShape(QFrame.Shape.StyledPanel)
        scroll_layout = QVBoxLayout(self.scroll_area)

        for i in range(self.num_joints):
            axis_layout = QVBoxLayout()
            
            # Zeile 1: Name und Echte Position (ACT)
            label_row = QHBoxLayout()
            name_label = QLabel(f"Axis {i+1}")
            name_label.setStyleSheet("font-weight: bold; color: #ecf0f1;")
            
            actual_display = QLabel("ACT: 0.00°")
            actual_display.setStyleSheet("font-family: monospace; color: #2ecc71; font-weight: bold;")
            
            label_row.addWidget(name_label)
            label_row.addStretch()
            label_row.addWidget(actual_display)
            axis_layout.addLayout(label_row)

            # Zeile 2: Slider und Soll-Position (SET)
            slider_row = QHBoxLayout()
            min_val, max_val = self.limits[i]
            
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(min_val * 100), int(max_val * 100))
            slider.setValue(0)
            
            target_display = QLabel("SET: 0.00°")
            target_display.setFixedWidth(80)
            target_display.setStyleSheet("font-family: monospace; color: #3498db;")
            
            slider.valueChanged.connect(lambda val, td=target_display: td.setText(f"SET: {val/100:.2f}°"))
            
            slider_row.addWidget(slider)
            slider_row.addWidget(target_display)
            
            axis_layout.addLayout(slider_row)
            scroll_layout.addLayout(axis_layout)
            
            self.sliders.append(slider)
            self.actual_labels.append(actual_display)
            self.target_labels.append(target_display)

        self.main_layout.addWidget(self.scroll_area)
        self.main_layout.addStretch()

        # Initialen Sperr-Zustand erzwingen
        self.update_ui_lock_state()

    def setup_speed_slider_ui(self):
        speed_container = QFrame()
        speed_container.setStyleSheet("background-color: #2c3e50; border-radius: 8px; border: 1px solid #3498db;")
        speed_layout = QVBoxLayout(speed_container)
        
        speed_header = QHBoxLayout()
        speed_title = QLabel("Speed Scale")
        speed_title.setStyleSheet("font-weight: bold; color: #3498db; border: none;")
        self.speed_display = QLabel("100%")
        self.speed_display.setStyleSheet("font-weight: bold; color: #f1c40f; border: none;")
        
        speed_header.addWidget(speed_title)
        speed_header.addStretch()
        speed_header.addWidget(self.speed_display)
        speed_layout.addLayout(speed_header)
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 200)
        self.speed_slider.setValue(100)
        
        # --- HIER DIE ÄNDERUNG ---
        # valueChanged ändert NUR die Textanzeige (flüssig beim Ziehen)
        self.speed_slider.valueChanged.connect(lambda val: self.speed_display.setText(f"{val}%"))
        
        # sliderReleased feuert erst, wenn die Maus losgelassen wird (Absetzen!)
        self.speed_slider.sliderReleased.connect(self.on_speed_slider_released)
        
        speed_layout.addWidget(self.speed_slider)
        self.main_layout.addWidget(speed_container)

    def on_speed_slider_released(self):
        val = self.speed_slider.value()
        target = val / 100
        if not self.speed_scale_client.service_is_ready():
            self.node.get_logger().warn("Speed service '/teleop_controller/set_speed' is not available!")
            return
        try:
            req = SetFloat64.Request()
            req.data = float(target)
            future = self.speed_scale_client.call_async(req)
            
            # Inline callback to handle the service response safely
            def cb(fut):
                try:
                    res = fut.result()
                    if not res.success:
                        self.node.get_logger().error(f"Speed scale request rejected: {res.message}")
                except Exception as e:
                    self.node.get_logger().error(f"Speed service call crashed: {e}")

            future.add_done_callback(cb)
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to send speed scale service request: {e}")

    def on_lock_toggle_clicked(self, is_checked):
        """Wird aufgerufen, wenn der Benutzer manuell auf den Freischalt-Button klickt."""
        self.btn_toggle_lock.blockSignals(True)
        self.btn_toggle_lock.setChecked(self.movement_enabled)
        self.btn_toggle_lock.blockSignals(False)
        
        if is_checked:
            req_type = RequestAction.Request.TYPE_START
        else:
            req_type = RequestAction.Request.TYPE_STOP

        self.request_action_client.send_request(
            action_id=RequestAction.Request.ACTION_MISSION,
            request_type=req_type,
            node_name=self.name,
            controller_names=self.required_controller)

    def update_ui_lock_state(self):
        """Aktiviert oder deaktiviert alle Eingabeelemente basierend auf movement_enabled."""
        for slider in self.sliders:
            slider.setEnabled(self.movement_enabled)
        self.speed_slider.setEnabled(self.movement_enabled)

        if self.movement_enabled:
            self.btn_toggle_lock.setText("DISENGAGE MOVEMENT")
            self.btn_toggle_lock.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
            self.btn_toggle_lock.setChecked(True)
        else:
            self.btn_toggle_lock.setText("ENABLE MOVEMENT")
            self.btn_toggle_lock.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
            self.btn_toggle_lock.setChecked(False)

    def set_movement_allowed(self, allowed: bool):
        if self.movement_enabled != allowed:
            self.movement_enabled = allowed
            self.update_ui_lock_state()
            
            # Sicherheits-Feature: Beim frisch Freischalten snappen wir die Slider 
            # sofort auf den echten Roboterzustand, um Start-Sprünge zu verhindern
            if self.movement_enabled:
                self.sync_sliders_to_actual()

    def update_widget(self):
        """
        Wird zyklisch (z.B. mit 50Hz) vom übergeordneten GUI-Timer aufgerufen.
        Nimmt jetzt keine Argumente mehr entgegen.
        """
        if self.num_joints == 0:
            axis_data = self.store.get_joint_limits()
            self.set_axis_limits(axis_data)
            return

        # 1. Aktuelle IST-Werte aus dem Store holen und anzeigen
        actual_positions_rad = self.store.get_current_positions() 
        
        # DIREKTE KONVERTIERUNG: Für die Anzeige im Widget rechnen wir Radian in Grad um
        self.current_values_deg = [math.degrees(rad) for rad in actual_positions_rad]
        for i, val in enumerate(self.current_values_deg):
            if i < len(self.actual_labels):
                self.actual_labels[i].setText(f"ACT: {val:.2f}°")

        status = self.store.get_status()
        active_nodes_string = status["active_nodes"]
        if self.name in active_nodes_string:
            self.set_movement_allowed(True)
        else:
            self.set_movement_allowed(False)

        # 2. Zyklisches Streaming an den Roboter (nur wenn von FSM freigegeben)
        if self.movement_enabled:
            target_positions_deg = [s.value() / 100.0 for s in self.sliders]
            for i in range(self.num_joints):
                self.target_labels[i].setText(f"SET: {target_positions_deg[i]:.2f}°")
            msg = Float64MultiArray()
            msg.data = [math.radians(deg) for deg in target_positions_deg]
            self.joint_target_pub.publish(msg)

    def sync_sliders_to_actual(self):
        """Setzt die Slider exakt dorthin, wo der Roboter gerade physikalisch steht."""
        if hasattr(self, 'current_values_deg'):
            for i, val in enumerate(self.current_values_deg):
                if i < len(self.sliders):
                    self.sliders[i].blockSignals(True)
                    self.sliders[i].setValue(int(val * 100))
                    self.target_labels[i].setText(f"SET: {val:.2f}°")
                    self.sliders[i].blockSignals(False)
            self.last_sent_positions = list(self.current_values_deg)