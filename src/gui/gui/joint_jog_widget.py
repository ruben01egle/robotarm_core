from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QFrame
from PyQt6.QtCore import Qt
import math

from std_msgs.msg import Float64MultiArray

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction, SetFloat64

from .controller_modes import (ExecutionToggle, JOINT_STREAM_CONTROLLERS, JOINT_STREAM_MODES, POSITION,
                               COMPLIANT, mode_available)

class JointJogWidget(QWidget):
    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node
        self.name = "JointJogWidget"

        self.request_action_client = RequestActionClient(self.node)
        # Command input of each mode: same JointStreamController interface under a different name
        self.joint_target_pubs = {
            mode: self.node.create_publisher(Float64MultiArray, f'/{name}/commands', 1)
            for mode, name in JOINT_STREAM_CONTROLLERS.items()
        }
        self.speed_scale_clients = {
            mode: self.node.create_client(SetFloat64, f'/{name}/scale_speed')
            for mode, name in JOINT_STREAM_CONTROLLERS.items()
        }
        
        # Internal state
        self.limits = []
        self.num_joints = 0
        self.movement_enabled = False
                
        # Lists of the dynamic UI elements
        self.sliders = []
        self.target_labels = []
        self.actual_labels = []
        self.last_sent_positions = []

        # Main layout (stays empty until set_axis_limits is called)
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)
        
        # Initial placeholder
        self.placeholder_label = QLabel("Waiting for URDF joint limits...")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.main_layout.addWidget(self.placeholder_label)

    def set_axis_limits(self, axis_data: list):
        """
        Expects a list of tuples: [(min, max, v_max_rad_s), ...]
        v_max_rad_s: maximum velocity in rad/s.
        """
        if not axis_data:
            return
            
        self.num_joints = len(axis_data)
        self.limits = []
        self.velocity_limits = []
        self.last_sent_positions = [0.0] * self.num_joints

        for min_rad, max_rad, _ in axis_data:
            # 1. Position limits of the sliders (degrees)
            min_deg = math.degrees(min_rad)
            max_deg = math.degrees(max_rad)
            self.limits.append((min_deg, max_deg))

        # Remove the placeholder
        self.main_layout.removeWidget(self.placeholder_label)
        self.placeholder_label.hide()
        self.placeholder_label.deleteLater()

        # --- 1. HEADER & UNLOCK BUTTON ---
        header_layout = QHBoxLayout()
        header = QLabel("<b>Joint Control</b>")
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

        # Position (JOINT_STREAM) | Compliant (IMPEDANCE_JOINT_STREAM)
        self.execution_toggle = ExecutionToggle()
        self.main_layout.addWidget(self.execution_toggle)

        # --- 2. SPEED OVERRIDE SECTION ---
        self.setup_speed_slider_ui()

        # --- 3. AXES (dynamic) ---
        self.scroll_area = QFrame()
        self.scroll_area.setFrameShape(QFrame.Shape.StyledPanel)
        scroll_layout = QVBoxLayout(self.scroll_area)

        for i in range(self.num_joints):
            axis_layout = QVBoxLayout()
            
            # Row 1: name and actual position (ACT)
            label_row = QHBoxLayout()
            name_label = QLabel(f"Axis {i+1}")
            name_label.setStyleSheet("font-weight: bold; color: #ecf0f1;")
            
            actual_display = QLabel("ACT: 0.00°")
            actual_display.setStyleSheet("font-family: monospace; color: #2ecc71; font-weight: bold;")
            
            label_row.addWidget(name_label)
            label_row.addStretch()
            label_row.addWidget(actual_display)
            axis_layout.addLayout(label_row)

            # Row 2: slider and target position (SET)
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

        # Apply the initial lock state
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
        # The controller only accepts (0, 1]
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(100)
        
        # valueChanged only updates the text (smooth while dragging)
        self.speed_slider.valueChanged.connect(lambda val: self.speed_display.setText(f"{val}%"))
        
        # sliderReleased fires only when the mouse is released
        self.speed_slider.sliderReleased.connect(self.on_speed_slider_released)
        
        speed_layout.addWidget(self.speed_slider)
        self.main_layout.addWidget(speed_container)

    def execution_mode(self) -> str:
        return self.execution_toggle.mode() if self.num_joints else None

    def on_speed_slider_released(self):
        self.send_speed_scale()

    def send_speed_scale(self):
        """Send the speed scale to the instance of the selected mode (each instance has its own value)."""
        mode = self.execution_toggle.mode()
        client = self.speed_scale_clients[mode]
        target = self.speed_slider.value() / 100
        if not client.service_is_ready():
            self.node.get_logger().warn(f"Speed service '/{JOINT_STREAM_CONTROLLERS[mode]}/scale_speed' is not available!")
            return
        try:
            req = SetFloat64.Request()
            req.data = float(target)
            future = client.call_async(req)
            
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
        """Called when the user clicks the unlock button."""
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
            mode=JOINT_STREAM_MODES[self.execution_toggle.mode()])

    def update_ui_lock_state(self):
        """Enable or disable all inputs according to movement_enabled."""
        for slider in self.sliders:
            slider.setEnabled(self.movement_enabled)
        self.speed_slider.setEnabled(self.movement_enabled)
        self.execution_toggle.set_locked(self.movement_enabled)

        if self.movement_enabled:
            self.btn_toggle_lock.setText("DISABLE MOVEMENT")
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
            
            # Safety: right after unlocking, snap the sliders 
            # to the actual robot state, so the robot does not jump at the start
            if self.movement_enabled:
                self.sync_sliders_to_actual()
                # Bring the freshly activated instance to the displayed speed scale
                self.send_speed_scale()

    def update_widget(self):
        """
        Called periodically by the main GUI timer (~30 Hz).
        """
        if self.num_joints == 0:
            axis_data = self.store.get_joint_limits()
            self.set_axis_limits(axis_data)
            return

        # 1. Show the current actual values from the store
        actual_positions_rad = self.store.get_current_positions() 
        
        # The store holds rad, the widget shows degrees
        self.current_values_deg = [math.degrees(rad) for rad in actual_positions_rad]
        for i, val in enumerate(self.current_values_deg):
            if i < len(self.actual_labels):
                self.actual_labels[i].setText(f"ACT: {val:.2f}°")

        available = self.store.get_available_modes()
        self.execution_toggle.set_available(
            mode_available(JOINT_STREAM_MODES[POSITION], available),
            mode_available(JOINT_STREAM_MODES[COMPLIANT], available))
        # While moving, the selected mode is the actual one and so available; nothing while switching
        self.btn_toggle_lock.setEnabled(
            mode_available(JOINT_STREAM_MODES[self.execution_toggle.mode()], available))

        status = self.store.get_status()
        if status["active_node"] == self.name:
            self.set_movement_allowed(True)
        else:
            self.set_movement_allowed(False)

        # 2. Stream to the robot periodically (only while the mission allows it)
        if self.movement_enabled:
            target_positions_deg = [s.value() / 100.0 for s in self.sliders]
            for i in range(self.num_joints):
                self.target_labels[i].setText(f"SET: {target_positions_deg[i]:.2f}°")
            msg = Float64MultiArray()
            msg.data = [math.radians(deg) for deg in target_positions_deg]
            self.joint_target_pubs[self.execution_toggle.mode()].publish(msg)

    def sync_sliders_to_actual(self):
        """Set the sliders exactly to where the robot currently is."""
        if hasattr(self, 'current_values_deg'):
            for i, val in enumerate(self.current_values_deg):
                if i < len(self.sliders):
                    self.sliders[i].blockSignals(True)
                    self.sliders[i].setValue(int(val * 100))
                    self.target_labels[i].setText(f"SET: {val:.2f}°")
                    self.sliders[i].blockSignals(False)
            self.last_sent_positions = list(self.current_values_deg)