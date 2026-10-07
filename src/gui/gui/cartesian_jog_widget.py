from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QFrame, QApplication, QButtonGroup
from PyQt6.QtCore import Qt

from geometry_msgs.msg import TwistStamped

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction

# Slider-Auflösung: -SLIDER_RESOLUTION ... +SLIDER_RESOLUTION entspricht -v_max ... +v_max
SLIDER_RESOLUTION = 1000

# header.frame_id für den cartesian_jog_controller
FRAME_BASE = ""      # leer = Root-Link der URDF (Basis), unabhängig vom Link-Namen
FRAME_TOOL = "tcp"   # TCP-Link, Name ist fest vorgegeben

FRAME_HINTS = {
    FRAME_BASE: "Base frame · rotation about TCP",
    FRAME_TOOL: "Tool frame (tcp) · axes follow tool orientation, rotation about TCP",
}


class SpringBackSlider(QSlider):
    """
    Slider, der beim Loslassen immer auf 0 zurückspringt.
    Mausrad und Tastatur sind deaktiviert, damit kein Wert "stehen bleiben" kann.
    """
    def __init__(self, parent=None):
        super().__init__(Qt.Orientation.Horizontal, parent)
        self.setRange(-SLIDER_RESOLUTION, SLIDER_RESOLUTION)
        self.setValue(0)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.setValue(0)

    def wheelEvent(self, event):
        event.ignore()

    def keyPressEvent(self, event):
        event.ignore()


class CartesianJogWidget(QWidget):
    # (Anzeigename, Kurzname, Twist-Feld, Komponente, Einheit)
    AXES = [
        ("Linear X", "X", "linear", "x", "m/s"),
        ("Linear Y", "Y", "linear", "y", "m/s"),
        ("Linear Z", "Z", "linear", "z", "m/s"),
        ("Angular X", "Rx", "angular", "x", "rad/s"),
        ("Angular Y", "Ry", "angular", "y", "rad/s"),
        ("Angular Z", "Rz", "angular", "z", "rad/s"),
    ]

    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node
        self.name = "CartesianJogWidget"
        self.required_controller = "cartesian_jog_controller"

        self.request_action_client = RequestActionClient(self.node)
        # Der Controller hat einen 250 ms Watchdog: Befehle werden daher zyklisch gesendet
        # und bei jeder Unterbrechung wird zusätzlich explizit ein Null-Twist geschickt.
        self.twist_pub = self.node.create_publisher(
            TwistStamped,
            '/cartesian_jog_controller/twist_cmds',
            10
        )

        # Interner Zustand
        self.frame_id = FRAME_BASE
        self.max_linear = 0.0
        self.max_angular = 0.0
        self.ui_ready = False
        self.movement_enabled = False
        self.disengage_requested = False

        self.sliders = []
        self.target_labels = []

        # Haupt-Layout vorbereiten (bleibt leer, bis die Controller-Limits gelesen wurden)
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)

        self.placeholder_label = QLabel("Waiting for cartesian jog controller parameters...")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.main_layout.addWidget(self.placeholder_label)

        # Fokusverlust des Fensters -> sofort stoppen
        app = QApplication.instance()
        if app is not None:
            app.applicationStateChanged.connect(self.on_application_state_changed)

    def set_velocity_limits(self, limits: tuple):
        """Erwartet (max_linear_velocity [m/s], max_angular_velocity [rad/s])."""
        if not limits:
            return

        self.max_linear, self.max_angular = limits

        # Platzhalter entfernen
        self.main_layout.removeWidget(self.placeholder_label)
        self.placeholder_label.deleteLater()

        # --- 1. HEADER & UNLOCK BUTTON ---
        header_layout = QHBoxLayout()
        header = QLabel("<b>Cartesian Jog Control</b>")
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

        # --- 2. WARNUNG BEI UNERWARTETEM ABSCHALTEN ---
        self.warning_label = QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet("""
            background-color: #2c1a1a; color: #e74c3c; font-weight: bold;
            border: 1px solid #e74c3c; border-radius: 6px; padding: 6px;
        """)
        self.warning_label.hide()
        self.main_layout.addWidget(self.warning_label)

        # --- 3. KOORDINATENSYSTEM (Base / Tool) ---
        # Umschalten ist auch während der Bewegung erlaubt, der Controller blendet weich über.
        frame_row = QHBoxLayout()
        btn_base = QPushButton("Base")
        btn_base.setToolTip("Jog along the fixed robot base axes")
        btn_tool = QPushButton("Tool")
        btn_tool.setToolTip("Linear: move along the tool's own x/y/z (e.g. tool z = towards workpiece).\n"
                            "Angular: rotate about the tool axes, tool tip stays in place.")

        self.frame_group = QButtonGroup(self)
        self.frame_group.setExclusive(True)
        for btn, frame_id in ((btn_base, FRAME_BASE), (btn_tool, FRAME_TOOL)):
            btn.setCheckable(True)
            btn.setFixedWidth(70)
            btn.setChecked(frame_id == self.frame_id)
            btn.toggled.connect(lambda checked, f=frame_id: checked and self.set_frame(f))
            self.frame_group.addButton(btn)
            frame_row.addWidget(btn)

        self.frame_hint = QLabel(FRAME_HINTS[self.frame_id])
        self.frame_hint.setStyleSheet("color: #95a5a6; font-size: 11px;")
        frame_row.addWidget(self.frame_hint)
        frame_row.addStretch()
        self.main_layout.addLayout(frame_row)

        # --- 4. ACHSEN ---
        self.axes_frame = QFrame()
        self.axes_frame.setFrameShape(QFrame.Shape.StyledPanel)
        axes_layout = QVBoxLayout(self.axes_frame)

        for i, (axis_name, short_name, _, _, unit) in enumerate(self.AXES):
            axis_layout = QVBoxLayout()
            v_max = self.axis_max(i)

            # Zeile 1: Name und Limit
            label_row = QHBoxLayout()
            name_label = QLabel(axis_name)
            name_label.setStyleSheet("font-weight: bold;")

            limit_label = QLabel(f"max ±{v_max:.3f} {unit}")
            limit_label.setStyleSheet("font-family: monospace; color: #95a5a6;")

            label_row.addWidget(name_label)
            label_row.addStretch()
            label_row.addWidget(limit_label)
            axis_layout.addLayout(label_row)

            # Zeile 2: Slider und Soll-Geschwindigkeit (SET)
            slider_row = QHBoxLayout()
            axis_tag = QLabel(short_name)
            axis_tag.setFixedWidth(32)
            axis_tag.setAlignment(Qt.AlignmentFlag.AlignCenter)
            axis_tag.setStyleSheet("""
                background-color: #34495e; color: white; font-weight: bold;
                border-radius: 3px; padding: 2px;
            """)
            slider = SpringBackSlider()

            target_display = QLabel(f"SET: {0.0:+.3f} {unit}")
            target_display.setFixedWidth(130)
            target_display.setStyleSheet("font-family: monospace; color: #3498db;")

            slider.valueChanged.connect(lambda val, idx=i: self.update_target_label(idx, self.slider_to_velocity(idx, val)))

            slider_row.addWidget(axis_tag)
            slider_row.addWidget(slider)
            slider_row.addWidget(target_display)
            axis_layout.addLayout(slider_row)
            axes_layout.addLayout(axis_layout)

            self.sliders.append(slider)
            self.target_labels.append(target_display)

        self.main_layout.addWidget(self.axes_frame)

        # --- 5. STOP / ZERO ALL ---
        self.btn_zero_all = QPushButton("STOP / ZERO ALL")
        self.btn_zero_all.setFixedHeight(34)
        self.btn_zero_all.setStyleSheet("""
            QPushButton {
                background-color: #c0392b; color: white; font-weight: bold;
                border-radius: 4px; font-size: 13px;
            }
            QPushButton:hover { background-color: #e74c3c; }
            QPushButton:pressed { background-color: #962d22; }
        """)
        self.btn_zero_all.clicked.connect(self.zero_all)
        self.main_layout.addWidget(self.btn_zero_all)

        self.main_layout.addStretch()

        self.ui_ready = True

        # Initialen Sperr-Zustand erzwingen
        self.update_ui_lock_state()

    def axis_max(self, idx):
        return self.max_linear if self.AXES[idx][2] == "linear" else self.max_angular

    def slider_to_velocity(self, idx, slider_value):
        return slider_value / SLIDER_RESOLUTION * self.axis_max(idx)

    def update_target_label(self, idx, velocity):
        unit = self.AXES[idx][4]
        self.target_labels[idx].setText(f"SET: {velocity:+.3f} {unit}")

    def get_target_velocities(self):
        return [self.slider_to_velocity(i, s.value()) for i, s in enumerate(self.sliders)]

    def set_frame(self, frame_id):
        self.frame_id = frame_id
        self.frame_hint.setText(FRAME_HINTS[frame_id])

    def publish_twist(self, velocities):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for (_, _, field, component, _), vel in zip(self.AXES, velocities):
            setattr(getattr(msg.twist, field), component, float(vel))
        self.twist_pub.publish(msg)

    def zero_all(self):
        """Setzt alle Slider auf 0 und sendet sofort einen Null-Twist."""
        for i, slider in enumerate(self.sliders):
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
            self.update_target_label(i, 0.0)
        self.publish_twist([0.0] * len(self.AXES))

    def shutdown(self):
        """Beim Schließen der GUI: Null-Twist senden, damit der Arm nicht weiterfährt."""
        try:
            for _ in range(3):
                self.publish_twist([0.0] * len(self.AXES))
        except Exception as e:
            print(f"[CartesianJogWidget] Failed to send zero twist on shutdown: {e}")

    def on_application_state_changed(self, state):
        if state != Qt.ApplicationState.ApplicationActive:
            self.zero_all()

    def hideEvent(self, event):
        # Seitenwechsel im Control-Stack
        self.zero_all()
        super().hideEvent(event)

    def on_lock_toggle_clicked(self, is_checked):
        """Wird aufgerufen, wenn der Benutzer manuell auf den Freischalt-Button klickt."""
        self.btn_toggle_lock.blockSignals(True)
        self.btn_toggle_lock.setChecked(self.movement_enabled)
        self.btn_toggle_lock.blockSignals(False)

        if is_checked:
            req_type = RequestAction.Request.TYPE_START
        else:
            req_type = RequestAction.Request.TYPE_STOP
            self.disengage_requested = True
            self.zero_all()

        def cb(response):
            # Abgelehnter Disengage -> späteres Abschalten wäre wieder "unerwartet"
            if not is_checked and not response.success:
                self.disengage_requested = False

        self.request_action_client.send_request(
            action_id=RequestAction.Request.ACTION_MISSION,
            request_type=req_type,
            user_cb=cb,
            node_name=self.name,
            controller_names=self.required_controller)

    def update_ui_lock_state(self):
        """Aktiviert oder deaktiviert alle Eingabeelemente basierend auf movement_enabled."""
        for slider in self.sliders:
            slider.setEnabled(self.movement_enabled)

        if self.movement_enabled:
            self.btn_toggle_lock.setText("DISENGAGE MOVEMENT")
            self.btn_toggle_lock.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
            self.btn_toggle_lock.setChecked(True)
        else:
            self.btn_toggle_lock.setText("ENABLE MOVEMENT")
            self.btn_toggle_lock.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
            self.btn_toggle_lock.setChecked(False)

    def set_movement_allowed(self, allowed: bool, state: str):
        if self.movement_enabled == allowed:
            return

        self.movement_enabled = allowed

        if allowed:
            self.disengage_requested = False
            self.warning_label.hide()
        else:
            # Immer mit Null-Slidern in den gesperrten Zustand
            self.zero_all()
            if not self.disengage_requested:
                text = f"Movement disengaged by system (state: {state}). See log for reason."
                self.warning_label.setText(f"⚠ {text}")
                self.warning_label.show()
                self.node.get_logger().warn(f"{self.name}: {text}")
            self.disengage_requested = False

        self.update_ui_lock_state()

    def update_widget(self):
        """Wird zyklisch vom übergeordneten GUI-Timer aufgerufen (~30 Hz) und streamt den Twist."""
        if not self.ui_ready:
            self.set_velocity_limits(self.store.get_cartesian_limits())
            return

        status = self.store.get_status()
        self.set_movement_allowed(self.name in status["active_nodes"], status["state"])

        # Zyklisches Streaming an den Controller (auch Null-Twists), nur wenn von FSM freigegeben
        if self.movement_enabled:
            self.publish_twist(self.get_target_velocities())
