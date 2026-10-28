import math

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QFrame, QApplication, QButtonGroup
from PyQt6.QtCore import Qt

from geometry_msgs.msg import TwistStamped

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction
from robotarm_interface.msg import CartesianIncrement

from .controller_modes import (ExecutionToggle, CARTESIAN_JOG_CONTROLLERS, POSITION, COMPLIANT,
                               controller_set, set_available)

# Slider-Auflösung: -SLIDER_RESOLUTION ... +SLIDER_RESOLUTION entspricht -v_max ... +v_max
SLIDER_RESOLUTION = 1000

# Schrittweite pro Klick (Controller begrenzt auf max. 3 mm / 3° pro Nachricht)
INC_LINEAR = 0.001               # m (1 mm)
INC_ANGULAR = math.radians(1.0)  # rad (1°)
INC_LABELS = {"linear": "1 mm", "angular": "1°"}

# header.frame_id für die Jog-Controller
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

        self.request_action_client = RequestActionClient(self.node)
        # Je Modus eine Instanz des CartesianJogControllers, gleiche Schnittstelle unter anderem Namen.
        # Twist-Protokoll: nur während aktivem Jog zyklisch senden (Watchdog 250 ms), beim Loslassen
        # genau EIN Null-Twist, im Leerlauf nichts. Jeder Twist (auch Null) bricht ein laufendes Inkrement ab.
        self.twist_pubs = {
            mode: self.node.create_publisher(TwistStamped, f'/{name}/twist_cmds', 10)
            for mode, name in CARTESIAN_JOG_CONTROLLERS.items()
        }
        # Inkremente: genau eine Nachricht pro Klick, relativ zur aktuellen Position, nicht streamen
        self.increment_pubs = {
            mode: self.node.create_publisher(CartesianIncrement, f'/{name}/increment_cmds', 10)
            for mode, name in CARTESIAN_JOG_CONTROLLERS.items()
        }

        # Interner Zustand
        self.jogging = False
        self.frame_id = FRAME_BASE
        # Controller-Name -> (max_linear, max_angular), Limits können je Instanz verschieden sein
        self.velocity_limits = {}
        self.ui_ready = False
        self.movement_enabled = False
        self.disengage_requested = False

        self.sliders = []
        self.target_labels = []
        self.limit_labels = []
        self.increment_buttons = []

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

    def build_ui(self, limits: dict):
        """Wird aufgebaut, sobald die Limits mindestens einer Jog-Instanz gelesen wurden."""
        if not limits:
            return

        self.velocity_limits = limits

        # Platzhalter entfernen
        self.main_layout.removeWidget(self.placeholder_label)
        self.placeholder_label.hide()
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

        # Position (cartesian_jog_controller) | Compliant (impedance + cartesian_jog_controller_impedance)
        self.execution_toggle = ExecutionToggle()
        self.execution_toggle.mode_changed.connect(lambda _: self.update_limit_labels())
        self.main_layout.addWidget(self.execution_toggle)

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
        # Umbrechen statt die Spalte zu verbreitern (Tool-Hinweis ist deutlich länger)
        self.frame_hint.setWordWrap(True)
        frame_row.addWidget(self.frame_hint, stretch=1)
        frame_row.addStretch()
        self.main_layout.addLayout(frame_row)

        # --- 4. ACHSEN ---
        self.axes_frame = QFrame()
        self.axes_frame.setFrameShape(QFrame.Shape.StyledPanel)
        axes_layout = QVBoxLayout(self.axes_frame)

        for i, (axis_name, short_name, field, _, unit) in enumerate(self.AXES):
            axis_layout = QVBoxLayout()

            # Zeile 1: Name und Limit (Text je Modus in update_limit_labels)
            label_row = QHBoxLayout()
            name_label = QLabel(axis_name)
            name_label.setStyleSheet("font-weight: bold;")

            limit_label = QLabel("")
            self.limit_labels.append(limit_label)
            limit_label.setStyleSheet("font-family: monospace; color: #95a5a6; font-size: 10px;")

            label_row.addWidget(name_label)
            label_row.addStretch()
            label_row.addWidget(limit_label)
            axis_layout.addLayout(label_row)

            # Zeile 2: Inkrement-Buttons, Slider und Soll-Geschwindigkeit
            slider_row = QHBoxLayout()
            axis_tag = QLabel(short_name)
            axis_tag.setFixedWidth(32)
            axis_tag.setAlignment(Qt.AlignmentFlag.AlignCenter)
            axis_tag.setStyleSheet("""
                background-color: #34495e; color: white; font-weight: bold;
                border-radius: 3px; padding: 2px;
            """)
            slider = SpringBackSlider()

            # Nur der Sollwert, die Einheit steht schon beim Limit
            target_display = QLabel(f"{0.0:+.3f}")
            target_display.setFixedWidth(52)
            target_display.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            target_display.setStyleSheet("font-family: monospace; color: #3498db;")

            slider.valueChanged.connect(lambda val, idx=i: self.update_target_label(idx, self.slider_to_velocity(idx, val)))

            # Inkrement-Buttons: minus links, plus rechts vom Slider (wie die Slider-Richtung)
            btn_minus = self.make_increment_button(i, -1, f"−{INC_LABELS[field]}", short_name)
            btn_plus = self.make_increment_button(i, +1, f"+{INC_LABELS[field]}", short_name)

            slider_row.addWidget(axis_tag)
            slider_row.addWidget(btn_minus)
            slider_row.addWidget(slider)
            slider_row.addWidget(btn_plus)
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
        self.update_limit_labels()
        self.update_ui_lock_state()

    def make_increment_button(self, idx, sign, text, short_name):
        btn = QPushButton(text)
        btn.setFixedWidth(52)
        btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        btn.setToolTip(f"Single step of {text} along {short_name} in the selected frame.\n"
                       "Ignored by the controller while jogging or braking.")
        btn.clicked.connect(lambda _, idx=idx, sign=sign: self.publish_increment(idx, sign))
        self.increment_buttons.append(btn)
        return btn

    def execution_mode(self) -> str:
        return self.execution_toggle.mode() if self.ui_ready else None

    def current_limits(self):
        """Limits der Instanz des gewählten Modus oder None, solange nicht gelesen."""
        return self.velocity_limits.get(CARTESIAN_JOG_CONTROLLERS[self.execution_toggle.mode()])

    def axis_max(self, idx):
        limits = self.current_limits()
        if limits is None:
            return 0.0
        return limits[0] if self.AXES[idx][2] == "linear" else limits[1]

    def update_limit_labels(self):
        known = self.current_limits() is not None
        for i, label in enumerate(self.limit_labels):
            label.setText(f"max ±{self.axis_max(i):.3f} {self.AXES[i][4]}" if known else "limits not read yet")
        # Ohne Limits keine Freigabe möglich (Slider wären wirkungslos)
        self.btn_toggle_lock.setEnabled(known or self.movement_enabled)

    def slider_to_velocity(self, idx, slider_value):
        return slider_value / SLIDER_RESOLUTION * self.axis_max(idx)

    def update_target_label(self, idx, velocity):
        self.target_labels[idx].setText(f"{velocity:+.3f}")

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
        self.twist_pubs[self.execution_toggle.mode()].publish(msg)

    def publish_increment(self, idx, sign):
        """Genau eine Inkrement-Nachricht pro Klick für eine Achse."""
        _, _, field, component, _ = self.AXES[idx]
        step = INC_LINEAR if field == "linear" else INC_ANGULAR

        msg = CartesianIncrement()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        setattr(getattr(msg, field), component, float(sign * step))
        self.increment_pubs[self.execution_toggle.mode()].publish(msg)

    def zero_all(self):
        """Setzt alle Slider auf 0 und sendet sofort einen Null-Twist (Soft-Stop, bricht auch Inkremente ab)."""
        if not self.ui_ready:
            return
        for i, slider in enumerate(self.sliders):
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
            self.update_target_label(i, 0.0)
        self.jogging = False
        self.publish_twist([0.0] * len(self.AXES))

    def shutdown(self):
        """Beim Schließen der GUI: Null-Twist an beide Instanzen senden, damit der Arm nicht weiterfährt."""
        try:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            for _ in range(3):
                for pub in self.twist_pubs.values():
                    pub.publish(msg)
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
            controller_names=controller_set(CARTESIAN_JOG_CONTROLLERS, self.execution_toggle.mode()))

    def update_ui_lock_state(self):
        """Aktiviert oder deaktiviert alle Eingabeelemente basierend auf movement_enabled."""
        for widget in self.sliders + self.increment_buttons:
            widget.setEnabled(self.movement_enabled)
        self.execution_toggle.set_locked(self.movement_enabled)
        self.update_limit_labels()

        if self.movement_enabled:
            self.btn_toggle_lock.setText("DISABLE MOVEMENT")
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
        """Wird zyklisch vom übergeordneten GUI-Timer aufgerufen (~30 Hz) und streamt den Twist während des Jogs."""
        if not self.ui_ready:
            self.build_ui(self.store.get_cartesian_limits())
            return

        limits = self.store.get_cartesian_limits()
        if limits != self.velocity_limits:
            self.velocity_limits = limits
            self.update_limit_labels()

        loaded = self.store.get_loaded_controllers()
        self.execution_toggle.set_available(
            set_available(controller_set(CARTESIAN_JOG_CONTROLLERS, POSITION), loaded),
            set_available(controller_set(CARTESIAN_JOG_CONTROLLERS, COMPLIANT), loaded))

        status = self.store.get_status()
        self.set_movement_allowed(status["active_node"] == self.name, status["state"])

        if not self.movement_enabled:
            return

        # Nur während aktivem Jog streamen; beim Loslassen genau ein Null-Twist, im Leerlauf nichts
        velocities = self.get_target_velocities()
        if any(v != 0.0 for v in velocities):
            self.jogging = True
            self.publish_twist(velocities)
        elif self.jogging:
            self.jogging = False
            self.publish_twist(velocities)
