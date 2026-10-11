from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction


class CartesianImpedanceWidget(QWidget):
    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node
        self.name = "CartesianImpedanceWidget"
        self.required_controller = "cartesian_impedance_controller"

        self.request_action_client = RequestActionClient(self.node)

        # Interner Zustand
        self.movement_enabled = False
        self.disengage_requested = False

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)

        self.init_ui()

    def init_ui(self):
        # --- 1. HEADER & UNLOCK BUTTON ---
        header_layout = QHBoxLayout()
        header = QLabel("<b>Cartesian Impedance</b>")
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

        # --- 3. MODUS-HINWEIS ---
        # Controller macht aktuell nur Gravitationskompensation
        mode_hint = QLabel("Gravity compensation: the arm can be moved by hand "
                           "and stays where it is left.")
        mode_hint.setWordWrap(True)
        mode_hint.setStyleSheet("color: #95a5a6; font-size: 11px;")
        self.main_layout.addWidget(mode_hint)

        # --- 4. IMPEDANZ (später: Referenz-Pose / Trajektorie, kartesische Steifigkeit) ---

        self.main_layout.addStretch()

        # Initialen Sperr-Zustand erzwingen
        self.update_ui_lock_state()

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
            if not self.disengage_requested:
                text = f"Controller disengaged by system (state: {state}). See log for reason."
                self.warning_label.setText(f"⚠ {text}")
                self.warning_label.show()
                self.node.get_logger().warn(f"{self.name}: {text}")
            self.disengage_requested = False

        self.update_ui_lock_state()

    def update_widget(self):
        """Wird zyklisch vom übergeordneten GUI-Timer aufgerufen (~30 Hz)."""
        status = self.store.get_status()
        self.set_movement_allowed(self.name in status["active_nodes"], status["state"])
