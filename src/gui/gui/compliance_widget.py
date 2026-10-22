from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction

from .controller_modes import GC_CONTROLLER, IMPEDANCE_CONTROLLER, set_available

HAND_GUIDING = "hand_guiding"
COMPLIANT_HOLD = "compliant_hold"

# (Titel, Beschreibung, Controller-Set)
MODES = {
    HAND_GUIDING: ("Hand guiding",
                   "Arm floats, move it by hand, stays where it is left.",
                   [GC_CONTROLLER]),
    COMPLIANT_HOLD: ("Compliant hold",
                     "TCP held at the activation pose by a virtual spring.",
                     [IMPEDANCE_CONTROLLER]),
}


class ComplianceWidget(QWidget):
    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node
        self.name = "ComplianceWidget"

        self.request_action_client = RequestActionClient(self.node)

        # Interner Zustand: None, HAND_GUIDING oder COMPLIANT_HOLD
        self.active_mode = None
        self.disengage_requested = False
        self.available = {mode: True for mode in MODES}

        self.buttons = {}

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)

        self.init_ui()

    def init_ui(self):
        # --- 1. HEADER ---
        header = QLabel("<b>Compliance</b>")
        header.setStyleSheet("font-size: 16px;")
        self.main_layout.addWidget(header)

        # --- 2. MODI (je eine Karte mit eigenem Enable-Button) ---
        for mode, (title, description, _) in MODES.items():
            card = QFrame()
            card.setFrameShape(QFrame.Shape.StyledPanel)
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(8, 6, 8, 6)

            title_label = QLabel(f"<b>{title}</b>")
            card_layout.addWidget(title_label)

            row = QHBoxLayout()
            desc_label = QLabel(description)
            desc_label.setWordWrap(True)
            desc_label.setStyleSheet("color: #95a5a6; font-size: 11px;")
            row.addWidget(desc_label, stretch=1)

            btn = QPushButton("ENABLE")
            btn.setCheckable(True)
            btn.setFixedWidth(90)
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda checked, m=mode: self.on_mode_clicked(m, checked))
            row.addWidget(btn)
            card_layout.addLayout(row)

            self.buttons[mode] = btn
            self.main_layout.addWidget(card)

        # --- 3. WARNUNG BEI UNERWARTETEM ABSCHALTEN ---
        self.warning_label = QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet("""
            background-color: #2c1a1a; color: #e74c3c; font-weight: bold;
            border: 1px solid #e74c3c; border-radius: 6px; padding: 6px;
        """)
        self.warning_label.hide()
        self.main_layout.addWidget(self.warning_label)

        self.main_layout.addStretch()

        # Initialen Sperr-Zustand erzwingen
        self.update_ui_lock_state()

    def on_mode_clicked(self, mode, is_checked):
        """Wird aufgerufen, wenn der Benutzer manuell auf einen Enable-Button klickt."""
        btn = self.buttons[mode]
        btn.blockSignals(True)
        btn.setChecked(self.active_mode == mode)
        btn.blockSignals(False)

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
            controller_names=MODES[mode][2])

    def update_ui_lock_state(self):
        for mode, btn in self.buttons.items():
            active = self.active_mode == mode
            btn.setChecked(active)
            # Nur ein Modus gleichzeitig, nicht geladene Controller ausgrauen
            btn.setEnabled(active or (self.active_mode is None and self.available[mode]))
            if active:
                btn.setText("DISABLE")
                btn.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
            else:
                btn.setText("ENABLE")
                btn.setStyleSheet("""
                    QPushButton { background-color: #2ecc71; color: white; font-weight: bold; }
                    QPushButton:disabled { background-color: #3d3d3d; color: #7f8c8d; }
                """)

    def set_active_mode(self, mode, state: str):
        if self.active_mode == mode:
            return

        if mode is not None:
            self.disengage_requested = False
            self.warning_label.hide()
        elif not self.disengage_requested:
            text = f"Controller disengaged by system (state: {state}). See log for reason."
            self.warning_label.setText(f"⚠ {text}")
            self.warning_label.show()
            self.node.get_logger().warn(f"{self.name}: {text}")
        else:
            self.disengage_requested = False

        self.active_mode = mode
        self.update_ui_lock_state()

    def update_widget(self):
        """Wird zyklisch vom übergeordneten GUI-Timer aufgerufen (~30 Hz)."""
        loaded = self.store.get_loaded_controllers()
        available = {mode: set_available(controllers, loaded) for mode, (_, _, controllers) in MODES.items()}
        if available != self.available:
            self.available = available
            self.update_ui_lock_state()

        status = self.store.get_status()
        mode = None
        if self.name in status["active_nodes"]:
            mode = HAND_GUIDING if GC_CONTROLLER in self.store.get_active_controller_list() else COMPLIANT_HOLD
        self.set_active_mode(mode, status["state"])
