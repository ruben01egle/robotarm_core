from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QMessageBox
from PyQt6.QtCore import Qt

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction

from .controller_modes import mode_name

class ControlHeader(QWidget):
    def __init__(self, store, node):
        super().__init__()
        self.store = store
        self.node = node
        self.request_action_client = RequestActionClient(self.node)
        self.setFixedHeight(60)
        self.init_ui()
        self.name = "ControlHeaderWidget"

    def init_ui(self):
        # Dark style for the header
        self.setStyleSheet("""
            ControlHeader {
                background-color: #2c3e50;
                border-bottom: 2px solid #34495e;
            }
            QLabel { color: #ecf0f1; font-size: 12px; }
        """)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(20, 0, 20, 0)
        layout.setSpacing(20)

        # --- SECTION: CONNECTIVITY ---
        self.conn_label = QLabel("● DISCONNECTED")
        self.conn_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 15px;")
        layout.addWidget(self.conn_label)
        
        # --- SECTION: STATE MACHINE ---
        self.state_label = QLabel("STATE: UNKNOWN")
        self.state_label.setStyleSheet("""
            background-color: #1a252f; 
            color: #3498db; 
            padding: 8px 18px; 
            border-radius: 6px; 
            font-weight: bold;
            font-size: 15px; 
            border: 2px solid #3498db;
        """)
        layout.addWidget(self.state_label)

        # --- SECTION: ACTIVE NODE & CONTROLLERS ---
        self.info_layout = QVBoxLayout()
        self.info_layout.setContentsMargins(0, 0, 0, 0)
        self.info_layout.setSpacing(2)
        self.info_layout.setAlignment(Qt.AlignmentFlag.AlignVCenter)
        
        self.nodes_label = QLabel("Nodes: none")
        self.nodes_label.setStyleSheet("color: #3498db; font-family: monospace; font-size: 18px;")
        
        self.mode_label = QLabel("Mode: unknown")
        self.mode_label.setStyleSheet("color: #3498db; font-family: monospace; font-size: 18px;")
        
        self.info_layout.addWidget(self.nodes_label)
        self.info_layout.addWidget(self.mode_label)
        layout.addLayout(self.info_layout)

        # Spacer, pushes the following to the right
        layout.addStretch()

        # --- SECTION: ACTIONS ---
        # Arm Robot
        self.btn_arm = QPushButton("ARM ROBOT")
        self.btn_arm.setCheckable(True)
        self.btn_arm.setFixedHeight(42)
        self.btn_arm.setFixedWidth(130)
        self.btn_arm.setStyleSheet("""
            QPushButton { 
                background-color: #27ae60; color: white; font-weight: bold; border-radius: 4px; font-size: 13px;
            }
            QPushButton:hover { background-color: #2ecc71; }
            QPushButton:checked { 
                background-color: #d35400; 
                border: 2px solid #e67e22;
            }
        """)
        self.btn_arm.clicked.connect(self.handle_arm_click)
        layout.addWidget(self.btn_arm)

        # Stop (colours set dynamically in update_status)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setCheckable(True)
        self.btn_stop.setFixedHeight(42)
        self.btn_stop.setFixedWidth(110)
        # Base style, colours are set dynamically in update_status
        self.btn_stop.setStyleSheet("color: white; font-weight: bold; border-radius: 4px; font-size: 13px;")
        self.btn_stop.clicked.connect(self.handle_stop_click)
        layout.addWidget(self.btn_stop)

        # Emergency
        self.btn_emergency = QPushButton("EMERGENCY")
        self.btn_emergency.setFixedHeight(42)
        self.btn_emergency.setStyleSheet("""
            QPushButton { 
                background-color: #c0392b; color: white; font-weight: bold; 
                padding: 0 22px; border-radius: 4px; font-size: 13px;
            }
            QPushButton:hover { background-color: #e74c3c; border-color: white; }
            QPushButton:pressed { background-color: #962d22; }
        """)
        self.btn_emergency.clicked.connect(self.handle_emergency_click)
        layout.addWidget(self.btn_emergency)

    def handle_emergency_click(self):
        self.request_action_client.send_request(
            action_id=RequestAction.Request.INVOKE_EMERGENCY,
            request_type=RequestAction.Request.TYPE_START,
            node_name=self.name)

    def handle_arm_click(self):
        is_checked = self.btn_arm.isChecked()
        if is_checked:
            req_type = RequestAction.Request.TYPE_START
        else:
            # Disarming (LIMP) powers off the drives wherever the arm is
            answer = QMessageBox.question(
                self, "Disarm",
                "Disarming powers off the drives: the arm falls wherever it is. Disarm now?")
            if answer != QMessageBox.StandardButton.Yes:
                self.btn_arm.blockSignals(True)
                self.btn_arm.setChecked(True)
                self.btn_arm.blockSignals(False)
                return
            req_type = RequestAction.Request.TYPE_STOP
        self.request_action_client.send_request(
            action_id=RequestAction.Request.ACTION_ARM_ROBOT,
            request_type=req_type,
            node_name=self.name)

    def handle_stop_click(self):
        is_checked = self.btn_stop.isChecked()
        if is_checked:
            req_type = RequestAction.Request.TYPE_START
        else:
            req_type = RequestAction.Request.TYPE_STOP
        self.request_action_client.send_request(
            action_id=RequestAction.Request.INVOKE_STOP,
            request_type=req_type,
            node_name=self.name)

    def update_status(self):
        """Update the whole header from the status."""
        status = self.store.get_status()
        connected = status["connected"]
        state = status["state"]
        armed = status["armed"]
        stopped = status["stopped"]
        active_node = status["active_node"]
        mode = status["mode"]
        
        # 1. Connection
        color = "#2ecc71" if connected else "#e74c3c"
        status_text = "● CONNECTED" if connected else "● DISCONNECTED"
        self.conn_label.setText(status_text)
        self.conn_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 15px;")

        # 2. State
        self.state_label.setText(f"STATE: {state.upper()}")

        # 3. Active node & firmware mode
        self.nodes_label.setText(f"Nodes: {active_node if active_node else 'none'}")
        mode_text = mode_name(mode) if mode is not None else "unknown"
        self.mode_label.setText(f"Mode: {mode_text}{' (switching...)' if status['mode_switching'] else ''}")

        # 4. Sync the arm button
        if armed != self.btn_arm.isChecked():
            self.btn_arm.blockSignals(True)
            self.btn_arm.setChecked(armed)
            self.btn_arm.blockSignals(False)
        self.btn_arm.setText("DISARM" if armed else "ARM ROBOT")
        # The firmware rejects requests while switching; STOP and EMERGENCY stay usable
        # (control_center retries STOP, EMERGENCY_OFF aborts the switch)
        self.btn_arm.setEnabled(not status["mode_switching"])

        # 5. Sync the stop button and its colour (red/green)
        if stopped != self.btn_stop.isChecked():
            self.btn_stop.blockSignals(True)
            self.btn_stop.setChecked(stopped)
            self.btn_stop.blockSignals(False)
        
        if stopped:
            # Robot stopped: the button offers "RESUME" (green)
            self.btn_stop.setText("RESUME")
            self.btn_stop.setStyleSheet("""
                QPushButton { 
                    background-color: #27ae60; color: white; font-weight: bold; border-radius: 4px; font-size: 13px; 
                }
                QPushButton:hover { background-color: #2ecc71; }
            """)
        else:
            # Robot running: the button offers "STOP" (red/orange)
            self.btn_stop.setText("STOP")
            self.btn_stop.setStyleSheet("""
                QPushButton { 
                    background-color: #d35400; color: white; font-weight: bold; border-radius: 4px; font-size: 13px; 
                }
                QPushButton:hover { background-color: #e67e22; }
            """)