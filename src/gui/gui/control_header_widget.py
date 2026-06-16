from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QVBoxLayout
from PyQt6.QtCore import Qt

from utility.RequestActionClient import RequestActionClient
from robotarm_interface.srv import RequestAction

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
        # Dunkles Design für den Header
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

        # --- SEKTION: KONNEKTIVITÄT ---
        self.conn_label = QLabel("● DISCONNECTED")
        self.conn_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 15px;")
        layout.addWidget(self.conn_label)
        
        # --- SEKTION: STATE MACHINE ---
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

        # --- SEKTION: ACTIVE NODES & CONTROLLERS (Neu) ---
        self.info_layout = QVBoxLayout()
        self.info_layout.setContentsMargins(0, 0, 0, 0)
        self.info_layout.setSpacing(2)
        self.info_layout.setAlignment(Qt.AlignmentFlag.AlignVCenter)
        
        self.nodes_label = QLabel("Nodes: none")
        self.nodes_label.setStyleSheet("color: #3498db; font-family: monospace; font-size: 18px;")
        
        self.controllers_label = QLabel("Controllers: none")
        self.controllers_label.setStyleSheet("color: #3498db; font-family: monospace; font-size: 18px;")
        
        self.info_layout.addWidget(self.nodes_label)
        self.info_layout.addWidget(self.controllers_label)
        layout.addLayout(self.info_layout)

        # Platzhalter nach rechts schieben
        layout.addStretch()

        # --- SEKTION: AKTIONEN ---
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

        # Stop (Dynamisches Farb-Design via update_status)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setCheckable(True)
        self.btn_stop.setFixedHeight(42)
        self.btn_stop.setFixedWidth(110)
        # Basis-Style, Farben werden dynamisch in update_status gesetzt
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
        """Zentrale Update-Logik für den Header."""
        status = self.store.get_status()
        connected = status["connected"]
        state = status["state"]
        armed = status["armed"]
        stopped = status["stopped"]
        active_nodes = status["active_nodes"]
        active_controllers = status["active_controllers"]
        
        # 1. Verbindung anlegen
        color = "#2ecc71" if connected else "#e74c3c"
        status_text = "● CONNECTED" if connected else "● DISCONNECTED"
        self.conn_label.setText(status_text)
        self.conn_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 15px;")

        # 2. State
        self.state_label.setText(f"STATE: {state.upper()}")

        # 3. Active Nodes & Controllers Texte aktualisieren
        self.nodes_label.setText(f"Nodes: {active_nodes if active_nodes else 'none'}")
        self.controllers_label.setText(f"Controllers: {active_controllers if active_controllers else 'none'}")

        # 4. Arm-Button Sync
        if armed != self.btn_arm.isChecked():
            self.btn_arm.blockSignals(True)
            self.btn_arm.setChecked(armed)
            self.btn_arm.blockSignals(False)
        self.btn_arm.setText("DISARM" if armed else "ARM ROBOT")

        # 5. Stop-Button Sync & Dynamisches Styling (Rot/Grün Wechsel)
        if stopped != self.btn_stop.isChecked():
            self.btn_stop.blockSignals(True)
            self.btn_stop.setChecked(stopped)  # FEHLER BEHOBEN: War vorher fälschlicherweise 'armed'
            self.btn_stop.blockSignals(False)
        
        if stopped:
            # Roboter steht -> Button bietet "RESUME" an (Grün)
            self.btn_stop.setText("RESUME")
            self.btn_stop.setStyleSheet("""
                QPushButton { 
                    background-color: #27ae60; color: white; font-weight: bold; border-radius: 4px; font-size: 13px; 
                }
                QPushButton:hover { background-color: #2ecc71; }
            """)
        else:
            # Roboter läuft -> Button bietet "STOP" an (Kräftiges Rot/Orange)
            self.btn_stop.setText("STOP")
            self.btn_stop.setStyleSheet("""
                QPushButton { 
                    background-color: #d35400; color: white; font-weight: bold; border-radius: 4px; font-size: 13px; 
                }
                QPushButton:hover { background-color: #e67e22; }
            """)