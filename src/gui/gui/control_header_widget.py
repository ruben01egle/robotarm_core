from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QFrame
from PyQt6.QtCore import Qt, pyqtSignal

class ControlHeader(QWidget):
    """
    Zentrale Kontrollleiste für den Roboter.
    Vereint Monitoring (Status, Latenz) und kritische Kommandos (Arm, Stop).
    """
    emergency_stop_pressed = pyqtSignal()
    arm_toggled = pyqtSignal(bool)

    def __init__(self, store):
        super().__init__()
        self.store = store
        self.setFixedHeight(60) # Etwas mehr Höhe für die Buttons
        self.init_ui()

    def init_ui(self):
        # Dunkles Design für den Header, um sich vom Rest abzuheben
        self.setStyleSheet("""
            RobotMasterHeader {
                background-color: #2c3e50;
                border-bottom: 2px solid #34495e;
            }
            QLabel { color: #ecf0f1; font-size: 12px; }
        """)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 0, 15, 0)
        layout.setSpacing(20)

        # --- SEKTION: KONNEKTIVITÄT ---
        self.conn_label = QLabel("● DISCONNECTED")
        self.conn_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 14px;")
        
        # --- SEKTION: STATE MACHINE ---
        self.state_label = QLabel("STATE: UNKNOWN")
        self.state_label.setStyleSheet("""
            background-color: #1a252f; 
            color: #3498db; 
            padding: 5px 12px; 
            border-radius: 15px; 
            font-weight: bold;
            border: 1px solid #3498db;
        """)

        # --- SEKTION: NETZWERK ---
        self.latency_label = QLabel("LATENCY: -- ms")
        self.latency_label.setStyleSheet("font-family: 'Consolas'; color: #95a5a6;")

        # --- SEKTION: AKTIONEN (Rechtsbündig) ---
        self.btn_arm = QPushButton("ARM ROBOT")
        self.btn_arm.setCheckable(True)
        self.btn_arm.setFixedHeight(38)
        self.btn_arm.setFixedWidth(130)
        self.btn_arm.setStyleSheet("""
            QPushButton { 
                background-color: #27ae60; color: white; font-weight: bold; border-radius: 4px;
            }
            QPushButton:checked { 
                background-color: #d35400; /* Orange Warnfarbe wenn aktiv */
                border: 2px solid #e67e22;
            }
        """)
        self.btn_arm.clicked.connect(self.handle_arm_click)

        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setFixedHeight(38)
        self.btn_emergency.setStyleSheet("""
            QPushButton { 
                background-color: #c0392b; color: white; font-weight: bold; 
                padding: 0 25px; border-radius: 4px; font-size: 13px;
            }
            QPushButton:hover { background-color: #e74c3c; }
            QPushButton:pressed { background-color: #962d22; }
        """)
        self.btn_emergency.clicked.connect(self.emergency_stop_pressed.emit)

        # Zusammenbau
        layout.addWidget(self.conn_label)
        layout.addWidget(self.state_label)
        layout.addWidget(self.latency_label)
        layout.addStretch()
        layout.addWidget(self.btn_arm)
        layout.addWidget(self.btn_emergency)

    def handle_arm_click(self):
        is_checked = self.btn_arm.isChecked()
        # Text ändert sich erst, wenn wir wirklich "Armed" sind (wird über update_telemetry gesteuert)
        self.arm_toggled.emit(is_checked)

    def update_status(self):
        """Zentrale Update-Logik für den Header."""
        status = self.store.get_status()
        connected = status["connected"]
        state = status["state"]
        latency = status["latency"]
        armed = status["armed"]
        # Verbindung
        color = "#2ecc71" if connected else "#e74c3c"
        status_text = "● CONNECTED" if connected else "● DISCONNECTED"
        self.conn_label.setText(status_text)
        self.conn_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 14px;")

        # State
        self.state_label.setText(f"STATE: {state.upper()}")
        
        # Latency
        self.latency_label.setText(f"LATENCY: {latency} ms")
        if latency > 150: self.latency_label.setStyleSheet("color: #e67e22;")
        else: self.latency_label.setStyleSheet("color: #95a5a6;")

        # Arm-Button Sync (Wichtig: Signale blockieren, um Endlosschleife zu verhindern)
        if armed != self.btn_arm.isChecked():
            self.btn_arm.blockSignals(True)
            self.btn_arm.setChecked(armed)
            self.btn_arm.blockSignals(False)
        
        self.btn_arm.setText("DISARM" if armed else "ARM ROBOT")