from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton
from PyQt6.QtCore import Qt, pyqtSignal

class ControlHeader(QWidget):
    """
    Zentrale Kontrollleiste für den Roboter (oben platziert).
    Vereint Monitoring (Status) und kritische Kommandos (Arm, Stop).
    """
    emergency_pressed = pyqtSignal()
    stop_pressed = pyqtSignal()
    arm_toggled = pyqtSignal(bool)

    def __init__(self, store):
        super().__init__()
        self.store = store
        self.setFixedHeight(60) # Auf 60px erhöht, damit größere Widgets Platz haben
        self.init_ui()

    def init_ui(self):
        # Dunkles Design für den Header, um sich vom Rest abzuheben
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
        
        # --- SEKTION: STATE MACHINE (Vergrößert und Präsenter) ---
        self.state_label = QLabel("STATE: UNKNOWN")
        self.state_label.setStyleSheet("""
            background-color: #1a252f; 
            color: #3498db; 
            padding: 8px 18px; /* Mehr Padding für mehr physische Größe */
            border-radius: 6px; /* Modernerer, leicht abgerundeter Look statt Pille */
            font-weight: bold;
            font-size: 15px; /* Größere Schriftart */
            border: 2px solid #3498db;
        """)

        # --- SEKTION: AKTIONEN ---
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

        # Hard Stop (Kräftiges Orange/Hellrot - Sofortiger Achsstopp)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setFixedHeight(42)
        self.btn_stop.setStyleSheet("""
            QPushButton { 
                background-color: #e65100; color: white; font-weight: bold; 
                padding: 0 22px; border-radius: 4px; font-size: 13px;
            }
            QPushButton:hover { background-color: #f39c12; }
            QPushButton:pressed { background-color: #d35400; }
        """)
        self.btn_stop.clicked.connect(self.stop_pressed.emit)

        # Emergency Stop (Dominantes Signalrot mit hellem Kontrastrahmen - Strom weg)
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
        self.btn_emergency.clicked.connect(self.emergency_pressed.emit)

        # Zusammenbau
        layout.addWidget(self.conn_label)
        layout.addWidget(self.state_label)
        layout.addStretch()
        layout.addWidget(self.btn_arm)
        layout.addWidget(self.btn_stop)
        layout.addWidget(self.btn_emergency)

    def handle_arm_click(self):
        is_checked = self.btn_arm.isChecked()
        self.arm_toggled.emit(is_checked)

    def update_status(self):
        """Zentrale Update-Logik für den Header."""
        status = self.store.get_status()
        connected = status["connected"]
        state = status["state"]
        armed = status["armed"]
        
        # Verbindung
        color = "#2ecc71" if connected else "#e74c3c"
        status_text = "● CONNECTED" if connected else "● DISCONNECTED"
        self.conn_label.setText(status_text)
        self.conn_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 15px;")

        # State
        self.state_label.setText(f"STATE: {state.upper()}")

        # Arm-Button Sync
        if armed != self.btn_arm.isChecked():
            self.btn_arm.blockSignals(True)
            self.btn_arm.setChecked(armed)
            self.btn_arm.blockSignals(False)
        
        self.btn_arm.setText("DISARM" if armed else "ARM ROBOT")