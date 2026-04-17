from PyQt6.QtWidgets import (QMainWindow, QWidget, QGridLayout, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QStackedWidget, QTextEdit)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QTextCursor

from qt_gui.joint_tiles import JointTile
# Hier dein neues Widget importieren
from qt_gui.motues_config_widget import RobotParameterConfig 

class RobotMainWindow(QMainWindow):
    def __init__(self, guiDataStore):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.resize(1200, 900)
        self.data_store = guiDataStore

        # --- HAUPT LAYOUT ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # 1. OBEN: Navigation/Modus-Schalter
        self.nav_layout = QHBoxLayout()
        self.btn_toggle_config = QPushButton("MODE: Configuration / Telemetry")
        self.btn_toggle_config.setMinimumHeight(40)
        self.btn_toggle_config.setStyleSheet("font-weight: bold;")
        self.btn_toggle_config.clicked.connect(self.toggle_view_mode)
        self.nav_layout.addWidget(self.btn_toggle_config)
        self.main_layout.addLayout(self.nav_layout)

        # 2. MITTE: Stacked Widget für die Modi
        self.stack = QStackedWidget()
        
        # --- Seite 0: Telemetry Dashboard (Joint Tiles) ---
        self.dashboard_widget = QWidget()
        self.grid_layout = QGridLayout(self.dashboard_widget)
        self.tiles = []
        for i in range(6):
            tile = JointTile(i+1, self.data_store)
            self.tiles.append(tile)
            row, col = i // 3, i % 3
            self.grid_layout.addWidget(tile, row, col)
        
        # --- Seite 1: Global Configuration Widget ---
        self.config_page = RobotParameterConfig()
        # Verbindung des Signals (hier die Logik einfügen)
        self.config_page.request_param_update.connect(self.log_param_change)

        self.stack.addWidget(self.dashboard_widget) # Index 0
        self.stack.addWidget(self.config_page)      # Index 1
        self.main_layout.addWidget(self.stack, stretch=4) # Nimmt mehr Platz ein

        # 3. UNTEN: Log-Konsole (bleibt immer sichtbar)
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setStyleSheet("""
            background-color: #1e1e1e; 
            color: #d4d4d4; 
            font-family: 'Consolas', 'Monaco', monospace;
            font-size: 11px;
        """)
        self.log_console.setPlaceholderText("System Logs...")
        self.main_layout.addWidget(self.log_console, stretch=1) # Kleinerer Bereich

        # Timer starten
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.global_update)
        self.update_timer.start(33) # ~30 FPS reichen für die GUI völlig

        self.log_message("System started. Ready for telemetry.")

    def toggle_view_mode(self):
        """Schaltet zwischen Dashboard (0) und Config (1) um."""
        new_idx = 1 if self.stack.currentIndex() == 0 else 0
        self.stack.setCurrentIndex(new_idx)
        mode_name = "CONFIGURATION" if new_idx == 1 else "TELEMETRY"
        self.log_message(f"Switched to {mode_name} mode.")

    def log_message(self, message):
        """Hilfsfunktion um Nachrichten in die Konsole zu schreiben."""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_console.append(f"[{timestamp}] {message}")
        # Auto-Scroll nach unten
        self.log_console.moveCursor(QTextCursor.MoveOperation.End)

    def log_param_change(self, axis_id, param, value):
        """Wird aufgerufen, wenn im Config-Widget 'Apply' gedrückt wird."""
        # Hier später die Send_to_STM32 Logik!
        self.log_message(f"PARAM UPDATE: Axis {axis_id} | {param} set to {value}")

    def global_update(self):
        # Update nur, wenn das Dashboard sichtbar ist (Performance)
        #if self.stack.currentIndex() == 0:
        for tile in self.tiles:
            tile.update_plots()