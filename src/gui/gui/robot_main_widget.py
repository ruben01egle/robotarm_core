from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QStackedWidget, QTextEdit)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QTextCursor
import os

# Deine Widgets importieren
from .telemetry_widget import TelemetryDashboard
from .motues_config_widget import RobotParameterConfig 
from .trajectory_widget import TrajectoryControlWidget 
from .manual_control_widget import ManualControlWidget 
from .control_header_widget import ControlHeader

class RobotMainWindow(QMainWindow):
    def __init__(self, guiDataStore):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.data_store = guiDataStore

        # --- HAUPT LAYOUT ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # 1. OBEN: Navigation (Haupt-Modi)
        self.control_header = ControlHeader(self.data_store)

        # 2. MITTE: Side-by-Side Content
        self.content_layout = QHBoxLayout()
        
        # LINKS: Der Control Stack
        self.init_nav_bar()
        self.left_column_container = QWidget()
        self.left_column_container.setFixedWidth(400) # Die Breite gilt nun für die ganze Spalte
        self.left_column_layout = QVBoxLayout(self.left_column_container)
        self.left_column_layout.setContentsMargins(0, 0, 0, 0)

        self.control_stack = QStackedWidget()
        self.control_stack.setFixedWidth(400) 
        
        self.traj_page = TrajectoryControlWidget(self.data_store)
        self.manual_page = ManualControlWidget(self.data_store)
        self.config_page = RobotParameterConfig(self.data_store)
        
        self.control_stack.addWidget(self.traj_page)    # Index 0
        self.control_stack.addWidget(self.manual_page)  # Index 1
        self.control_stack.addWidget(self.config_page)  # Index 2

        self.left_column_layout.addWidget(self.nav_container) 
        self.left_column_layout.addWidget(self.control_stack)
        
        # RECHTS: Das neue, selbstverwaltete Dashboard
        self.dashboard = TelemetryDashboard(self.data_store)

        # Zusammenfügen der Mitte
        self.content_layout.addWidget(self.left_column_container)
        self.content_layout.addWidget(self.dashboard, stretch=1)

        self.main_layout.addWidget(self.control_header, stretch=1)
        self.main_layout.addLayout(self.content_layout, stretch=4)

        # 3. UNTEN: Log-Konsole
        self.init_log_console()

        self.resize(1400, 1000)

        # --- SIGNALE VERBINDEN ---
        self.config_page.request_param_update.connect(self.log_param_change)
        self.manual_page.request_move.connect(self.log_manual_move)
        self.traj_page.start_trajectory.connect(self.handle_start_traj)

        # Timer (ca. 30 FPS für flüssige Plots)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.global_update)
        self.update_timer.start(33)

        self.log_message("System initialized. Dashboard and Controls ready.")

    def init_nav_bar(self):
        """Erstellt die obere Leiste zum Umschalten der linken Control-Seite."""
        self.nav_container = QWidget()
        self.nav_layout = QVBoxLayout(self.nav_container)
        
        self.btn_traj = QPushButton("TRAJECTORY")
        self.btn_manual = QPushButton("MANUAL")
        self.btn_config = QPushButton("CONFIG")
        
        for btn, idx in [(self.btn_traj, 0), (self.btn_manual, 1), (self.btn_config, 2)]:
            btn.setCheckable(True)
            btn.setAutoExclusive(True)
            btn.setMinimumHeight(40)
            btn.setStyleSheet("""
                    QPushButton { 
                        background-color: #3d3d3d; color: white; 
                    }
                    QPushButton:checked { 
                        background-color: #27ae60;  /* Ein schönes Grün für den aktiven Modus */
                        border: 2px solid #2ecc71;
                    }
                """)
            # Lambda nutzt hier den Default-Parameter i=idx, um den Scope zu fixieren
            btn.clicked.connect(lambda checked, i=idx: self.switch_gui_mode(i))
            self.nav_layout.addWidget(btn)
        self.btn_traj.setChecked(True)
        self.main_layout.addWidget(self.nav_container)

    def init_log_console(self):
        """Erstellt die Konsole am unteren Rand."""
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumHeight(120) 
        self.log_console.setMinimumHeight(60)
        self.log_console.setStyleSheet("""
            background-color: #1e1e1e; 
            color: #d4d4d4; 
            font-family: 'Consolas', monospace;
            font-size: 11px;
            border-top: 2px solid #3d3d3d;
        """)
        self.main_layout.addWidget(self.log_console, stretch=0)

    def switch_gui_mode(self, index):
        """Schaltet nur den linken Control-Stack um."""
        self.control_stack.setCurrentIndex(index)
        modes = ["TRAJECTORY", "MANUAL", "CONFIGURATION"]
        self.log_message(f"Gui Mode changed to: {modes[index]}")

    def global_update(self):
        """Zentraler Timer-Aufruf. Das Dashboard aktualisiert alle seine Tiles selbst."""
        self.dashboard.update_all()
        self.manual_page.update_actual_positions()
        self.control_header.update_status()
        self.config_page.update_params()
        self.traj_page.update_progress()

    # --- Event Handler für Logging ---
    def log_message(self, message):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_console.append(f"[{ts}] {message}")
        self.log_console.moveCursor(QTextCursor.MoveOperation.End)

    def log_param_change(self, axis_id, param, value):
        self.log_message(f"PARAM UPDATE: Axis {axis_id} | {param} set to {value}")

    def log_manual_move(self, positions):
        """Loggt den Befehl für alle 6 Achsen gleichzeitig."""
        # Erstellt einen String wie: J1: 10.2° | J2: -5.0° | ...
        pos_strings = [f"J{i+1}: {pos:.1f}°" for i, pos in enumerate(positions)]
        formatted_msg = " | ".join(pos_strings)
        
        self.log_message(f"MANUAL MOVE CMD: {formatted_msg}")

    def handle_start_traj(self, path):
        filename = os.path.basename(path)
        self.log_message(f"EXECUTING TRAJECTORY: {filename}")