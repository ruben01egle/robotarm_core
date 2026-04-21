from PyQt6.QtWidgets import (QMainWindow, QWidget, QGridLayout, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QStackedWidget, QTextEdit)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QTextCursor

import os

# Deine Widgets importieren (Pfade ggf. anpassen)
from qt_gui.joint_tiles import JointTile
from qt_gui.motues_config_widget import RobotParameterConfig 
from qt_gui.trajectory_widget import TrajectoryControlWidget # Das neue Traj-Widget
from qt_gui.manual_control_widget import ManualControlWidget # Das neue Manual-Widget

class RobotMainWindow(QMainWindow):
    def __init__(self, guiDataStore):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.resize(1400, 900) # Etwas breiter für das Side-by-Side Layout
        self.data_store = guiDataStore

        # --- HAUPT LAYOUT ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # 1. OBEN: Erweiterte Navigation
        self.init_nav_bar()

        # 2. MITTE: Das Herzstück (Side-by-Side)
        self.content_layout = QHBoxLayout()
        
        # LINKS: Der Control Stack (Wechselt zwischen Modi)
        self.control_stack = QStackedWidget()
        self.control_stack.setFixedWidth(400) # Feste Breite für die Bedienelemente
        
        # Instanzen der Unter-Widgets erstellen
        self.config_page = RobotParameterConfig()
        self.traj_page = TrajectoryControlWidget()
        self.manual_page = ManualControlWidget()
        
        # Widgets zum Stack hinzufügen
        self.control_stack.addWidget(self.traj_page)    # Index 0
        self.control_stack.addWidget(self.manual_page)  # Index 1
        self.control_stack.addWidget(self.config_page)  # Index 2
        
        # RECHTS: Das permanente Dashboard
        self.init_dashboard()

        # Zusammenfügen
        self.content_layout.addWidget(self.control_stack)
        self.content_layout.addWidget(self.dashboard_widget, stretch=1)
        self.main_layout.addLayout(self.content_layout, stretch=4)

        # 3. UNTEN: Log-Konsole
        self.init_log_console()

        # Signale verbinden
        self.config_page.request_param_update.connect(self.log_param_change)
        self.manual_page.request_move.connect(self.log_manual_move)
        self.traj_page.start_trajectory.connect(self.handle_start_traj)

        # Timer
        self.refresh_time =33
        self.refresh_counter = 0
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.global_update)
        self.update_timer.start(self.refresh_time)

    def init_nav_bar(self):
        self.nav_layout = QHBoxLayout()
        
        # Drei Buttons für die drei Modi
        self.btn_traj = QPushButton("TRAJECTORY")
        self.btn_manual = QPushButton("MANUAL")
        self.btn_config = QPushButton("CONFIG")
        
        for btn, idx in [(self.btn_traj, 0), (self.btn_manual, 1), (self.btn_config, 2)]:
            btn.setMinimumHeight(50)
            btn.setStyleSheet("font-weight: bold;")
            btn.clicked.connect(lambda checked, i=idx: self.switch_control_mode(i))
            self.nav_layout.addWidget(btn)

        self.main_layout.addLayout(self.nav_layout)

    def init_dashboard(self):
        self.dashboard_widget = QWidget()
        self.dashboard_widget.setStyleSheet("background-color: #2b2b2b; border-radius: 5px;")
        grid = QGridLayout(self.dashboard_widget)
        self.tiles = []
        for i in range(6):
            tile = JointTile(i+1, self.data_store)
            self.tiles.append(tile)
            grid.addWidget(tile, i // 2, i % 2) # 3 Zeilen, 2 Spalten für Side-Layout besser

    def init_log_console(self):
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: monospace;")
        self.main_layout.addWidget(self.log_console, stretch=1)

    def switch_control_mode(self, index):
        self.control_stack.setCurrentIndex(index)
        modes = ["TRAJECTORY", "MANUAL", "CONFIGURATION"]
        self.log_message(f"Switched to {modes[index]} mode.")

    def global_update(self):
        for tile in self.tiles:
            tile.update_plots()

    # --- Event Handler ---
    def log_message(self, message):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_console.append(f"[{ts}] {message}")
        self.log_console.moveCursor(QTextCursor.MoveOperation.End)

    def log_param_change(self, axis_id, param, value):
        self.log_message(f"PARAM UPDATE: Axis {axis_id} | {param} = {value}")

    def log_manual_move(self, axis_id, position):
        self.log_message(f"MANUAL MOVE: Axis {axis_id} to {position}°")

    def handle_start_traj(self, path):
        self.log_message(f"START TRAJECTORY: {os.path.basename(path)}")