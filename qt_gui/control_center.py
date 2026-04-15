from PyQt6.QtWidgets import QMainWindow, QWidget, QGridLayout
from PyQt6.QtCore import QTimer

from qt_gui.joint_tiles import JointTile
from qt_gui.data_store import DataStore
from .mock_package import MockPacket

import time

class RobotControlCenter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.resize(1200, 800)

        # Zentrales Widget und Layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.grid_layout = QGridLayout(self.central_widget)

        self.data_store = DataStore()
        # Liste für unsere 6 Gelenk-Tiles
        self.tiles = []
        self.sim_idx = 0

        # Erstelle 6 Tiles (z.B. 2 Reihen, 3 Spalten)
        for i in range(6):
            tile = JointTile(i+1, self.data_store)
            self.tiles.append(tile)
            
            # Berechne Position im Grid: row 0 oder 1, col 0, 1 oder 2
            row = i // 3
            col = i % 3
            self.grid_layout.addWidget(tile, row, col)

        # Timer für das GUI-Update (z.B. alle 50ms -> 20 FPS)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.global_update)
        self.update_timer.start(10)

    def global_update(self):
        #1. MOCK DATEN GENERIEREN
        self.sim_idx += 1
        curr_time = int(time.time() * 1e6)
        
        # Erzeuge ein Ist-Paket (Actual)
        telemetry_mock = MockPacket(self.sim_idx, curr_time)
        
        # Erzeuge ein Soll-Paket (Target) 
        # (etwas versetzt oder glatter, damit man den Unterschied sieht)
        trajectory_mock = MockPacket(self.sim_idx, curr_time)
        for i in range(1, 7):
            ax = getattr(trajectory_mock, f"axis{i}")
            ax.position += 0.2  # Kleiner Offset für die rote Referenzlinie
        
        # 2. IN DEN STORE PUSHEN
        # Wir nutzen deine push_frame Methode (die mit dem Index-Check)
        try:
            self.data_store.push_frame(trajectory_mock, telemetry_mock)
        except ValueError as e:
            print(f"Error: {e}")
        for tile in self.tiles:
            tile.update_plots()