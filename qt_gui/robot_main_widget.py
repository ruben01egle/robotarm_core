from PyQt6.QtWidgets import QMainWindow, QWidget, QGridLayout
from PyQt6.QtCore import QTimer

from qt_gui.joint_tiles import JointTile
from qt_gui.data_store import GuiDataStore


class RobotMainWindow(QMainWindow):
    def __init__(self, guiDataStore):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.resize(1200, 800)

        # Zentrales Widget und Layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.grid_layout = QGridLayout(self.central_widget)

        self.data_store = guiDataStore
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
        """Holt nur noch die Daten ab, die der Thread reingeschrieben hat."""
        for tile in self.tiles:
            tile.update_plots()