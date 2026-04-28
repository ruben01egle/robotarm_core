import os
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QComboBox, QProgressBar, QFrame)

from PyQt6.QtCore import pyqtSignal

class TrajectoryControlWidget(QWidget):
    # Signale für das Backend
    start_trajectory = pyqtSignal(str) 
    stop_trajectory = pyqtSignal()

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self.init_ui()
        self.refresh_file_list()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)

        # Titel
        header = QLabel("<b>Trajectory Control</b>")
        header.setStyleSheet("font-size: 16px;")
        layout.addWidget(header)

        # 1. Dateiauswahl
        file_box = QFrame()
        file_box.setFrameShape(QFrame.Shape.StyledPanel)
        file_layout = QVBoxLayout(file_box)
        file_layout.addWidget(QLabel("Choose csv:"))
        
        self.file_selector = QComboBox()
        self.file_selector.setMinimumHeight(35)
        file_layout.addWidget(self.file_selector)

        self.btn_refresh = QPushButton("Refresh folder")
        self.btn_refresh.clicked.connect(self.refresh_file_list)
        file_layout.addWidget(self.btn_refresh)
        layout.addWidget(file_box)

        # 2. Controls
        ctrl_layout = QHBoxLayout()
        self.btn_start = QPushButton("START")
        self.btn_start.setFixedHeight(45)
        self.btn_start.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")
        
        # HIER war der Fehlerpunkt: Die Verbindung zum Slot
        self.btn_start.clicked.connect(self.on_start_clicked)
        
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setFixedHeight(45)
        self.btn_stop.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        
        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_stop)
        layout.addLayout(ctrl_layout)

        # 3. Progress
        self.progress_bar_planning = QProgressBar()
        layout.addWidget(QLabel("Planning Progress:"))
        layout.addWidget(self.progress_bar_planning)

        self.progress_bar_executing = QProgressBar()
        layout.addWidget(QLabel("Executing Progress:"))
        layout.addWidget(self.progress_bar_executing)
        layout.addStretch()

    # --- WICHTIG: Diese Methoden müssen auf der gleichen Ebene wie init_ui stehen ---

    def refresh_file_list(self):
        path = "trajectories"
        if not os.path.exists(path):
            os.makedirs(path)
        files = sorted([f for f in os.listdir(path) if f.endswith('.csv')])
        self.file_selector.clear()
        self.file_selector.addItems(files)

    def on_start_clicked(self):
        """Wird aufgerufen, wenn START gedrückt wird."""
        filename = self.file_selector.currentText()
        if filename:
            full_path = os.path.join("trajectories", filename)
            self.start_trajectory.emit(full_path)
            self.progress_bar_planning.setValue(0)
            self.progress_bar_executing.setValue(0)

    def on_stop_clicked(self):
        """Wird aufgerufen, wenn STOP gedrückt wird."""
        self.stop_trajectory.emit()

    def update_progress(self):
        prog_planning, prog_executing = self.store.get_progress()
        self.progress_bar_planning.setValue(int(prog_planning))
        self.progress_bar_executing.setValue(int(prog_executing))