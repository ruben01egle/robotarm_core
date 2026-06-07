from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QScrollArea, QSizePolicy,
                             QPushButton, QStackedWidget, QGridLayout, QFrame, QLabel)
from PyQt6.QtCore import Qt, pyqtSlot
import functools

from .joint_tiles_widget import JointTile

class TelemetryDashboard(QWidget):
    def __init__(self, store):
        super().__init__()
        self.store = store
        
        # Interner Zustand für dynamischen Aufbau
        self.tiles = []
        self.num_joints = 0
        self.nav_buttons = [] # Halten wir fest, um sie später zu löschen/ändern

        # Haupt-Layout für das gesamte Widget
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)
        
        # Initialer Platzhalter, bis URDF geladen ist
        self.placeholder_label = QLabel("Waiting for URDF joint limits to initialize telemetry...")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder_label.setStyleSheet("color: #7f8c8d; font-size: 14px;")
        self.main_layout.addWidget(self.placeholder_label)

    @pyqtSlot(list)
    def set_axis_limits(self, new_limits_rad: list):
        """
        Wird beim Startup von außen aufgerufen, sobald die URDF geladen ist.
        Baut die gesamte UI dynamisch für X Achsen auf.
        """
        if not new_limits_rad or self.num_joints > 0:
            return # Verhindert doppelte Initialisierung
            
        self.num_joints = len(new_limits_rad)
        
        # 1. Kacheln dynamisch erzeugen (ID startet bei 1)
        self.tiles = [JointTile(i + 1) for i in range(self.num_joints)]

        # 2. Platzhalter entfernen
        self.main_layout.removeWidget(self.placeholder_label)
        self.placeholder_label.deleteLater()

        # --- Dynamische UI aufbauen ---
        
        # --- 1. Navigationsleiste (Bleibt oben fixiert) ---
        self.nav_bar = QHBoxLayout()
        self.btn_grid = QPushButton("All Axis")
        self.btn_grid.setStyleSheet("font-weight: bold; background-color: #34495e; color: white;")
        self.btn_grid.clicked.connect(self.show_grid)
        self.nav_bar.addWidget(self.btn_grid)
        
        # Buttons dynamisch für jede Achse generieren
        for i in range(self.num_joints):
            btn = QPushButton(f"J{i+1}")
            btn.clicked.connect(functools.partial(self.show_focus, i))
            self.nav_bar.addWidget(btn)
            self.nav_buttons.append(btn)
        
        self.main_layout.addLayout(self.nav_bar)

        # --- 2. Die Scroll Area erstellen ---
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.scroll_area.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        
        # --- 3. Das Stacked Widget als Inhalt für die Scroll Area ---
        self.container = QWidget()
        self.container_layout = QVBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)

        self.display_stack = QStackedWidget()
        
        # Ansicht A: Das dynamische Grid
        self.grid_container = QWidget()
        self.grid_layout = QGridLayout(self.grid_container)
        
        # Spalten- und Zeilen-Stretch dynamisch anpassen (immer 2 Spalten)
        self.grid_layout.setColumnStretch(0, 1)
        self.grid_layout.setColumnStretch(1, 1)
        
        num_rows = (self.num_joints + 1) // 2
        for r in range(num_rows):
            self.grid_layout.setRowStretch(r, 1)
        
        # Ansicht B: Der Fokus-Bereich
        self.focus_container = QWidget()
        self.focus_layout = QVBoxLayout(self.focus_container)
        self.focus_layout.setContentsMargins(0, 0, 0, 0)
        
        self.display_stack.addWidget(self.grid_container) # Index 0
        self.display_stack.addWidget(self.focus_container) # Index 1
        
        # Stack in den Container, Container in die ScrollArea
        self.container_layout.addWidget(self.display_stack)
        self.scroll_area.setWidget(self.container)

        # ScrollArea zum Haupt-Layout hinzufügen
        self.main_layout.addWidget(self.scroll_area)
        
        # Initialisierung der ersten Ansicht
        self.show_grid()

    def clear_layout(self, layout):
        if layout is not None:
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.setParent(None)
                    widget.hide()

    def show_grid(self):
        # Nur ausführen, wenn die UI bereits über das Signal initialisiert wurde
        if self.num_joints == 0:
            return
        self.display_stack.setCurrentIndex(0)
        self.clear_layout(self.grid_layout)
        for i, tile in enumerate(self.tiles):
            tile.show()
            self.grid_layout.addWidget(tile, i // 2, i % 2)

    def show_focus(self, index):
        if self.num_joints == 0:
            return
        self.display_stack.setCurrentIndex(1)
        self.clear_layout(self.focus_layout)
        target_tile = self.tiles[index]
        target_tile.show()
        self.focus_layout.addWidget(target_tile)

    def update_all(self):
        # Solange keine Achsen da sind, müssen wir auch nichts plotten
        if self.num_joints == 0:
            return
            
        frames = self.store.get_plot_data()
        if not frames:
            return
        for tile in self.tiles:
            tile.update_plots(frames)