from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QScrollArea, QSizePolicy,
                             QPushButton, QStackedWidget, QGridLayout, QFrame)
from PyQt6.QtCore import Qt
import functools

from .joint_tiles_widget import JointTile

class TelemetryDashboard(QWidget):
    def __init__(self, store):
        super().__init__()
        self.store = store
        self.tiles = [JointTile(i+1) for i in range(6)]
        self.init_ui()

    def init_ui(self):
        # Haupt-Layout für das gesamte Widget
        self.main_layout = QVBoxLayout(self)
        
        # --- 1. Navigationsleiste (Bleibt oben fixiert) ---
        self.nav_bar = QHBoxLayout()
        self.btn_grid = QPushButton("All Axis")
        self.btn_grid.setStyleSheet("font-weight: bold; background-color: #34495e; color: white;")
        self.btn_grid.clicked.connect(self.show_grid)
        self.nav_bar.addWidget(self.btn_grid)
        
        for i in range(6):
            btn = QPushButton(f"J{i+1}")
            btn.clicked.connect(functools.partial(self.show_focus, i))
            self.nav_bar.addWidget(btn)
        
        self.main_layout.addLayout(self.nav_bar)

        # --- 2. Die Scroll Area erstellen ---
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)  # Inhalt passt sich der Breite an
        self.scroll_area.setFrameShape(QFrame.Shape.NoFrame) # Entfernt unschöne Rahmen
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        # Optional: Wenn du das Scrollen fast ganz unterbinden willst, solange es geht:
        self.scroll_area.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        
        # --- 3. Das Stacked Widget als Inhalt für die Scroll Area ---
        # Wir brauchen ein Container-Widget, das den Stack hält
        self.container = QWidget()
        self.container_layout = QVBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)

        self.display_stack = QStackedWidget()
        
        # Ansicht A: Das 6er Grid
        self.grid_container = QWidget()
        self.grid_layout = QGridLayout(self.grid_container)
        self.grid_layout.setColumnStretch(0, 1)
        self.grid_layout.setColumnStretch(1, 1)
        self.grid_layout.setRowStretch(0, 1)
        self.grid_layout.setRowStretch(1, 1)
        self.grid_layout.setRowStretch(2, 1)
        
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
        
        # Initialisierung
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
        self.display_stack.setCurrentIndex(0)
        self.clear_layout(self.grid_layout)
        for i, tile in enumerate(self.tiles):
            tile.show()
            self.grid_layout.addWidget(tile, i // 2, i % 2)

    def show_focus(self, index):
        self.display_stack.setCurrentIndex(1)
        self.clear_layout(self.focus_layout)
        target_tile = self.tiles[index]
        target_tile.show()
        self.focus_layout.addWidget(target_tile)

    def update_all(self):
        frames = self.store.get_plot_data()
        if not frames:
            return
        for tile in self.tiles:
            tile.update_plots(frames)