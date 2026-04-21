from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QStackedWidget, QGridLayout)
import functools # Sicherere Methode für Lambda-Loops

from .joint_tiles import JointTile

class TelemetryDashboard(QWidget):
    def __init__(self, store):
        super().__init__()
        self.store = store
        # Wir erstellen die Tiles EINMAL und behalten sie im Speicher
        self.tiles = [JointTile(i+1, store) for i in range(6)]
        self.init_ui()

    def init_ui(self):
        # FEHLER FIX: self.layout -> self.main_layout (Vermeidet Konflikt mit QWidget.layout())
        self.main_layout = QVBoxLayout(self)
        
        # --- Navigationsleiste (intern im Widget) ---
        self.nav_bar = QHBoxLayout()
        self.btn_grid = QPushButton("Ganzes Grid")
        self.btn_grid.setStyleSheet("font-weight: bold; background-color: #34495e; color: white;")
        self.btn_grid.clicked.connect(self.show_grid)
        self.nav_bar.addWidget(self.btn_grid)
        
        # Buttons für Einzel-Fokus J1 bis J6
        for i in range(6):
            btn = QPushButton(f"J{i+1}")
            # Fix für Lambda in Loops: functools.partial verhindert, dass alle Buttons auf J6 zeigen
            btn.clicked.connect(functools.partial(self.show_focus, i))
            self.nav_bar.addWidget(btn)
        
        self.main_layout.addLayout(self.nav_bar)

        # --- Stacked Widget für den Inhaltswechsel ---
        self.display_stack = QStackedWidget()
        
        # Ansicht A: Das 6er Grid
        self.grid_container = QWidget()
        self.grid_layout = QGridLayout(self.grid_container)
        # Tiles initial ins Grid setzen
        self.show_grid() 
        
        # Ansicht B: Der Fokus-Bereich (Einzelelement groß)
        self.focus_container = QWidget()
        self.focus_layout = QVBoxLayout(self.focus_container)
        self.focus_layout.setContentsMargins(0, 0, 0, 0)
        
        self.display_stack.addWidget(self.grid_container) # Index 0
        self.display_stack.addWidget(self.focus_container) # Index 1
        
        self.main_layout.addWidget(self.display_stack)

    def clear_layout(self, layout):
        """Hilfsfunktion: Entfernt Widgets aus einem Layout, ohne sie zu löschen."""
        if layout is not None:
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.setParent(None)

    def show_grid(self):
        """Zeigt alle Achsen im Mini-Modus an."""
        self.display_stack.setCurrentIndex(0)
        self.clear_layout(self.grid_layout) # Sicherstellen, dass das Grid leer ist
        
        for i, tile in enumerate(self.tiles):
            self.grid_layout.addWidget(tile, i // 2, i % 2)

    def show_focus(self, index):
        """Zeigt eine einzelne Achse im Detail-Modus an."""
        self.display_stack.setCurrentIndex(1)
        self.clear_layout(self.focus_layout) # Fokus-Bereich leeren
        
        target_tile = self.tiles[index]
        self.focus_layout.addWidget(target_tile)

    def update_all(self):
        """Wird vom MainWindow-Timer aufgerufen."""
        # Wir müssen immer alle updaten, damit die Kurven auch im Hintergrund weiterlaufen
        for tile in self.tiles:
            tile.update_plots()