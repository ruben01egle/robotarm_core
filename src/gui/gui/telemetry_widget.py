from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QScrollArea, QSizePolicy,
                             QPushButton, QStackedWidget, QGridLayout, QFrame, QLabel, QButtonGroup)
from PyQt6.QtCore import Qt
import functools
import math

from .joint_tiles_widget import AxisTile

# Anzeige in mm bzw. Grad (Daten im Store bleiben SI), max. 0.1 mm / 0.1° Auflösung.
# min_range: kleinster Y-Bereich, damit Rauschen/Drift nicht formatfüllend aufgezoomt wird
LINEAR_DISPLAY = {
    "pos": {"unit": "mm", "scale": 1000.0, "decimals": 1, "min_range": 2.0},
    "speed": {"unit": "mm/s", "scale": 1000.0, "decimals": 1, "min_range": 2.0},
}
ANGULAR_DISPLAY = {
    "pos": {"unit": "deg", "scale": 180.0 / math.pi, "decimals": 1, "min_range": 1.0},
    "speed": {"unit": "deg/s", "scale": 180.0 / math.pi, "decimals": 1, "min_range": 2.0},
}
JOINT_DISPLAY = {**ANGULAR_DISPLAY, "torque": {"unit": "Nm"}}

CARTESIAN_AXES = [
    ("X", LINEAR_DISPLAY),
    ("Y", LINEAR_DISPLAY),
    ("Z", LINEAR_DISPLAY),
    ("Rx", ANGULAR_DISPLAY),
    ("Ry", ANGULAR_DISPLAY),
    ("Rz", ANGULAR_DISPLAY),
]

MODE_BTN_STYLE = """
    QPushButton { font-weight: bold; }
    QPushButton:checked { background-color: #2980b9; color: white; }
"""

class TelemetryDashboard(QWidget):
    def __init__(self, store):
        super().__init__()
        self.store = store

        # Interner Zustand: zwei Kachel-Sätze, angezeigt wird self.tiles
        self.mode = "joint"
        self.joint_tiles = [] # Wird erst aufgebaut, sobald die URDF geladen ist
        self.cart_tiles = [
            AxisTile(i, f"TCP {name}", frame_key='axes', channels=("speed", "pos"), display=display)
            for i, (name, display) in enumerate(CARTESIAN_AXES)
        ]
        self.tiles = self.joint_tiles
        self.nav_buttons = []
        self.num_grid_rows = 0

        # Haupt-Layout für das gesamte Widget
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)

        # --- 1. Navigationsleiste (Bleibt oben fixiert) ---
        self.nav_bar = QHBoxLayout()

        # Umschalter Gelenk-/kartesischer Raum
        self.btn_mode_joint = QPushButton("Joint")
        self.btn_mode_cart = QPushButton("Cartesian")
        self.mode_group = QButtonGroup(self)
        for btn, mode in [(self.btn_mode_joint, "joint"), (self.btn_mode_cart, "cartesian")]:
            btn.setCheckable(True)
            btn.setStyleSheet(MODE_BTN_STYLE)
            btn.clicked.connect(functools.partial(self.set_mode, mode))
            self.mode_group.addButton(btn)
            self.nav_bar.addWidget(btn)
        self.btn_mode_joint.setChecked(True)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.VLine)
        self.nav_bar.addWidget(separator)

        self.btn_grid = QPushButton("All Axis")
        self.btn_grid.setStyleSheet("font-weight: bold; background-color: #34495e; color: white;")
        self.btn_grid.clicked.connect(self.show_grid)
        self.nav_bar.addWidget(self.btn_grid)

        # Achsen-Buttons (J1..Jn bzw. X..Rz) werden je nach Modus neu befüllt
        self.axis_btn_layout = QHBoxLayout()
        self.nav_bar.addLayout(self.axis_btn_layout)

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

        # Ansicht A: Das dynamische Grid (immer 2 Spalten)
        self.grid_container = QWidget()
        self.grid_layout = QGridLayout(self.grid_container)
        self.grid_layout.setColumnStretch(0, 1)
        self.grid_layout.setColumnStretch(1, 1)

        # Ansicht B: Der Fokus-Bereich
        self.focus_container = QWidget()
        self.focus_layout = QVBoxLayout(self.focus_container)
        self.focus_layout.setContentsMargins(0, 0, 0, 0)

        # Ansicht C: Platzhalter, solange keine Daten/Achsen da sind
        self.placeholder_label = QLabel()
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder_label.setStyleSheet("color: #7f8c8d; font-size: 14px;")

        self.display_stack.addWidget(self.grid_container) # Index 0
        self.display_stack.addWidget(self.focus_container) # Index 1
        self.display_stack.addWidget(self.placeholder_label) # Index 2

        # Stack in den Container, Container in die ScrollArea
        self.container_layout.addWidget(self.display_stack)
        self.scroll_area.setWidget(self.container)

        # ScrollArea zum Haupt-Layout hinzufügen
        self.main_layout.addWidget(self.scroll_area)

        # Initialisierung der ersten Ansicht
        self.set_mode("joint")

    def set_axis_limits(self, axis_data: list):
        """
        Wird aus update_all aufgerufen, sobald die URDF geladen ist.
        Baut die Gelenk-Kacheln dynamisch für X Achsen auf.
        """
        if not axis_data or self.joint_tiles:
            return # Verhindert doppelte Initialisierung

        # Kacheln dynamisch erzeugen (Titel startet bei 1)
        self.joint_tiles.extend(
            AxisTile(i, f"JOINT {i + 1}", display=JOINT_DISPLAY) for i in range(len(axis_data))
        )

        if self.mode == "joint":
            self.set_mode("joint")

    def set_mode(self, mode):
        self.mode = mode
        self.tiles = self.joint_tiles if mode == "joint" else self.cart_tiles

        # Achsen-Buttons neu aufbauen
        self.clear_layout(self.axis_btn_layout)
        self.nav_buttons = []
        for i, tile in enumerate(self.tiles):
            label = f"J{i + 1}" if mode == "joint" else CARTESIAN_AXES[i][0]
            btn = QPushButton(label)
            btn.clicked.connect(functools.partial(self.show_focus, i))
            self.axis_btn_layout.addWidget(btn)
            self.nav_buttons.append(btn)

        # Zeilen-Stretch an die neue Anzahl anpassen
        for r in range(self.num_grid_rows):
            self.grid_layout.setRowStretch(r, 0)
        self.num_grid_rows = (len(self.tiles) + 1) // 2
        for r in range(self.num_grid_rows):
            self.grid_layout.setRowStretch(r, 1)

        self.show_grid()

    def clear_layout(self, layout):
        if layout is not None:
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.setParent(None)
                    widget.hide()

    def show_placeholder(self):
        if self.mode == "joint":
            text = "Waiting for URDF joint limits to initialize telemetry..."
        else:
            base = self.store.get_base_frame() or "base"
            text = f"Waiting for TF {base} → tcp..."
        self.placeholder_label.setText(text)
        self.display_stack.setCurrentIndex(2)

    def has_data(self):
        if self.mode == "joint":
            return bool(self.joint_tiles)
        return self.store.get_current_tcp_pose() is not None

    def show_grid(self):
        self.clear_layout(self.grid_layout)
        self.clear_layout(self.focus_layout)
        if not self.has_data():
            self.show_placeholder()
            return
        self.display_stack.setCurrentIndex(0)
        for i, tile in enumerate(self.tiles):
            tile.show()
            self.grid_layout.addWidget(tile, i // 2, i % 2)

    def show_focus(self, index):
        if not self.has_data():
            return
        self.display_stack.setCurrentIndex(1)
        self.clear_layout(self.grid_layout)
        self.clear_layout(self.focus_layout)
        target_tile = self.tiles[index]
        target_tile.show()
        self.focus_layout.addWidget(target_tile)

    def update_all(self):
        # Gelenk-Kacheln aufbauen, sobald die URDF da ist
        if not self.joint_tiles:
            self.set_axis_limits(self.store.get_joint_limits())

        # Platzhalter verlassen, sobald Daten da sind
        if self.display_stack.currentIndex() == 2:
            if not self.has_data():
                return
            self.show_grid()

        if self.mode == "joint":
            frames = self.store.get_plot_data()
        else:
            frames = self.store.get_cartesian_plot_data()
        if not frames:
            return
        # Nur die aktiven Kacheln plotten, die anderen sind ausgeblendet
        for tile in self.tiles:
            tile.update_plots(frames)
