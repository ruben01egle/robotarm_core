from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QScrollArea, QSizePolicy,
                             QPushButton, QStackedWidget, QGridLayout, QFrame, QLabel, QButtonGroup)
from PyQt6.QtCore import Qt
import functools
import math

from .joint_tiles_widget import AxisTile

# Shown in mm or degrees (the store keeps SI units), at most 0.1 mm / 0.1° resolution.
# min_range: smallest y range, so noise and drift are not zoomed to fill the plot
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

        # Internal state: two sets of tiles, self.tiles is the one shown
        self.mode = "joint"
        self.joint_tiles = [] # Built once the URDF is loaded
        self.cart_tiles = [
            AxisTile(i, f"TCP {name}", frame_key='axes', channels=("speed", "pos"), display=display)
            for i, (name, display) in enumerate(CARTESIAN_AXES)
        ]
        self.tiles = self.joint_tiles
        self.nav_buttons = []
        self.num_grid_rows = 0

        # Main layout of the whole widget
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)

        # --- 1. Navigation bar (stays fixed at the top) ---
        self.nav_bar = QHBoxLayout()

        # Toggle between joint and Cartesian space
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

        # Axis buttons (J1..Jn or X..Rz) are refilled depending on the mode
        self.axis_btn_layout = QHBoxLayout()
        self.nav_bar.addLayout(self.axis_btn_layout)

        self.main_layout.addLayout(self.nav_bar)

        # --- 2. Scroll area ---
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.scroll_area.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        # --- 3. Stacked widget as content of the scroll area ---
        self.container = QWidget()
        self.container_layout = QVBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)

        self.display_stack = QStackedWidget()

        # View A: the dynamic grid (always 2 columns)
        self.grid_container = QWidget()
        self.grid_layout = QGridLayout(self.grid_container)
        self.grid_layout.setColumnStretch(0, 1)
        self.grid_layout.setColumnStretch(1, 1)

        # View B: the focus view
        self.focus_container = QWidget()
        self.focus_layout = QVBoxLayout(self.focus_container)
        self.focus_layout.setContentsMargins(0, 0, 0, 0)

        # View C: placeholder while there is no data or no axes
        self.placeholder_label = QLabel()
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder_label.setStyleSheet("color: #7f8c8d; font-size: 14px;")

        self.display_stack.addWidget(self.grid_container) # Index 0
        self.display_stack.addWidget(self.focus_container) # Index 1
        self.display_stack.addWidget(self.placeholder_label) # Index 2

        # Stack into the container, container into the scroll area
        self.container_layout.addWidget(self.display_stack)
        self.scroll_area.setWidget(self.container)

        # Add the scroll area to the main layout
        self.main_layout.addWidget(self.scroll_area)

        # Initialise the first view
        self.set_mode("joint")

    def set_axis_limits(self, axis_data: list):
        """
        Called from update_all once the URDF is loaded.
        Builds the joint tiles dynamically for the number of axes.
        """
        if not axis_data or self.joint_tiles:
            return # prevents initialising twice

        # Create the tiles dynamically (titles start at 1)
        self.joint_tiles.extend(
            AxisTile(i, f"JOINT {i + 1}", display=JOINT_DISPLAY) for i in range(len(axis_data))
        )

        if self.mode == "joint":
            self.set_mode("joint")

    def set_mode(self, mode):
        self.mode = mode
        self.tiles = self.joint_tiles if mode == "joint" else self.cart_tiles

        # Rebuild the axis buttons
        self.clear_layout(self.axis_btn_layout)
        self.nav_buttons = []
        for i, tile in enumerate(self.tiles):
            label = f"J{i + 1}" if mode == "joint" else CARTESIAN_AXES[i][0]
            btn = QPushButton(label)
            btn.clicked.connect(functools.partial(self.show_focus, i))
            self.axis_btn_layout.addWidget(btn)
            self.nav_buttons.append(btn)

        # Adapt the row stretch to the new number of rows
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
        # Build the joint tiles once the URDF is there
        if not self.joint_tiles:
            self.set_axis_limits(self.store.get_joint_limits())

        # Leave the placeholder once there is data
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
        # Only plot the active tiles, the others are hidden
        for tile in self.tiles:
            tile.update_plots(frames)
