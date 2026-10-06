from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QSizePolicy,
                             QHBoxLayout, QPushButton, QStackedWidget)
from PyQt6.QtCore import Qt
import pyqtgraph as pg

# Kanal -> (Button/Plot-Titel, Key im Achsen-Dict)
CHANNELS = {
    "torque": ("Torque", 't'),
    "speed": ("Speed", 'v'),
    "pos": ("Position", 'p'),
}

class FixedDecimalAxis(pg.AxisItem):
    """Achse mit fester Anzahl Nachkommastellen (None = pyqtgraph-Standard)."""
    def __init__(self, decimals=None, orientation='left'):
        super().__init__(orientation)
        self.decimals = decimals
        self.enableAutoSIPrefix(False)

    def tickStrings(self, values, scale, spacing):
        if self.decimals is None:
            return super().tickStrings(values, scale, spacing)
        return [f"{v * scale:.{self.decimals}f}" for v in values]

class AxisTile(QWidget):
    """
    Telemetrie-Kachel für eine Achse (Gelenk oder kartesische Koordinate).
    frame_key: Liste im Frame, aus der gelesen wird ('joint_state' oder 'axes')
    channels:  welche Kanäle angezeigt werden, z.B. ("torque", "speed", "pos")
    display:   Darstellung je Kanal, z.B. {"pos": {"unit": "mm", "scale": 1000.0, "decimals": 1, "min_range": 2.0}}
               unit: Achsbeschriftung, scale: Faktor auf die Rohdaten, decimals: Nachkommastellen,
               min_range: kleinster erlaubter Y-Bereich (in Anzeigeeinheit), damit Rauschen nicht aufgezoomt wird
    """
    def __init__(self, index, title, frame_key='joint_state',
                 channels=("torque", "speed", "pos"), display=None):
        super().__init__()
        self.index = index
        self.title = title
        self.frame_key = frame_key
        self.channels = list(channels)
        self.display = display or {}

        self.current_view = "pos"
        self.init_ui()


    def init_ui(self):
        layout = QVBoxLayout(self)

        # --- TITEL ZEILE ---
        self.title_label = QLabel(self.title)
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("""
            background-color: #34495e;
            color: white;
            font-weight: bold;
            padding: 4px;
            border-radius: 3px;
            font-size: 13px;
        """)
        layout.addWidget(self.title_label)

        # --- Buttons ---
        btn_layout = QHBoxLayout()
        self.btn_toggle_mode = QPushButton("Switch Detail/Mini")
        self.btn_toggle_mode.clicked.connect(self.toggle_main_mode)
        btn_layout.addWidget(self.btn_toggle_mode)

        self.view_buttons = []
        for ch in self.channels:
            btn = QPushButton(CHANNELS[ch][0])
            btn.clicked.connect(lambda _, c=ch: self.set_mini_view(c))
            btn_layout.addWidget(btn)
            self.view_buttons.append(btn)
        layout.addLayout(btn_layout)

        # --- Stacked Widget ---
        self.stack = QStackedWidget()

        # 1. MINI MODUS
        self.mini_axis = FixedDecimalAxis()
        self.mini_plot = pg.PlotWidget(title="Mini View", axisItems={'left': self.mini_axis, 'bottom': self.time_axis()})
        self.mini_curve = self.mini_plot.plot(pen='y')
        self.set_mini_view(self.current_view)

        # 2. DETAIL MODUS
        self.detail_widget = QWidget()
        detail_layout = QVBoxLayout(self.detail_widget)

        # Kurven als Attribute merken, damit update_plots darauf zugreifen kann
        self.detail_curves = {}
        for ch in self.channels:
            axis = FixedDecimalAxis()
            p = pg.PlotWidget(title=CHANNELS[ch][0], axisItems={'left': axis, 'bottom': self.time_axis()})
            self.apply_display(p, axis, ch)
            self.detail_curves[ch] = p.plot(pen='y')
            p.setMinimumHeight(30)
            p.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Ignored)
            detail_layout.addWidget(p)

        self.stack.addWidget(self.mini_plot)
        self.stack.addWidget(self.detail_widget)
        layout.addWidget(self.stack)

    def toggle_main_mode(self):
        new_idx = 1 if self.stack.currentIndex() == 0 else 0
        self.stack.setCurrentIndex(new_idx)
        show_buttons = (new_idx == 0)

        for btn in self.view_buttons:
            btn.setVisible(show_buttons)

    def time_axis(self):
        axis = FixedDecimalAxis(decimals=1, orientation='bottom')
        axis.setLabel('t [s]')
        return axis

    def apply_display(self, plot, axis, ch):
        cfg = self.display.get(ch, {})
        axis.decimals = cfg.get('decimals')
        plot.setLabel('left', cfg.get('unit', ''))
        # setLimits greift auch beim Auto-Range: Bereich wird nie kleiner als min_range
        plot.getViewBox().setLimits(minYRange=cfg.get('min_range'))
        axis.picture = None
        axis.update()

    def set_mini_view(self, view):
        self.current_view = view
        self.mini_plot.setTitle(f"Mini: {CHANNELS[view][0]}")
        self.apply_display(self.mini_plot, self.mini_axis, view)

    def channel_data(self, frames, ch):
        key = CHANNELS[ch][1]
        scale = self.display.get(ch, {}).get('scale', 1.0)
        return [f[self.frame_key][self.index][key] * scale for f in frames]

    def update_plots(self, frames):
        # Zeit relativ zum neuesten Frame (-4 s ... 0 s) statt absoluter Epoch-Sekunden (~1.7e9)
        t_last = frames[-1]['time']
        times = [f['time'] - t_last for f in frames]

        if self.stack.currentIndex() == 0:  # --- MINI MODUS ---
            self.mini_curve.setData(times, self.channel_data(frames, self.current_view))

        else:  # --- DETAIL MODUS ---
            for ch, curve in self.detail_curves.items():
                curve.setData(times, self.channel_data(frames, ch))
