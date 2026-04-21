from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QSizePolicy,
                             QHBoxLayout, QPushButton, QStackedWidget)
from PyQt6.QtCore import Qt
import pyqtgraph as pg

class JointTile(QWidget):
    def __init__(self, joint_id, store):
        super().__init__()
        self.joint_id = joint_id
        self.store = store
        
        self.current_view = "pos"
        self.init_ui()
        

    def init_ui(self):
        layout = QVBoxLayout(self)

        # --- TITEL ZEILE ---
        self.title_label = QLabel(f"JOINT {self.joint_id}")
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
        
        self.btn_torque = QPushButton("Torque")
        self.btn_speed = QPushButton("Speed")
        self.btn_pos = QPushButton("Position")
        
        self.btn_torque.clicked.connect(lambda: self.set_mini_view("torque"))
        self.btn_speed.clicked.connect(lambda: self.set_mini_view("speed"))
        self.btn_pos.clicked.connect(lambda: self.set_mini_view("pos"))

        for b in [self.btn_toggle_mode, self.btn_torque, self.btn_speed, self.btn_pos]:
            btn_layout.addWidget(b)
        layout.addLayout(btn_layout)

        # --- Stacked Widget ---
        self.stack = QStackedWidget()
        
        # 1. MINI MODUS
        self.mini_plot = pg.PlotWidget(title="Mini View")
        self.mini_curve = self.mini_plot.plot(pen='y') 
        self.mini_ref_curve = self.mini_plot.plot(pen=pg.mkPen('r', style=Qt.PenStyle.DashLine))
        self.set_mini_view(self.current_view)
        
        # 2. DETAIL MODUS
        self.detail_widget = QWidget()
        detail_layout = QVBoxLayout(self.detail_widget)
        
        self.p1 = pg.PlotWidget(title="Torque")
        self.p2 = pg.PlotWidget(title="Speed")
        self.p3 = pg.PlotWidget(title="Position")

        # --- WICHTIG: KURVEN INITIALISIEREN ---
        # Wir speichern die Kurven als Attribute, damit update_plots darauf zugreifen kann
        self.curve_t_act = self.p1.plot(pen='y')
        self.curve_t_ref = self.p1.plot(pen=pg.mkPen('r', style=Qt.PenStyle.DashLine))
        
        self.curve_v_act = self.p2.plot(pen='y')
        self.curve_v_ref = self.p2.plot(pen=pg.mkPen('r', style=Qt.PenStyle.DashLine))
        
        self.curve_p_act = self.p3.plot(pen='y')
        self.curve_p_ref = self.p3.plot(pen=pg.mkPen('r', style=Qt.PenStyle.DashLine))

        for p in [self.p1, self.p2, self.p3]:
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
        
        self.btn_torque.setVisible(show_buttons)
        self.btn_speed.setVisible(show_buttons)
        self.btn_pos.setVisible(show_buttons)

    def set_mini_view(self, view):
        self.current_view = view
        self.mini_plot.setTitle(f"Mini: {view.capitalize()}")

    def update_plots(self):
        frames = self.store.get_plot_data()
        
        if not frames:
            return

        axis_key = f"axis{self.joint_id}"

        times = [f['t'] / 1e6 for f in frames]

        # 3. Mini-Modus oder Detail-Modus updaten
        if self.stack.currentIndex() == 0:  # MINI MODUS
            if self.current_view == "pos":
                actual = [getattr(f['actual'], axis_key).position for f in frames]
                target = [getattr(f['target'], axis_key).position for f in frames]
            elif self.current_view == "speed":
                actual = [getattr(f['actual'], axis_key).velocity for f in frames]
                target = [getattr(f['target'], axis_key).velocity for f in frames]
            else: # torque
                actual = [getattr(f['actual'], axis_key).torque for f in frames]
                target = [getattr(f['target'], axis_key).torque for f in frames]

            self.mini_curve.setData(times, actual)
            self.mini_ref_curve.setData(times, target)

        else:  # DETAIL MODUS
            # Hier extrahieren wir alles für diese Achse mit der richtigen getattr() Funktion
            t_data = [getattr(f['actual'], axis_key).torque for f in frames]
            v_data = [getattr(f['actual'], axis_key).velocity for f in frames]
            p_data = [getattr(f['actual'], axis_key).position for f in frames]
            
            t_ref = [getattr(f['target'], axis_key).torque for f in frames]
            v_ref = [getattr(f['target'], axis_key).velocity for f in frames]
            p_ref = [getattr(f['target'], axis_key).position for f in frames]

            self.curve_t_act.setData(times, t_data)
            self.curve_t_ref.setData(times, t_ref)
            
            self.curve_v_act.setData(times, v_data)
            self.curve_v_ref.setData(times, v_ref)
            
            self.curve_p_act.setData(times, p_data)
            self.curve_p_ref.setData(times, p_ref)