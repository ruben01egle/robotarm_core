from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QDoubleSpinBox, QPushButton, QComboBox, QGridLayout)
from PyQt6.QtCore import Qt, pyqtSignal

class RobotParameterConfig(QWidget):
    request_param_write = pyqtSignal(int, dict)
    request_param_read = pyqtSignal()

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self.param_definitions = {
            "p_gain": ("P-Gain", 0.0, 500.0, 40.0),
            "i_gain": ("I-Gain", 0.0, 100.0, 0.0),
            "d_gain": ("D-Gain", 0.0, 10.0, 1.0),
            "limit_v": ("Max Vel", 0.0, 50.0, 5.0),
        }
        self.inputs = {} 
        self.actual_labels = {} # Für das Feedback
        self.last_seen_params = {}
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(15) # Abstand zwischen den Blöcken
        main_layout.setContentsMargins(30, 30, 30, 30) # Rand außen

        # 1. Kopfzeile: Achsen-Wahl (schön zentriert/oben)
        top_wrapper = QWidget()
        top_layout = QHBoxLayout(top_wrapper)
        
        self.axis_selector = QComboBox()
        self.axis_selector.addItems([f"Axis {i+1}" for i in range(6)])
        self.axis_selector.setFixedWidth(150)
        self.axis_selector.currentIndexChanged.connect(self.clear_param_values)
        
        top_layout.addStretch() # Schiebt alles zur Mitte
        top_layout.addWidget(QLabel("<b>Active axis:</b>"))
        top_layout.addWidget(self.axis_selector)
        top_layout.addStretch() # Schiebt alles zur Mitte
        
        main_layout.addWidget(top_wrapper)

        # 2. Parameter-Bereich (Grid)
        grid_container = QWidget()
        grid = QGridLayout(grid_container)
        grid.setHorizontalSpacing(20)
        grid.setVerticalSpacing(15)

        # Header für das Grid
        grid.addWidget(QLabel("<b>Parameter</b>"), 0, 0)
        grid.addWidget(QLabel("<b>target value</b>"), 0, 1)
        grid.addWidget(QLabel("<b>current value</b>"), 0, 2)

        for i, (key, (label_text, min_v, max_v, default)) in enumerate(self.param_definitions.items(), 1):
            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setValue(default)
            spin.setFixedWidth(120)
            spin.valueChanged.connect(self.clear_param_values)
            
            actual_val_label = QLabel("---") 
            actual_val_label.setStyleSheet("color: gray; font-family: monospace; font-size: 14px;")
            
            grid.addWidget(QLabel(label_text), i, 0)
            grid.addWidget(spin, i, 1)
            grid.addWidget(actual_val_label, i, 2, Qt.AlignmentFlag.AlignCenter)
            
            self.inputs[key] = spin
            self.actual_labels[key] = actual_val_label
            
        main_layout.addWidget(grid_container, alignment=Qt.AlignmentFlag.AlignCenter)

        # 3. Apply Button
        actions_layout = QHBoxLayout()

        self.btn_apply = QPushButton("Reconfigure motor")
        self.btn_apply.setFixedHeight(40)
        self.btn_apply.setStyleSheet("""
            QPushButton {
                background-color: #2c3e50; 
                color: white; 
                border-radius: 5px; 
                font-weight: bold;
            }
            QPushButton:hover { background-color: #34495e; }
        """)
        self.btn_apply.clicked.connect(self.emit_all_params)

        # Der neue Sync-Button
        self.btn_sync = QPushButton("Sync sliders to motor")
        self.btn_sync.setFixedHeight(40)
        self.btn_sync.setStyleSheet("""
            QPushButton {
                background-color: #7f8c8d; 
                color: white; 
                border-radius: 5px; 
            }
            QPushButton:hover { background-color: #95a5a6; }
        """)
        self.btn_sync.clicked.connect(self.sync_inputs_to_motor)

        # read motor button
        self.btn_read = QPushButton("Read motor values")
        self.btn_read.setFixedHeight(40)
        self.btn_read.setStyleSheet("""
            QPushButton {
                background-color: #7f8c8d; 
                color: white; 
                border-radius: 5px; 
            }
            QPushButton:hover { background-color: #95a5a6; }
        """)
        self.btn_read.clicked.connect(lambda: self.request_param_read.emit())
        
        actions_layout.addWidget(self.btn_read)
        actions_layout.addWidget(self.btn_sync)

        main_layout.addLayout(actions_layout)
        main_layout.addWidget(self.btn_apply)
        
        main_layout.addStretch() # Schiebt alles nach oben
        

    def clear_param_values(self):
        self.last_seen_params = {}
        for label in self.actual_labels.values():
            label.setText("---")
            label.setStyleSheet("color: gray; font-family: monospace;")

    def sync_inputs_to_motor(self):
        axis_id = self.axis_selector.currentIndex() + 1
        current_params = self.last_seen_params.get(axis_id, {})

        if not current_params:
            print("No data for sync available")
            return

        for key, spinbox in self.inputs.items():
            if key in current_params:
                spinbox.blockSignals(True)
                spinbox.setValue(current_params[key])
                spinbox.blockSignals(False)

        self.clear_param_values()


    def update_widget(self):
        current_params = self.store.get_params()["params"]

        if current_params == self.last_seen_params:
            return
        
        axis_id = self.axis_selector.currentIndex() + 1
        current_axis_data = current_params.get(axis_id, {})
        
        for key, value in current_axis_data.items():
            if key in self.actual_labels:
                label = self.actual_labels[key]
                label.setText(f"{value:.3f}")
                
                # Vergleich Logik
                target_val = self.inputs[key].value()
                color = "#27ae60" if abs(value - target_val) < 0.0001 else "#e74c3c"
                label.setStyleSheet(f"color: {color}; font-weight: bold;")

        import copy
        self.last_seen_params = copy.deepcopy(current_params)


    def emit_all_params(self):
        # 1. Aktuelle Achse ermitteln
        axis_id = self.axis_selector.currentIndex() + 1
        
        # 2. Alle Werte aus den Spinboxen in ein Dictionary sammeln
        params_to_send = {}
        for key, spinbox in self.inputs.items():
            params_to_send[key] = spinbox.value()
            
        # 3. Das gesamte Paket mit einem Signal emitten
        self.request_param_write.emit(axis_id, params_to_send)
        
        # Debug-Log
        print(f"Sende Konfiguration: Achse {axis_id} -> {params_to_send}")