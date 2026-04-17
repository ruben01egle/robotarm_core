from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QDoubleSpinBox, QPushButton, QComboBox, QGridLayout)
from PyQt6.QtCore import Qt, pyqtSignal

class RobotParameterConfig(QWidget):
    request_param_update = pyqtSignal(int, str, float)
    request_param_read = pyqtSignal(int) # Signal, um Werte vom Roboter anzufordern

    def __init__(self, parent=None):
        super().__init__(parent)
        self.param_definitions = {
            "p_gain": ("P-Gain", 0.0, 500.0, 40.0),
            "i_gain": ("I-Gain", 0.0, 100.0, 0.0),
            "d_gain": ("D-Gain", 0.0, 10.0, 1.0),
            "limit_v": ("Max Velocity", 0.0, 50.0, 5.0),
        }
        self.inputs = {} 
        self.actual_labels = {} # Für das Feedback
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(20) # Abstand zwischen den Blöcken
        main_layout.setContentsMargins(30, 30, 30, 30) # Rand außen

        # 1. Kopfzeile: Achsen-Wahl (schön zentriert/oben)
        top_wrapper = QWidget()
        top_layout = QHBoxLayout(top_wrapper)
        
        self.axis_selector = QComboBox()
        self.axis_selector.addItems([f"Gelenk {i+1}" for i in range(6)])
        self.axis_selector.setFixedWidth(150)
        # NEU: Triggert Read beim Umschalten
        self.axis_selector.currentIndexChanged.connect(self.on_axis_changed)
        
        top_layout.addStretch() # Schiebt alles zur Mitte
        top_layout.addWidget(QLabel("<b>Aktive Achse:</b>"))
        top_layout.addWidget(self.axis_selector)
        top_layout.addStretch() # Schiebt alles zur Mitte
        
        main_layout.addWidget(top_wrapper)

        # 2. Parameter-Bereich (Grid)
        grid_container = QWidget()
        grid = QGridLayout(grid_container)
        grid.setHorizontalSpacing(40)
        grid.setVerticalSpacing(15)

        # Header für das Grid
        grid.addWidget(QLabel("<b>Parameter</b>"), 0, 0)
        grid.addWidget(QLabel("<b>Soll-Wert (Edit)</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Ist-Wert (Motor)</b>"), 0, 2)

        for i, (key, (label_text, min_v, max_v, default)) in enumerate(self.param_definitions.items(), 1):
            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setValue(default)
            spin.setFixedWidth(120)
            
            actual_val_label = QLabel("---") 
            actual_val_label.setStyleSheet("color: gray; font-family: monospace; font-size: 14px;")
            
            grid.addWidget(QLabel(label_text), i, 0)
            grid.addWidget(spin, i, 1)
            grid.addWidget(actual_val_label, i, 2, Qt.AlignmentFlag.AlignCenter)
            
            self.inputs[key] = spin
            self.actual_labels[key] = actual_val_label
            
        main_layout.addWidget(grid_container, alignment=Qt.AlignmentFlag.AlignCenter)

        # 3. Apply Button
        self.btn_apply = QPushButton("Konfiguration an Motor übertragen")
        self.btn_apply.setFixedWidth(300)
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
        
        main_layout.addWidget(self.btn_apply, alignment=Qt.AlignmentFlag.AlignCenter)
        main_layout.addStretch() # Schiebt alles nach oben

    def on_axis_changed(self):
        """Wird aufgerufen, wenn im Dropdown eine andere Achse gewählt wird."""
        axis_id = self.axis_selector.currentIndex() + 1
        # Erstmal Labels zurücksetzen
        for label in self.actual_labels.values():
            label.setText("Warten...")
        # Read-Befehl ans Backend feuern
        self.request_param_read.emit(axis_id)

    def update_actual_values(self, param_dict):
        """Wird aufgerufen, wenn neue Daten vom STM32/Moteus kommen."""
        for key, value in param_dict.items():
            if key in self.actual_labels:
                self.actual_labels[key].setText(f"{value:.3f}")
                # Optional: Färbe den Ist-Wert grün, wenn er mit Soll übereinstimmt
                if abs(value - self.inputs[key].value()) < 0.001:
                    self.actual_labels[key].setStyleSheet("color: green;")
                else:
                    self.actual_labels[key].setStyleSheet("color: red;")

    def emit_all_params(self):
        """Liest alle Felder aus und sendet ein Signal für jeden Wert."""
        axis_id = self.axis_selector.currentIndex() + 1 # +1 weil IDs bei 1 starten
        
        for key, spinbox in self.inputs.items():
            value = spinbox.value()
            self.request_param_update.emit(axis_id, key, value)
            print(f"Sende: Achse {axis_id} -> {key} = {value}")