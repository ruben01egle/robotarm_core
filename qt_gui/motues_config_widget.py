from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QDoubleSpinBox, QPushButton, QComboBox, QGridLayout)
from PyQt6.QtCore import pyqtSignal

class RobotParameterConfig(QWidget):
    # Signal, das gesendet wird, wenn "Apply" gedrückt wird
    # Übergibt: (Achsen-ID, Parameter-Name, Wert)
    request_param_update = pyqtSignal(int, str, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Definition der Parameter (Leicht erweiterbar!)
        # Format: "Key": ("Anzeigename", Min, Max, Standardwert)
        self.param_definitions = {
            "p_gain": ("P-Gain", 0.0, 500.0, 40.0),
            "i_gain": ("I-Gain", 0.0, 100.0, 0.0),
            "d_gain": ("D-Gain", 0.0, 10.0, 1.0),
            "limit_v": ("Max Velocity", 0.0, 50.0, 5.0),
            "limit_t": ("Max Torque", 0.0, 10.0, 2.0),
            "placeholder_3": ("Reserve", 0.0, 1.0, 0.0)
        }
        
        self.inputs = {} # Speichert die SpinBox-Referenzen
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        
        # 1. Achsen-Auswahl (Das Menü)
        selection_layout = QHBoxLayout()
        selection_layout.addWidget(QLabel("<b>Wähle Achse:</b>"))
        self.axis_selector = QComboBox()
        self.axis_selector.addItems([f"Achse {i+1}" for i in range(6)])
        selection_layout.addWidget(self.axis_selector)
        main_layout.addLayout(selection_layout)

        # 2. Parameter-Eingabefelder (Grid)
        grid = QGridLayout()
        row = 0
        for key, (label_text, min_val, max_val, default) in self.param_definitions.items():
            label = QLabel(label_text)
            spin = QDoubleSpinBox()
            spin.setRange(min_val, max_val)
            spin.setValue(default)
            spin.setDecimals(3)
            spin.setFixedWidth(100)
            
            grid.addWidget(label, row, 0)
            grid.addWidget(spin, row, 1)
            
            self.inputs[key] = spin # Speichern für Zugriff
            row += 1
            
        main_layout.addLayout(grid)

        # 3. Apply-Button
        self.btn_apply_all = QPushButton("Alle Werte an diese Achse senden")
        self.btn_apply_all.setStyleSheet("background-color: #2c3e50; color: white; font-weight: bold; height: 30px;")
        self.btn_apply_all.clicked.connect(self.emit_all_params)
        main_layout.addWidget(self.btn_apply_all)
        
        main_layout.addStretch() # Schiebt alles nach oben

    def emit_all_params(self):
        """Liest alle Felder aus und sendet ein Signal für jeden Wert."""
        axis_id = self.axis_selector.currentIndex() + 1 # +1 weil IDs bei 1 starten
        
        for key, spinbox in self.inputs.items():
            value = spinbox.value()
            self.request_param_update.emit(axis_id, key, value)
            print(f"Sende: Achse {axis_id} -> {key} = {value}")