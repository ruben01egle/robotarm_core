from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QSlider, QPushButton, QDoubleSpinBox, QFrame)
from PyQt6.QtCore import pyqtSignal, Qt

class ManualControlWidget(QWidget):
    # Signal: sendet (Achsen_ID, Zielposition)
    request_move = pyqtSignal(int, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        # Limits für die Achsen (Min, Max, Standard-Schrittweite)
        self.limits = [(-180.0, 180.0)] * 6 
        self.sliders = []
        self.value_labels = []
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # Titel
        header = QLabel("<b>Manueller Achs-Vorschub</b>")
        header.setStyleSheet("font-size: 16px; margin-bottom: 5px;")
        layout.addWidget(header)

        # Bereich für die 6 Achsen
        self.scroll_area = QFrame()
        self.scroll_area.setFrameShape(QFrame.Shape.StyledPanel)
        scroll_layout = QVBoxLayout(self.scroll_area)

        for i in range(6):
            axis_layout = QVBoxLayout()
            
            # Label-Zeile (Name und Wert)
            label_row = QHBoxLayout()
            name_label = QLabel(f"Achse {i+1}")
            name_label.setStyleSheet("font-weight: bold;")
            val_display = QLabel("0.00°")
            val_display.setStyleSheet("font-family: monospace; color: #3498db;")
            
            label_row.addWidget(name_label)
            label_row.addStretch()
            label_row.addWidget(val_display)
            axis_layout.addLayout(label_row)

            # Slider Zeile
            slider_row = QHBoxLayout()
            min_val, max_val = self.limits[i]
            
            # QSlider arbeitet nur mit Integern, daher nutzen wir Faktor 100 für 2 Nachkommastellen
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(min_val * 100), int(max_val * 100))
            slider.setValue(0)
            slider.setEnabled(False) # Erstmal deaktiviert
            
            # Update Funktion für die Anzeige
            slider.valueChanged.connect(lambda val, l=val_display: l.setText(f"{val/100:.2f}°"))
            # Signal senden wenn losgelassen (oder während des Schiebens)
            slider.sliderReleased.connect(lambda idx=i: self.emit_move(idx))

            slider_row.addWidget(QLabel(f"{min_val}"))
            slider_row.addWidget(slider)
            slider_row.addWidget(QLabel(f"{max_val}"))
            
            axis_layout.addLayout(slider_row)
            scroll_layout.addLayout(axis_layout)
            
            self.sliders.append(slider)
            self.value_labels.append(val_display)

        layout.addWidget(self.scroll_area)

        # Unterer Bereich: Enable & Reset
        btn_layout = QHBoxLayout()
        
        self.btn_enable = QPushButton("MANUAL MODE AKTIVIEREN")
        self.btn_enable.setCheckable(True)
        self.btn_enable.setFixedHeight(40)
        self.btn_enable.setStyleSheet("""
            QPushButton { background-color: #7f8c8d; color: white; font-weight: bold; }
            QPushButton:checked { background-color: #e67e22; }
        """)
        self.btn_enable.toggled.connect(self.toggle_manual_mode)
        
        self.btn_reset = QPushButton("Alle auf 0°")
        self.btn_reset.setFixedHeight(40)
        self.btn_reset.clicked.connect(self.reset_all)

        btn_layout.addWidget(self.btn_enable)
        btn_layout.addWidget(self.btn_reset)
        layout.addLayout(btn_layout)
        
        layout.addStretch()

    def toggle_manual_mode(self, enabled):
        """Schaltet die Slider frei oder sperrt sie."""
        for s in self.sliders:
            s.setEnabled(enabled)
        
        if enabled:
            self.btn_enable.setText("MANUAL MODE: EIN")
        else:
            self.btn_enable.setText("MANUAL MODE AKTIVIEREN")

    def emit_move(self, axis_idx):
        """Sendet die aktuelle Position des Sliders als Signal."""
        value = self.sliders[axis_idx].value() / 100.0
        self.request_move.emit(axis_idx + 1, value)

    def reset_all(self):
        """Setzt alle Slider auf 0."""
        for s in self.sliders:
            s.setValue(0)
        # Signale feuern für alle Achsen
        for i in range(6):
            self.emit_move(i)