from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QSlider, QPushButton, QFrame)
from PyQt6.QtCore import pyqtSignal, Qt

class ManualControlWidget(QWidget):
    request_move = pyqtSignal(list)

    def __init__(self, store, parent=None,):
        super().__init__(parent)
        self.store = store
        self.limits = [(-180.0, 180.0)] * 6 
        self.sliders = []
        self.target_labels = [] # Anzeige für Slider-Stellung
        self.actual_labels = [] # Anzeige für echte Roboter-Position
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        header = QLabel("<b>Manual Axis Control</b>")
        header.setStyleSheet("font-size: 16px; margin-bottom: 5px;")
        layout.addWidget(header)

        self.scroll_area = QFrame()
        self.scroll_area.setFrameShape(QFrame.Shape.StyledPanel)
        scroll_layout = QVBoxLayout(self.scroll_area)

        for i in range(6):
            axis_layout = QVBoxLayout()
            
            # --- ZEILE 1: Achsen-Name und IST-WERT ---
            label_row = QHBoxLayout()
            name_label = QLabel(f"Axis {i+1}")
            name_label.setStyleSheet("font-weight: bold; color: #ecf0f1;")
            
            # Neue Anzeige für die echte Position vom Roboter
            actual_display = QLabel("ACT: 0.00°")
            actual_display.setStyleSheet("font-family: monospace; color: #2ecc71; font-weight: bold;")
            
            label_row.addWidget(name_label)
            label_row.addStretch()
            label_row.addWidget(actual_display)
            axis_layout.addLayout(label_row)

            # --- ZEILE 2: SLIDER und SOLL-WERT ---
            slider_row = QHBoxLayout()
            min_val, max_val = self.limits[i]
            
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(min_val * 100), int(max_val * 100))
            slider.setValue(0)
            
            # Anzeige für den Wert, den man gerade schiebt
            target_display = QLabel("SET: 0.00°")
            target_display.setFixedWidth(80)
            target_display.setStyleSheet("font-family: monospace; color: #3498db;")
            
            slider.valueChanged.connect(lambda val, td=target_display: td.setText(f"SET: {val/100:.2f}°"))
            
            slider_row.addWidget(slider)
            slider_row.addWidget(target_display)
            
            axis_layout.addLayout(slider_row)
            scroll_layout.addLayout(axis_layout)
            
            self.sliders.append(slider)
            self.actual_labels.append(actual_display)
            self.target_labels.append(target_display)

        layout.addWidget(self.scroll_area)

        # --- Buttons ---
        btn_layout = QVBoxLayout() # Vertikal für mehr Platz
        
        self.btn_write = QPushButton("WRITE TO ROBOT")
        self.btn_write.setFixedHeight(45)
        self.btn_write.setStyleSheet("""
            QPushButton { background-color: #e67e22; color: white; font-weight: bold; border-radius: 4px; }
            QPushButton:pressed { background-color: #d35400; }
        """)
        self.btn_write.clicked.connect(self.emit_all_moves)
        
        # NEU: "Snap to Actual" statt einfach nur "Reset"
        self.btn_sync = QPushButton("Sync Sliders to Robot Position")
        self.btn_sync.setFixedHeight(35)
        self.btn_sync.setStyleSheet("background-color: #34495e; color: white;")
        self.btn_sync.clicked.connect(self.sync_sliders_to_actual)

        btn_layout.addWidget(self.btn_write)
        btn_layout.addWidget(self.btn_sync)
        layout.addLayout(btn_layout)
        
        layout.addStretch()

    def update_actual_positions(self):
        """
        Wird vom Main-Timer aufgerufen.
        Übernimmt eine Liste [p1, p2, p3, p4, p5, p6]
        """
        positions = self.store.get_current_positions()
        self.current_actual_values = positions # Intern speichern für Sync
        for i, val in enumerate(positions):
            self.actual_labels[i].setText(f"ACT: {val:.2f}°")

    def sync_sliders_to_actual(self):
        """Setzt die Slider genau dahin, wo der Roboter gerade wirklich steht."""
        if hasattr(self, 'current_actual_values'):
            for i, val in enumerate(self.current_actual_values):
                # Signale kurz blockieren, damit nicht versehentlich Befehle gefeuert werden
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(val * 100))
                self.target_labels[i].setText(f"SET: {val:.2f}°")
                self.sliders[i].blockSignals(False)

    def emit_all_moves(self):
        current_positions = [s.value() / 100.0 for s in self.sliders]
        self.request_move.emit(current_positions)