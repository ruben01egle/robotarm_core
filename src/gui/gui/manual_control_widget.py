from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QSlider, QPushButton, QFrame, QProgressBar)
from PyQt6.QtCore import pyqtSignal, Qt

class ManualControlWidget(QWidget):
    request_move = pyqtSignal(list, float)

    def __init__(self, store, parent=None,):
        super().__init__(parent)
        self.store = store
        # TODO: read limits from urdf file
        self.limits = [
            (-180.0, 180.0),  # Achse 1
            (-160.0, 20.0),   # Achse 2
            (-120.0, 135.0),  # Achse 3
            (-180.0, 180.0),  # Achse 4
            (-180.0, 180.0),  # Achse 5
            (-180.0, 180.0),  # Achse 6
        ]
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

        # --- SPEED OVERRIDE SECTION (NEU) ---
        speed_container = QFrame()
        speed_container.setStyleSheet("""
            QFrame {
                background-color: #2c3e50; 
                border-radius: 8px; 
                border: 1px solid #3498db;
            }
        """)
        speed_layout = QVBoxLayout(speed_container)

        speed_header = QHBoxLayout()
        speed_title = QLabel("Speed Scale")
        speed_title.setStyleSheet("font-weight: bold; color: #3498db; border: none;")

        self.speed_display = QLabel("50%") # Startwert
        self.speed_display.setStyleSheet("font-weight: bold; color: #f1c40f; border: none;")

        speed_header.addWidget(speed_title)
        speed_header.addStretch()
        speed_header.addWidget(self.speed_display)
        speed_layout.addLayout(speed_header)

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(0, 100) # 0 bis 100%
        self.speed_slider.setValue(50)
        self.speed_slider.setStyleSheet("height: 20px;")

        # Verbindung für die Anzeige
        self.speed_slider.valueChanged.connect(
            lambda val: self.speed_display.setText(f"{val}%")
        )

        speed_layout.addWidget(self.speed_slider)

        # Zuerst den Speed-Slider zum Haupt-Layout hinzufügen
        layout.addWidget(speed_container)

        # Ein kleiner Abstandshalter vor den Achsen
        layout.addSpacing(5)

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

        # 3. Progress
        self.progress_bar_planning = QProgressBar()
        layout.addWidget(QLabel("Planning Progress:"))
        layout.addWidget(self.progress_bar_planning)

        self.progress_bar_executing = QProgressBar()
        layout.addWidget(QLabel("Executing Progress:"))
        layout.addWidget(self.progress_bar_executing)
        layout.addStretch()
        
        layout.addStretch()

    def update_widget(self):
        """
        Wird vom Main-Timer aufgerufen.
        """
        positions = self.store.get_current_positions()
        self.current_actual_values = positions # Intern speichern für Sync
        for i, val in enumerate(positions):
            self.actual_labels[i].setText(f"ACT: {val:.2f}°")

        prog_planning, prog_executing = self.store.get_progress()
        self.progress_bar_planning.setValue(int(prog_planning))
        self.progress_bar_executing.setValue(int(prog_executing))

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
        current_scale = speed = self.speed_slider.value() / 100.0
        self.request_move.emit(current_positions, current_scale)