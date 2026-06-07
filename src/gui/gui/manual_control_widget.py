from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QFrame
from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt
import math

class ManualControlWidget(QWidget):
    # Signal sendet die Liste der Achspositionen (in Grad) im Takt von update_widget
    live_stream_move = pyqtSignal(list)
    request_movement = pyqtSignal(bool)

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        
        # Interner Zustand
        self.limits = []
        self.num_joints = 0
        self.movement_enabled = False
        
        # Sicherheits-Feature: Maximaler Sprung pro Update-Zyklus in Grad (z.B. max 5.0° pro Aufruf)
        self.max_allowed_delta = 5.0 
        
        # Listen für dynamische UI-Elemente
        self.sliders = []
        self.target_labels = []
        self.actual_labels = []
        self.last_sent_positions = []

        # Haupt-Layout vorbereiten (bleibt leer, bis set_axis_limits aufgerufen wird)
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)
        
        # Initialer Platzhalter
        self.placeholder_label = QLabel("Waiting for URDF joint limits...")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.main_layout.addWidget(self.placeholder_label)

    @pyqtSlot(list)
    def set_axis_limits(self, new_limits_rad: list):
        """
        Wird beim Startup von außen aufgerufen. 
        Erwartet eine Liste von Tuples: [(min, max), (min, max), ...] in Grad.
        Baut die GUI komplett dynamisch auf.
        """
        if not new_limits_rad:
            return
            
        self.limits = [(math.degrees(m_min), math.degrees(m_max)) for m_min, m_max in new_limits_rad]
        self.num_joints = len(new_limits_rad)
        self.last_sent_positions = [0.0] * self.num_joints

        # Platzhalter entfernen
        self.main_layout.removeWidget(self.placeholder_label)
        self.placeholder_label.deleteLater()

        # --- 1. HEADER & UNLOCK BUTTON ---
        header_layout = QHBoxLayout()
        header = QLabel("<b>Manual Axis Control</b>")
        header.setStyleSheet("font-size: 16px;")
        header_layout.addWidget(header)
        header_layout.addStretch()
        
        self.btn_toggle_lock = QPushButton("ENABLE MOVEMENT")
        self.btn_toggle_lock.setCheckable(True)
        self.btn_toggle_lock.setFixedWidth(150)
        self.btn_toggle_lock.setFixedHeight(30)
        self.btn_toggle_lock.clicked.connect(self.on_lock_toggle_clicked)
        header_layout.addWidget(self.btn_toggle_lock)
        
        self.main_layout.addLayout(header_layout)

        # --- 2. SPEED OVERRIDE SECTION ---
        self.setup_speed_slider_ui()

        # --- 3. DYNAMISCHE ACHSEN ---
        self.scroll_area = QFrame()
        self.scroll_area.setFrameShape(QFrame.Shape.StyledPanel)
        scroll_layout = QVBoxLayout(self.scroll_area)

        for i in range(self.num_joints):
            axis_layout = QVBoxLayout()
            
            # Zeile 1: Name und Echte Position (ACT)
            label_row = QHBoxLayout()
            name_label = QLabel(f"Axis {i+1}")
            name_label.setStyleSheet("font-weight: bold; color: #ecf0f1;")
            
            actual_display = QLabel("ACT: 0.00°")
            actual_display.setStyleSheet("font-family: monospace; color: #2ecc71; font-weight: bold;")
            
            label_row.addWidget(name_label)
            label_row.addStretch()
            label_row.addWidget(actual_display)
            axis_layout.addLayout(label_row)

            # Zeile 2: Slider und Soll-Position (SET)
            slider_row = QHBoxLayout()
            min_val, max_val = self.limits[i]
            
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(min_val * 100), int(max_val * 100))
            slider.setValue(0)
            
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

        self.main_layout.addWidget(self.scroll_area)
        self.main_layout.addStretch()

        # Initialen Sperr-Zustand erzwingen
        self.update_ui_lock_state()

    def setup_speed_slider_ui(self):
        speed_container = QFrame()
        speed_container.setStyleSheet("background-color: #2c3e50; border-radius: 8px; border: 1px solid #3498db;")
        speed_layout = QVBoxLayout(speed_container)
        
        speed_header = QHBoxLayout()
        speed_title = QLabel("Speed Scale")
        speed_title.setStyleSheet("font-weight: bold; color: #3498db; border: none;")
        self.speed_display = QLabel("50%")
        self.speed_display.setStyleSheet("font-weight: bold; color: #f1c40f; border: none;")
        
        speed_header.addWidget(speed_title)
        speed_header.addStretch()
        speed_header.addWidget(self.speed_display)
        speed_layout.addLayout(speed_header)
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.on_speed_slider_changed)
        
        speed_layout.addWidget(self.speed_slider)
        self.main_layout.addWidget(speed_container)

    def on_speed_slider_changed(self, val):
        pass

    def on_lock_toggle_clicked(self, checked):
        """Wird aufgerufen, wenn der Benutzer manuell auf den Freischalt-Button klickt."""
        
        # 1. Signale des Buttons kurz blockieren
        self.btn_toggle_lock.blockSignals(True)
        
        # 2. Den Button visuell auf den aktuellen FSM-Zustand zwingen.
        #    Weil Signale blockiert sind, wird HIERBEI kein neues clicked-Event ausgelöst!
        self.btn_toggle_lock.setChecked(self.movement_enabled)
        
        # 3. Blockierung sofort wieder aufheben, damit zukünftige Klicks erkannt werden
        self.btn_toggle_lock.blockSignals(False)
        
        # 4. Jetzt das Signal mit dem vom Benutzer GEWÜNSCHTEN Zustand (checked) abfeuern
        self.request_movement.emit(checked)

    def update_ui_lock_state(self):
        """Aktiviert oder deaktiviert alle Eingabeelemente basierend auf movement_enabled."""
        for slider in self.sliders:
            slider.setEnabled(self.movement_enabled)
        self.speed_slider.setEnabled(self.movement_enabled)

        if self.movement_enabled:
            self.btn_toggle_lock.setText("DISENGAGE MOVEMENT")
            self.btn_toggle_lock.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
            self.btn_toggle_lock.setChecked(True)
        else:
            self.btn_toggle_lock.setText("ENABLE MOVEMENT")
            self.btn_toggle_lock.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
            self.btn_toggle_lock.setChecked(False)

    @pyqtSlot(bool)
    def set_movement_allowed(self, allowed: bool):
        """
        Slot, der von außen aufgerufen wird, um die Slider freizuschalten oder zu sperren.
        Kann direkt mit einem PyQt-Signal der ROS-Bridge verbunden werden.
        """
        if self.movement_enabled != allowed:
            self.movement_enabled = allowed
            self.update_ui_lock_state()
            
            # Sicherheits-Feature: Beim frisch Freischalten snappen wir die Slider 
            # sofort auf den echten Roboterzustand, um Start-Sprünge zu verhindern
            if self.movement_enabled:
                self.sync_sliders_to_actual()

    def update_widget(self):
        """
        Wird zyklisch (z.B. mit 50Hz) vom übergeordneten GUI-Timer aufgerufen.
        Nimmt jetzt keine Argumente mehr entgegen.
        """
        if self.num_joints == 0:
            return

        # 1. Aktuelle IST-Werte aus dem Store holen und anzeigen
        actual_positions_rad = self.store.get_current_positions() 
        
        # DIREKTE KONVERTIERUNG: Für die Anzeige im Widget rechnen wir Radian in Grad um
        self.current_actual_values = [math.degrees(rad) for rad in actual_positions_rad]
        for i, val in enumerate(self.current_actual_values):
            if i < len(self.actual_labels):
                self.actual_labels[i].setText(f"ACT: {val:.2f}°")

        # 2. Zyklisches Streaming an den Roboter (nur wenn von FSM freigegeben)
        if self.movement_enabled:
            self.process_and_stream_sliders()

    def process_and_stream_sliders(self):
        """
        Wir nehmen die Slider-Positionen nur als 'Ziel' (Target).
        Wir senden aber nur ein Delta in Richtung dieses Ziels.
        """
        # 1. Slider-Werte als 'Wunsch-Positionen' lesen
        target_slider_positions = [s.value() / 100.0 for s in self.sliders]
        
        # 2. Wir berechnen für diesen Zyklus die nächste logische Position für den Roboter
        # ausgehend von der letzten gesendeten Position.
        new_sent_positions = []
        
        for i in range(self.num_joints):
            diff = target_slider_positions[i] - self.last_sent_positions[i]
            
            # Begrenze die Bewegung pro Zyklus auf das Maximum
            if abs(diff) > self.max_allowed_delta:
                step = math.copysign(self.max_allowed_delta, diff)
                new_pos = self.last_sent_positions[i] + step
            else:
                new_pos = target_slider_positions[i]
                
            new_sent_positions.append(new_pos)
            
            # WICHTIG: Update das Label (aber NICHT den Slider!)
            self.target_labels[i].setText(f"SET: {new_pos:.2f}°")

        # 3. Senden
        positions_in_radians = [math.radians(deg) for deg in new_sent_positions]
        self.live_stream_move.emit(positions_in_radians)
        
        # 4. Status merken
        self.last_sent_positions = new_sent_positions

    def sync_sliders_to_actual(self):
        """Setzt die Slider exakt dorthin, wo der Roboter gerade physikalisch steht."""
        if hasattr(self, 'current_actual_values'):
            for i, val in enumerate(self.current_actual_values):
                if i < len(self.sliders):
                    self.sliders[i].blockSignals(True)
                    self.sliders[i].setValue(int(val * 100))
                    self.target_labels[i].setText(f"SET: {val:.2f}°")
                    self.sliders[i].blockSignals(False)
            self.last_sent_positions = list(self.current_actual_values)

    def set_max_delta(self, max_degree: float):
        """Erlaubt das Ändern der Delta-Sicherheitsgrenze zur Laufzeit."""
        self.max_allowed_delta = max_degree