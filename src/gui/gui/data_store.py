from collections import deque
import threading

class GuiDataStore:
    def __init__(self, maxlen=4000):
        self._lock = threading.Lock()
        
        # 1. Hochfrequente Daten (für die Plots)
        self.frames = deque(maxlen=maxlen)
        
        # 2. Aktuelle Status-Werte (für Labels/Anzeigen)
        self.connected = False
        self.armed = False
        self.current_mode = "UNKNOWN"
        self.current_latency = 0.0
        self.last_update_time = 0
        self.trajectory_prog = 0
        
        # 3. Konfigurations-Daten (für das Parameter-Widget)
        self.axis_parameters = {i: {"p_gain": i, "i_gain": i, "d_gain": i, "limit_v": i} for i in range(1, 7)}

    def push_telemetry_frame(self, time_s, actual_list, target_list, traj_prog):
        """Wird 1000x pro Sekunde aufgerufen - NUR PLOTDATEN."""
        self.trajectory_prog = traj_prog
        frame = {
            'time': time_s,
            'joint_state_act': actual_list, # Liste von 6x {'p','v','t'}
            'joint_state_ref': target_list
        }
        with self._lock:
            self.frames.append(frame)

    def set_status(self, connected, armed, mode, latency):
        """Wird aufgerufen, wenn neue Status-Infos kommen."""
        with self._lock:
            self.connected = connected
            self.armed = armed
            self.current_mode = mode
            self.current_latency = latency

    def update_axis_config(self, axis_id, params):
        """Wird aufgerufen, wenn der Roboter Parameter-Änderungen bestätigt."""
        with self._lock:
            self.axis_parameters[axis_id].update(params)

    # --- Getter für die GUI ---
    def get_plot_data(self, count=None):
        with self._lock:
            return list(self.frames)[-count:] if count else list(self.frames)

    def get_status(self):
        """Gibt alles zurück, was ein Label oder Statusbar braucht."""
        with self._lock:
            return {
                "connected": self.connected,
                "armed": self.armed,
                "mode": self.current_mode,
                "latency": self.current_latency,
            }
    
    def get_params(self):
        """Gibt alles zurück, was ein Label oder Statusbar braucht."""
        with self._lock:
            return {
                "params": self.axis_parameters
            }
        
    def get_progress(self):
        with self._lock:
            return self.trajectory_prog
    
    def get_current_positions(self):
        """Gibt eine Liste der 6 aktuellen Ist-Positionen zurück [p1, p2, ..., p6]."""
        with self._lock:
            # Falls noch gar keine Daten da sind, geben wir Nullen zurück
            if not self.frames:
                return [0.0] * 6
            
            # Wir nehmen den aktuellsten Frame (das Ende der Deque)
            last_frame = self.frames[-1]
            
            # Wir extrahieren nur das 'p' aus jedem Eintrag der 'joint_state_act' Liste
            return [joint['p'] for joint in last_frame['joint_state_act']]