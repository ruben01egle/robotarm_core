from collections import deque
import threading
import bisect

class GuiDataStore:
    def __init__(self, maxlen=4000):
        self._lock = threading.Lock()
        self._maxlen = maxlen
        
        # 1. Hochfrequente Telemetrie-Daten (nur noch IST-Werte für Plots & Live-Abfragen)
        # Jeder Frame enthält: 'time' und 'joint_state_act' (Liste aus {'p':..., 'v':..., 't':...})
        self.frames = deque(maxlen=maxlen)
        
        # 2. Aktuelle Status-Werte der ROS FSM
        self.connected = False
        self.armed = False
        self.state = "UNKNOWN"

        # 3. Log-Nachrichten aus der ROS-Welt
        self.log_queue = deque(maxlen=50)

    def clear_store(self):
        with self._lock:
            self.connected = False
            self.armed = False
            self.state = "UNKNOWN"
            self.frames.clear()
            self.log_queue.clear()

    def push_telemetry_frame(self, time_s: float, actual_list: list):
        """
        Wird von der ROS-Node aufgerufen.
        actual_list ist eine Liste von Dicts für jede Achse:
        [{'p': pos, 'v': vel, 't': torq}, ...]
        """
        frame = {
            'time': time_s,
            'joint_state': actual_list
        }
        with self._lock:
            self.frames.append(frame)

    def set_status(self, state, connected, armed):
        with self._lock:
            self.state = state
            self.connected = connected
            self.armed = armed

    def add_log(self, level, name, text):
        """Wird von der ROS-Node aufgerufen, um Logs zu puffern."""
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_queue.append(f"[{ts}] {level} [{name}]: {text}")

    def get_new_logs(self):
        """Wird von der GUI aufgerufen. Holt Logs ab und leert die Queue."""
        with self._lock:
            logs = list(self.log_queue)
            self.log_queue.clear()
        return logs

    # --- Getter für GUI (Plots) ---
    def get_plot_data(self, seconds=4.0):
        with self._lock:
            if not self.frames:
                return []

            last_time = self.frames[-1]['time']
            cutoff_time = last_time - seconds

            times = [f['time'] for f in self.frames]
            start_idx = bisect.bisect_right(times, cutoff_time)

            actual_count = len(self.frames) - start_idx
            if actual_count > self._maxlen:
                start_idx = len(self.frames) - self._maxlen

            return list(self.frames)[start_idx:]
        
    def get_state(self):
        with self._lock:
            return self.state

    def get_status(self):
        with self._lock:
            return {
                "connected": self.connected,
                "armed": self.armed,
                "state": self.state,
            }
    
    # --- Live-Getter für dein ManualControlWidget ---
    def get_current_positions(self) -> list:
        """Gibt die reinen aktuellen IST-Positionen aller Achsen in Grad zurück."""
        with self._lock:
            if not self.frames:
                return [0.0] * 6 # Fallback, falls noch keine ROS-Daten da sind
            
            last_frame = self.frames[-1]
            return [joint['p'] for joint in last_frame['joint_state']]

    def get_current_velocities(self) -> list:
        """Gibt die aktuellen IST-Geschwindigkeiten aller Achsen zurück."""
        with self._lock:
            if not self.frames:
                return [0.0] * 6
            
            last_frame = self.frames[-1]
            return [joint['v'] for joint in last_frame['joint_state']]

    def get_current_torques(self) -> list:
        """Gibt die aktuellen IST-Drehmomente (Effort) aller Achsen in Nm zurück."""
        with self._lock:
            if not self.frames:
                return [0.0] * 6
            
            last_frame = self.frames[-1]
            return [joint['t'] for joint in last_frame['joint_state']]