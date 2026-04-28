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
        self.state = "UNKNOWN"
        self.current_latency = 0.0
        self.trajectory_planning_prog = 0
        self.trajectory_executing_prog = 0
        
        # 3. Konfigurations-Daten (für das Parameter-Widget)
        self.axis_parameters = {}

        self.log_queue = deque(maxlen=50)

    def push_telemetry_frame(self, time_s, actual_list, target_list):
        frame = {
            'time': time_s,
            'joint_state_act': actual_list,
            'joint_state_ref': target_list
        }
        with self._lock:
            self.frames.append(frame)
        
    def set_latency(self, latency):
        with self._lock:
            self.current_latency = latency

    def set_status(self, state, connected, armed):
        with self._lock:
            self.state = state
            self.connected = connected
            self.armed = armed

    def update_axis_config(self, params):
        with self._lock:
            self.axis_parameters.update(params)

    def add_log(self, level, name, text):
        """Wird von der ROS-Node aufgerufen"""
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_queue.append(f"[{ts}] {level} [{name}]: {text}")

    def get_new_logs(self):
        """Wird von der GUI aufgerufen. Holt alles ab und leert die Queue lokal."""
        logs = list(self.log_queue)
        self.log_queue.clear()
        return logs

    # --- Getter for GUI ---
    def get_plot_data(self, count=None):
        with self._lock:
            return list(self.frames)[-count:] if count else list(self.frames)
        
    def get_state(self):
        with self._lock:
            return self.state

    def get_status(self):
        with self._lock:
            return {
                "connected": self.connected,
                "armed": self.armed,
                "state": self.state,
                "latency": self.current_latency,
            }
    
    def clear_params(self):
        with self._lock:
            self.axis_parameters = {}
        
    def get_params(self):
        with self._lock:
            return {
                "params": self.axis_parameters
            }
        
    def get_progress(self):
        with self._lock:
            return self.trajectory_planning_prog, self.trajectory_executing_prog
    
    def get_current_positions(self):
        with self._lock:
            if not self.frames:
                return [0.0] * 6
            
            last_frame = self.frames[-1]
            
            return [joint['p'] for joint in last_frame['joint_state_act']]
