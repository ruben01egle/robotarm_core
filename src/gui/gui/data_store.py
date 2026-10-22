from collections import deque
import threading
import bisect
import math

class GuiDataStore:
    def __init__(self, maxlen=4000):
        self._lock = threading.Lock()
        self._maxlen = maxlen
        
        # 1. Hochfrequente Telemetrie-Daten (nur noch IST-Werte für Plots & Live-Abfragen)
        # Jeder Frame enthält: 'time' und 'joint_state_act' (Liste aus {'p':..., 'v':..., 't':...})
        self.frames = deque(maxlen=maxlen)
        # TCP-Pose aus TF (Root-Link der URDF -> tcp): 'time' und 'axes' (Liste aus {'p':..., 'v':...} für X, Y, Z, Rx, Ry, Rz)
        self.cart_frames = deque(maxlen=maxlen)
        
        # 2. Aktuelle Status-Werte der ROS FSM
        self.connected = False
        self.armed = False
        self.state = "UNKNOWN"
        self.stopped = False
        self.active_nodes = ""
        self.active_controllers = ""

        self.joint_limits = list()
        # Root-Link der URDF (Basis-Frame für die TCP-Pose), None bis die URDF gelesen wurde
        self.base_frame = None
        # Je Jog-Controller-Instanz: name -> (max_linear_velocity [m/s], max_angular_velocity [rad/s])
        self.cartesian_limits = {}
        # Namen der im controller_manager geladenen Controller, None bis zur ersten Abfrage
        self.loaded_controllers = None

        # Steifigkeit des cartesian_impedance_controller, wie vom Controller verwendet ("committed").
        # dict mit 'frame', 'linear', 'angular', 'max_linear', 'max_angular' oder None bis gelesen
        self.stiffness = None
        # Zähler, damit die GUI Änderungen (Lesen, /parameter_events, Apply) erkennt
        self.stiffness_version = 0
        # Ergebnis des letzten Apply: (successful, reason) oder None, wird von der GUI abgeholt
        self.stiffness_apply_result = None

        # 3. Log-Nachrichten aus der ROS-Welt
        self.log_queue = deque(maxlen=50)

    def clear_store(self):
        with self._lock:
            self.connected = False
            self.armed = False
            self.state = "UNKNOWN"
            self.stopped = False
            self.active_nodes = ""
            self.active_controllers = ""
            self.joint_limits = list()
            self.base_frame = None
            self.cartesian_limits = {}
            self.loaded_controllers = None
            self.stiffness = None
            self.stiffness_version += 1
            self.stiffness_apply_result = None
            self.frames.clear()
            self.cart_frames.clear()
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

    def push_cartesian_frame(self, time_s: float, pose: list):
        """
        Wird von der ROS-Node aufgerufen.
        pose = [x, y, z, rx, ry, rz] in m bzw. rad (RPY).
        Winkel werden entfaltet (keine ±pi Sprünge im Plot), Geschwindigkeit per Differenzenquotient.
        """
        with self._lock:
            prev = self.cart_frames[-1] if self.cart_frames else None
            values = list(pose)
            vels = [0.0] * len(values)
            if prev is not None:
                prev_p = [axis['p'] for axis in prev['axes']]
                for i in range(3, len(values)):
                    values[i] = prev_p[i] + math.remainder(values[i] - prev_p[i], 2.0 * math.pi)
                dt = time_s - prev['time']
                if dt <= 0.0:
                    return
                vels = [(values[i] - prev_p[i]) / dt for i in range(len(values))]

            self.cart_frames.append({
                'time': time_s,
                'axes': [{'p': p, 'v': v} for p, v in zip(values, vels)]
            })

    def set_joint_limits(self, limits: list):
        with self._lock:
            self.joint_limits = limits

    def get_joint_limits(self):
        with self._lock:
            return self.joint_limits

    def set_base_frame(self, frame: str):
        with self._lock:
            self.base_frame = frame

    def get_base_frame(self):
        with self._lock:
            return self.base_frame

    def set_cartesian_limits(self, controller: str, max_linear: float, max_angular: float):
        with self._lock:
            self.cartesian_limits[controller] = (max_linear, max_angular)

    def get_cartesian_limits(self):
        """dict: Controller-Name -> (max_linear, max_angular), nur bereits gelesene Instanzen."""
        with self._lock:
            return dict(self.cartesian_limits)

    def set_loaded_controllers(self, names):
        with self._lock:
            self.loaded_controllers = set(names)

    def get_loaded_controllers(self):
        """Set der geladenen Controller oder None, solange der controller_manager nicht abgefragt wurde."""
        with self._lock:
            return None if self.loaded_controllers is None else set(self.loaded_controllers)

    def update_stiffness(self, values: dict):
        """Übernimmt (Teil-)Werte der committed Steifigkeit, z.B. aus get_parameters oder /parameter_events."""
        with self._lock:
            if self.stiffness is None:
                # Partielle Events vor dem ersten vollständigen Lesen ignorieren
                if not all(k in values for k in ('frame', 'linear', 'angular', 'max_linear', 'max_angular')):
                    return
                self.stiffness = {}
            self.stiffness.update(values)
            self.stiffness_version += 1

    def get_stiffness(self):
        """(version, dict oder None)"""
        with self._lock:
            return self.stiffness_version, (dict(self.stiffness) if self.stiffness else None)

    def set_stiffness_apply_result(self, successful: bool, reason: str):
        with self._lock:
            self.stiffness_apply_result = (successful, reason)

    def pop_stiffness_apply_result(self):
        with self._lock:
            result = self.stiffness_apply_result
            self.stiffness_apply_result = None
            return result

    def set_status(self, state, connected, armed, stopped, active_nodes, active_controllers):
        with self._lock:
            self.state = state
            self.connected = connected
            self.armed = armed
            self.stopped = stopped
            self.active_nodes = active_nodes
            self.active_controllers = active_controllers

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
    def _window(self, frames, seconds):
        if not frames:
            return []

        cutoff_time = frames[-1]['time'] - seconds
        times = [f['time'] for f in frames]
        start_idx = bisect.bisect_right(times, cutoff_time)
        return list(frames)[start_idx:]

    def get_plot_data(self, seconds=4.0):
        with self._lock:
            return self._window(self.frames, seconds)

    def get_cartesian_plot_data(self, seconds=4.0):
        with self._lock:
            return self._window(self.cart_frames, seconds)

    def get_current_tcp_pose(self):
        """Letzte TCP-Pose [x, y, z, rx, ry, rz] oder None, falls noch kein TF da ist."""
        with self._lock:
            if not self.cart_frames:
                return None
            return [axis['p'] for axis in self.cart_frames[-1]['axes']]

    def get_state(self):
        with self._lock:
            return self.state

    def get_status(self):
        with self._lock:
            return {
                "connected": self.connected,
                "armed": self.armed,
                "state": self.state,
                "stopped": self.stopped,
                "active_nodes": self.active_nodes,
                "active_controllers": self.active_controllers
            }

    def get_active_controller_list(self) -> list:
        """active_controllers des control_center (", "-getrennt) als Liste."""
        with self._lock:
            return [name.strip() for name in self.active_controllers.split(",") if name.strip()]
    
    # --- Live-Getter für dein JointJogWidget ---
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