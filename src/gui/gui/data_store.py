from collections import deque
import threading
import bisect
import math

class GuiDataStore:
    def __init__(self, maxlen=4000):
        self._lock = threading.Lock()
        self._maxlen = maxlen
        
        # 1. High-rate telemetry (actual values only, for plots and live queries)
        # Each frame holds 'time' and 'joint_state' (list of {'p':..., 'v':..., 't':...}, SI units)
        self.frames = deque(maxlen=maxlen)
        # TCP pose from TF (URDF root link -> tcp): 'time' and 'axes' (list of {'p':..., 'v':...} for X, Y, Z, Rx, Ry, Rz)
        self.cart_frames = deque(maxlen=maxlen)
        
        # 2. Current status of the control_center state machine
        self.connected = False
        self.armed = False
        self.state = "UNKNOWN"
        self.stopped = False
        self.active_node = ""
        self.active_controllers = []

        self.joint_limits = list()
        # Root link of the URDF (base frame of the TCP pose), None until the URDF has been read
        self.base_frame = None
        # Per jog controller instance: name -> (max_linear_velocity [m/s], max_angular_velocity [rad/s])
        self.cartesian_limits = {}
        # Names of the controllers loaded in the controller_manager, None until the first query
        self.loaded_controllers = None

        # Stiffness of the cartesian_impedance_controller as used by the controller ("committed").
        # dict with 'frame', 'linear', 'angular', 'max_linear', 'max_angular', or None until read
        self.stiffness = None
        # Counter, so the GUI notices changes (read, /parameter_events, apply)
        self.stiffness_version = 0
        # Result of the last apply: (successful, reason) or None, collected by the GUI
        self.stiffness_apply_result = None

        # 3. Log messages from ROS
        self.log_queue = deque(maxlen=50)

    def clear_store(self):
        with self._lock:
            self.connected = False
            self.armed = False
            self.state = "UNKNOWN"
            self.stopped = False
            self.active_node = ""
            self.active_controllers = []
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
        Called by the ROS node.
        actual_list is a list with one dict per axis:
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
        Called by the ROS node.
        pose = [x, y, z, rx, ry, rz] in m and rad (RPY).
        Angles are unwrapped (no ±pi jumps in the plot), velocity by finite differences.
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
        """dict: controller name -> (max_linear, max_angular), only instances read so far."""
        with self._lock:
            return dict(self.cartesian_limits)

    def set_loaded_controllers(self, names):
        with self._lock:
            self.loaded_controllers = set(names)

    def get_loaded_controllers(self):
        """Set of loaded controllers, or None while the controller_manager has not been queried."""
        with self._lock:
            return None if self.loaded_controllers is None else set(self.loaded_controllers)

    def update_stiffness(self, values: dict):
        """Take over (partial) values of the committed stiffness, e.g. from get_parameters or /parameter_events."""
        with self._lock:
            if self.stiffness is None:
                # Ignore partial events before the first complete read
                if not all(k in values for k in ('frame', 'linear', 'angular', 'max_linear', 'max_angular')):
                    return
                self.stiffness = {}
            self.stiffness.update(values)
            self.stiffness_version += 1

    def get_stiffness(self):
        """(version, dict or None)"""
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

    def set_status(self, state, connected, armed, stopped, active_node, active_controllers):
        with self._lock:
            self.state = state
            self.connected = connected
            self.armed = armed
            self.stopped = stopped
            self.active_node = active_node
            self.active_controllers = list(active_controllers)

    def add_log(self, level, name, text):
        """Called by the ROS node to buffer log messages."""
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_queue.append(f"[{ts}] {level} [{name}]: {text}")

    def get_new_logs(self):
        """Called by the GUI: returns the buffered log messages and clears the queue."""
        with self._lock:
            logs = list(self.log_queue)
            self.log_queue.clear()
        return logs

    # --- Getters for the GUI (plots) ---
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
        """Latest TCP pose [x, y, z, rx, ry, rz] in m and rad, or None while there is no TF yet."""
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
                "active_node": self.active_node,
                "active_controllers": list(self.active_controllers)
            }

    def get_active_controller_list(self) -> list:
        """Controller set of the control_center's running mission."""
        with self._lock:
            return list(self.active_controllers)
    
    # --- Live getters (latest frame) ---
    def has_joint_state(self) -> bool:
        """True once a /joint_states message has arrived."""
        with self._lock:
            return bool(self.frames)

    def get_current_positions(self) -> list:
        """Current actual positions of all axes in rad."""
        with self._lock:
            if not self.frames:
                return [0.0] * 6  # fallback while there is no ROS data yet
            
            last_frame = self.frames[-1]
            return [joint['p'] for joint in last_frame['joint_state']]

    def get_current_velocities(self) -> list:
        """Current actual velocities of all axes in rad/s."""
        with self._lock:
            if not self.frames:
                return [0.0] * 6
            
            last_frame = self.frames[-1]
            return [joint['v'] for joint in last_frame['joint_state']]

    def get_current_torques(self) -> list:
        """Current actual torques (effort) of all axes in Nm."""
        with self._lock:
            if not self.frames:
                return [0.0] * 6
            
            last_frame = self.frames[-1]
            return [joint['t'] for joint in last_frame['joint_state']]