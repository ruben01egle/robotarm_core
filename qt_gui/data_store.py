from collections import deque
import threading

class DataStore:
    def __init__(self, maxlen=10000):
        self._lock = threading.Lock()
        self.frames = deque(maxlen=maxlen)

    def push_frame(self, trajectory, telemetry):
        if trajectory.idx != telemetry.idx:
            raise ValueError(
                f"Synchronisation of trajectory and telemtry failed in gui: ({trajectory.idx})"
            )
        frame = {
            't': telemetry.timeUs,
            'idx': telemetry.idx,
            'target': trajectory,
            'actual': telemetry,
        }
        
        with self._lock:
            self.frames.append(frame)

    def get_plot_data(self, count=None):
        with self._lock:
            snapshot = list(self.frames)
        if count:
            return snapshot[-count:]
        else:
            return snapshot