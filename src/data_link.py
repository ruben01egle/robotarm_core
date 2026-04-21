import queue
import threading

class DataLink:
    def __init__(self):
        self._lock = threading.Lock()
        self._emergency_stop = False

        self.heartbeat_out = queue.Queue(maxsize=10)
        self.can_fd_out = queue.Queue(maxsize=10)
        self.trajectory_out = queue.Queue(maxsize=100)
        self.cmd_out = queue.Queue(maxsize=5)

        self.heartbeat_in = queue.Queue(maxsize=10)
        self.msg_status_in = queue.Queue(maxsize=10)
        self.log_in = queue.Queue(maxsize=10)
        self.can_fd_in = queue.Queue(maxsize=50)
        self.telemetry_in = queue.Queue(maxsize=500)


    @property
    def emergency_stop(self):
        with self._lock:
            return self._emergency_stop

    def set_emergency_stop(self, state: bool):
        with self._lock:
            self._emergency_stop = state
            if state:
                print("!!! Emergency Stop triggered in Data Interface !!!")