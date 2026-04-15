import csv
import os
import threading
import queue

class RobotLogger(threading.Thread):
    def __init__(self, log_directory="logs"):
        super().__init__()
        self.log_directory = log_directory
        self.log_queue = queue.Queue()
        self.daemon = True
        
        self._is_logging = False
        self._stop_event = threading.Event()
        
        # Header wie gehabt
        axes_cols = [f"axis{i}_{prop}" for i in range(1, 7) for prop in ["pos", "vel", "trq"]]
        self.header = ["step", "timestamp"] + axes_cols

        if not os.path.exists(log_directory):
            os.makedirs(log_directory)

    def start_session(self, filename):
        self.filepath = os.path.join(self.log_directory, filename)
        self._is_logging = True
        
        if not self.is_alive():
            self.start()

    def log_frame(self, telemetry_pkt):
        if self._is_logging:
            self.log_queue.put((telemetry_pkt))

    def run(self):
        with open(self.filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.header)
            writer.writeheader()
            
            while not self._stop_event.is_set() or not self.log_queue.empty():
                try:
                    ts, pkt = self.log_queue.get(timeout=1.0)
                    
                    row = {"step": pkt.idx, "timestamp": pkt.timeUs}
                    for i in range(1, 7):
                        axis = getattr(pkt, f"axis{i}")
                        row[f"axis{i}_pos"] = f"{axis.position:.6f}"
                        row[f"axis{i}_vel"] = f"{axis.velocity:.6f}"
                        row[f"axis{i}_trq"] = f"{axis.torque:.6f}"
                    
                    writer.writerow(row)
                    self.log_queue.task_done()
                    
                except queue.Empty:
                    continue

    def stop_session(self):
        self._is_logging = False
        self._stop_event.set()
        print("logger shutting down...")