import threading
import time
from robot_protocol.udp_protocol import SystemMode, MessageType

class RobotController(threading.Thread):
    def __init__(self, data_link, gui_data_store):
        super().__init__(daemon=True)
        self.data_link = data_link
        self.gui_data_store = gui_data_store
        self.running = True
        
        # Interner State
        self.current_mode = SystemMode.IDLE

    def run(self):
        print("[Controller] State Machine gestartet...")
        while self.running:
            # 1. Telemetrie verarbeiten (High-Speed)
            self._process_telemetry()

            # 2. Status/Logs verarbeiten (Lower Speed)
            self._process_status_updates()

            # CPU entlasten, aber schnell genug für 1kHz reagieren
            # Da wir blockierende Queues mit Timeout nutzen können, 
            # ist ein kurzes sleep hier nur ein Fallback.
            time.sleep(0.001)

    def _process_telemetry(self):
        """Holt Telemetrie-Batches aus dem DataLink und füttert den GUI Store."""
        try:
            # Wir leeren die Queue so weit wie möglich
            while not self.data_link.telemetry_in.empty():
                msg = self.data_link.telemetry_in.get_nowait()
                
                # Ein CTelemetryMsg enthält ein Array von Datenpunkten
                for i in range(msg.mNumDataPoints):
                    data_point = msg.mData[i]
                    
                    # Umwandlung in das Format, das dein GuiDataStore erwartet
                    act_list = []
                    for axis_idx in range(1, 7):
                        ax = getattr(data_point, f"axis{axis_idx}")
                        act_list.append({
                            'p': ax.position, 
                            'v': ax.velocity, 
                            't': ax.torque
                        })

                    # Da der C-Roboter keine separaten Target-Listen in der Telemetrie schickt,
                    # setzen wir diese hier leer oder auf 0 (oder du erweiterst das Protokoll)
                    ref_list = [{'p': 0.0, 'v': 0.0, 't': 0.0}] * 6

                    self.gui_data_store.push_telemetry_frame(
                        time_s=data_point.timeUs / 1e6,
                        actual_list=act_list,
                        target_list=ref_list,
                        traj_prog=0 # Müsste aus einer Status-Msg kommen
                    )
        except Exception as e:
            pass # Queue leer

    def _process_status_updates(self):
        """Verarbeitet Heartbeats und System-Status."""
        # Heartbeat vom Roboter prüfen
        if not self.data_link.heartbeat_in.empty():
            hb_msg = self.data_link.heartbeat_in.get_nowait()
            self.current_mode = hb_msg.mHeader.mMode
            
            # Update GUI Status
            self.gui_data_store.set_status(
                connected=True,
                armed=(self.current_mode == SystemMode.ARMED),
                mode=str(SystemMode(self.current_mode).name),
                latency=0.0 # Hier könntest du die Zeitdifferenz berechnen
            )

    def stop(self):
        self.running = False