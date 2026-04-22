import socket
import time
import math
import ctypes
import random
from PyQt6.QtCore import QThread

# Import der echten Protokoll-Strukturen
from robot_protocol.udp_protocol import (
    CTelemetryMsg, UDP_MAGIC_BYTE, MessageType, SystemMode
)
from robot_protocol.data_types import CTelemetryData, CAxisDataInt16

class HardwareSimulator(QThread):
    def __init__(self, target_ip="127.0.0.1", target_port=12345):
        super().__init__()
        self.target_peer = (target_ip, target_port)
        self.running = True
        self.sim_idx = 0
        
        # Socket für den "ausgehenden" Datenstrom vom Roboter zur App
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        print(f"[Sim] Starte Hardware-Simulation an {self.target_peer}")
        
        while self.running:
            self.sim_idx += 1
            
            # 1. Haupt-Container erstellen (CTelemetryMsg)
            # Laut udp_protocol.py enthält CTelemetryMsg einen Header und ein Array
            msg = CTelemetryMsg()
            
            # Header füllen
            msg.mHeader.mMagicByte = UDP_MAGIC_BYTE
            msg.mHeader.mMsgType = MessageType.TELEMETRY
            msg.mHeader.mTimestamp = int(time.time() * 1000) & 0xFFFFFFFF
            msg.mHeader.mMode = SystemMode.CONNECTED
            
            msg.mPacketNum = self.sim_idx
            msg.mNumDataPoints = 1 # Wir senden hier pro UDP-Paket einen Datenpunkt
            
            # 2. Telemetrie-Datenpunkt füllen (CTelemetryData)
            data_point = msg.mData[0] # Zugriff auf das erste Element im Array
            data_point.timeUs = self.sim_idx * 1000
            data_point.idx = self.sim_idx
            
            # Achsen-Werte simulieren (Sinus-Wellen)
            t = time.time()
            for i in range(1, 7):
                ax = getattr(data_point, f"axis{i}")
                # Wir müssen hier Ganzzahlen (int16) verwenden, da CAxisDataInt16 so definiert ist
                # Wir simulieren hier Rohwerte, die später durch die Skalierung wieder zu Floats werden
                ax.position = int(math.sin(t + i) * 1000) 
                ax.velocity = int(math.cos(t + i) * 500)
                ax.torque = int(random.uniform(-100, 100))

            # 3. Als Bytes über das Netzwerk schicken
            try:
                self.sock.sendto(bytes(msg), self.target_peer)
            except Exception as e:
                print(f"[Sim] Send error: {e}")

            # 1 kHz Simulation
            time.sleep(0.01)

    def stop(self):
        self.running = False
        self.wait()
        self.sock.close()