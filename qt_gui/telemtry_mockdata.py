import random
import math
import time

# Hilfsklasse, um so zu tun, als hätten wir ctypes-Strukturen
class MockAxis:
    def __init__(self, p, v, t):
        self.position = p
        self.velocity = v
        self.torque = t

class MockPacket:
    def __init__(self, idx, t_us):
        self.idx = idx
        self.timeUs = t_us
        # Wir erstellen 6 Achsen dynamisch
        for i in range(1, 7):
            # Erzeuge sinusförmige Daten für weiche Kurven im Plot
            p = math.sin(time.time() + i) + (random.random() * 0.05)
            v = math.cos(time.time() + i)
            trq = random.random() * 2.0
            setattr(self, f"axis{i}", MockAxis(p, v, trq))


from PyQt6.QtCore import QThread
import time

class SimulatorWorker(QThread):
    def __init__(self, data_store):
        super().__init__()
        self.data_store = data_store
        self.running = True
        self.sim_idx = 0

    def run(self):
        """Diese Methode läuft in einem eigenen Thread."""
        while self.running:
            self.sim_idx += 1
            curr_time = int(time.time() * 1e6)
            
            # Mock Daten wie vorher
            telemetry_mock = MockPacket(self.sim_idx, curr_time)
            trajectory_mock = MockPacket(self.sim_idx, curr_time)
            
            for i in range(1, 7):
                ax = getattr(trajectory_mock, f"axis{i}")
                ax.position += 0.2
            
            # In den Store pushen (Threadsafe dank deinem Lock!)
            try:
                self.data_store.push_frame(trajectory_mock, telemetry_mock)
            except ValueError as e:
                print(f"Thread Error: {e}")
            
            # Simulation von 1 kHz (1ms Pause)
            # Hier sieht man jetzt die Power: Der Thread ballert mit 1000 Hz,
            # während die GUI gemütlich mit 30-60 Hz zeichnet.
            time.sleep(0.001) 

    def stop(self):
        self.running = False
        self.wait()