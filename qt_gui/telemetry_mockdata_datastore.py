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
        """Diese Methode simuliert den 1kHz Datenstrom vom Roboter."""
        while self.running:
            self.sim_idx += 1
            # Zeit in Mikrosekunden simulieren
            curr_time_us = self.sim_idx * 1000 
            
            # 1. Rohdaten erzeugen (Simuliert ctypes Empfang)
            telemetry_mock = MockPacket(self.sim_idx, curr_time_us)
            trajectory_mock = MockPacket(self.sim_idx, curr_time_us)
            
            # Kleine Abweichung für den Soll-Wert simulieren
            for i in range(1, 7):
                getattr(trajectory_mock, f"axis{i}").position += 0.1

            # 2. DATEN ÜBERSETZEN (Das "Auspacken")
            # Wir bereiten die Listen für den Store vor
            act_list = []
            ref_list = []

            for i in range(1, 7):
                ax_name = f"axis{i}"
                a = getattr(telemetry_mock, ax_name)
                r = getattr(trajectory_mock, ax_name)
                
                act_list.append({'p': a.position, 'v': a.velocity, 't': a.torque})
                ref_list.append({'p': r.position, 'v': r.velocity, 't': r.torque})

            # 3. In den Store pushen
            # Wir berechnen hier auch direkt die Latenz (simuliert)
            fake_latency = random.uniform(0.5, 1.5) 
            
            self.data_store.push_telemetry_frame(
                time_s=curr_time_us / 1e6, 
                actual_list=act_list, 
                target_list=ref_list,
                traj_prog = 50
            )
            
            # Status-Werte separat setzen (nicht in jedem Frame)
            if self.sim_idx % 100 == 0: # Nur alle 100ms den Status updaten
                self.data_store.set_status(connected=True, armed= False, mode="SIMULATION", latency=fake_latency)

            time.sleep(0.001) # 1 kHz

    def stop(self):
        self.running = False
        self.wait()