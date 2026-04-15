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