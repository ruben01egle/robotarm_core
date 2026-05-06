import numpy as np
import time

class JointSpacePlanner:
    def __init__(self, max_v, max_a, dt=0.001):
        """
        :param max_v: Liste/Array der maximalen Geschwindigkeiten pro Achse
        :param max_a: Liste/Array der maximalen Beschleunigungen pro Achse
        :param dt: Zeitschrittweite der Trajektorie
        """
        self.max_v = np.array(max_v)
        self.max_a = np.array(max_a)
        self.dt = dt

    def calculate(self, request, progress_cb=None, is_canceled_cb=None):
        # Extraktion der Wegpunkte (Start und Ziel aus dem Request)
        # Angenommen request hat .start_joints und .target_joints (JointAngles Objekte)
        p0 = np.array(request.waypoints[0].joint_angles)
        p1 = np.array(request.waypoints[-1].joint_angles)

        max_a = self.max_a * request.motion_scale
        max_v = self.max_v * request.motion_scale
        
        num_joints = len(p0)
        dist = p1 - p0  # Distanz pro Achse
        abs_dist = np.abs(dist)

        # 1. Minimale Zeit pro Achse berechnen (S-Kurve vereinfacht)
        # Wir berechnen die Zeit, die jede Achse bräuchte, wenn sie alleine führe
        # t_min = max(dist/v, sqrt(dist/a)) -> für S-Kurve leicht erhöht
        times_v = abs_dist / max_v
        times_a = np.sqrt(abs_dist / max_a) * 2 # Grobe Schätzung für Beschleunigungsphase
        
        t_total = np.max(np.maximum(times_v, times_a))
        
        if t_total <= 0:
            return [p0.tolist()]

        steps = int(t_total / self.dt)
        trajectory = []

        # 2. Generierung der S-Kurve (Sinus-Rampe für sanften Ruck)
        # s(t) verläuft von 0 bis 1
        for i in range(steps + 1):
            if is_canceled_cb and is_canceled_cb():
                return None
            
            t = i * self.dt
            # Normierte Zeit tau von 0 bis 1
            tau = t / t_total
            if tau > 1.0: tau = 1.0
            
            # S-Kurven Profil: s = tau - (1/(2*pi)) * sin(2*pi*tau)
            # Das sorgt für v=0 und a=0 an Start und Ziel
            s = tau - (1.0 / (2.0 * np.pi)) * np.sin(2.0 * np.pi * tau)
            
            # Interpolation im Joint Space
            current_pos = p0 + s * dist
            trajectory.append(current_pos.tolist())
            
            if progress_cb and i % 10 == 0:
                progress_cb(tau * 100.0)

        return trajectory