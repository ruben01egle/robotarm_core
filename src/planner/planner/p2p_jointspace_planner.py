import numpy as np

class TrajectoryPoint:
    def __init__(self, positions, velocities, torques):
        self.positions = positions
        self.velocities = velocities
        self.torques = torques

class JointSpacePlanner:
    def __init__(self, min_pos, max_pos, max_v, max_tau, dt=0.001):
        self.min_pos = np.array(min_pos)
        self.max_pos = np.array(max_pos)
        self.max_v = np.array(max_v)
        self.max_a = np.array(max_tau)  # Torque als Beschleunigungslimit
        self.dt = dt

    def _check_limits(self, p0, p1):
        if np.any(p0 < self.min_pos) or np.any(p0 > self.max_pos):
            raise ValueError(f"Start position not within limits!")
        if np.any(p1 < self.min_pos) or np.any(p1 > self.max_pos):
            raise ValueError(f"Goal position not within limits!")

    def calculate(self, request, progress_cb=None, is_canceled_cb=None):
        p0 = np.array(request.waypoints[0].joint_angles)
        p1 = np.array(request.waypoints[-1].joint_angles)
        
        self._check_limits(p0, p1)

        scale = request.motion_scale
        current_max_v = self.max_v * scale
        current_max_a = self.max_a * scale
        
        dist = p1 - p0
        abs_dist = np.abs(dist)

        # Zeitberechnung für S-Kurve (Sinus-Rampe)
        times_v = (2.0 * abs_dist) / current_max_v
        times_a = np.sqrt((2.0 * np.pi * abs_dist) / current_max_a)
        t_total = np.max(np.maximum(times_v, times_a))
        
        if t_total <= 0:
            return [TrajectoryPoint(p0.tolist(), np.zeros_like(p0).tolist(), np.zeros_like(p0).tolist())]

        steps = int(np.ceil(t_total / self.dt))
        trajectory = []

        for i in range(steps + 1):
            if is_canceled_cb and is_canceled_cb():
                return None
            
            t = i * self.dt
            tau = min(t / t_total, 1.0)
            
            # --- Position (s) ---
            # s = tau - sin(2*pi*tau) / (2*pi)
            s = tau - (1.0 / (2.0 * np.pi)) * np.sin(2.0 * np.pi * tau)
            curr_p = p0 + s * dist
            
            # --- Geschwindigkeit (v) ---
            # Ableitung von s nach t: ds/dt = ds/dtau * dtau/dt
            # ds/dtau = 1 - cos(2*pi*tau)
            # dtau/dt = 1 / t_total
            ds_dtau = 1.0 - np.cos(2.0 * np.pi * tau)
            curr_v = (dist / t_total) * ds_dtau
            
            # --- Torque (tau) ---
            # Initial auf 0 gesetzt wie gewünscht
            curr_tau = np.zeros(len(p0))
            
            # Punkt zur Liste hinzufügen
            trajectory.append(TrajectoryPoint(
                positions=curr_p.tolist(),
                velocities=curr_v.tolist(),
                torques=curr_tau.tolist()
            ))
            
            if progress_cb and i % 10 == 0:
                progress_cb(tau * 100.0)

        return trajectory