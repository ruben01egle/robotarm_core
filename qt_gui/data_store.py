from collections import deque
import threading

class GuiDataStore:
    def __init__(self, maxlen=4000):
        self._lock = threading.Lock()
        self.frames = deque(maxlen=maxlen)

    def push_frame(self, trajectory, telemetry):
        frame = {
            't': telemetry.timeUs,
            'target': trajectory,
            'actual': telemetry,
        }
        
        with self._lock:
            self.frames.append(frame)

    def get_current_positions(self):
        """Gibt eine Liste der 6 aktuellen Ist-Positionen zurück."""
        with self._lock:
            if not self.frames:
                return [0.0] * 6  # Fallback, falls noch keine Daten da sind
            
            last_frame = self.frames[-1]
            actual = last_frame['actual']
            
            # Extrahiert Achse 1 bis 6 dynamisch
            return [
                getattr(actual, f"axis{i}").position 
                for i in range(1, 7)
            ]

    def get_plot_data(self, count=None):
        with self._lock:
            snapshot = list(self.frames)
        if count:
            return snapshot[-count:]
        else:
            return snapshot