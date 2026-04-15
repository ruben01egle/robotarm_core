import csv
from robot_protocol.data_types import CTrajectoryData

class TrajectoryProvider:
    def __init__(self):
        self.filepath = ""
        self.points = {}

    def load_from_csv(self, file_path):
        self.filepath = file_path
        new_points = {}

        expected_axes = [f"axis{i}_{prop}" for i in range(1, 7) 
                         for prop in ["pos", "vel", "trq"]]
        expected_header = ["step"] + expected_axes

        with open(file_path, mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f, delimiter=',')
            
            header = reader.fieldnames
            if header is None:
                raise ValueError(f"File '{file_path}' is empty or has no header")
            missing = [col for col in expected_header if col not in header]
            if missing:
                raise ValueError(f"CSV missing header: {missing}")

            for row_idx, row in enumerate(reader):
                try:
                    p = CTrajectoryData()
                    
                    idx = int(row["step"])
                    p.idx = idx
                    
                    for i in range(1, 7):
                        axis = getattr(p, f"axis{i}")
                        axis.position = float(row[f"axis{i}_pos"])
                        axis.velocity = float(row[f"axis{i}_vel"])
                        axis.torque   = float(row[f"axis{i}_trq"])
                    
                    new_points[idx] = p
                    
                except ValueError as e:
                    raise ValueError(f"data error in {row_idx + 2}: {e}")

        self.points = new_points
        print(f"Succesfully loaded: {len(self.points)} points from '{file_path}'.")

    def get_ref(self, idx):
        return self.points.get(idx)