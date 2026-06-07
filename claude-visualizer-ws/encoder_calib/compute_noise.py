import numpy as np
import csv

positions = []
with open("encoder_data.csv") as f:
    reader = csv.reader(f)
    for row in reader:
        positions.append(float(row[4]))  # sec, nanosec, frame_id, ticks, raw_position, dt_us

positions = np.array(positions)
print(f"Samples       : {len(positions)}")
print(f"Mean position : {np.mean(positions):.6f} rad")
print(f"kf_r_position : {np.var(positions):.8f} rad²")
