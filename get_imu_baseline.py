from pymultiwii import MultiWii
import time
import numpy as np
import math


board = MultiWii('/dev/ttyUSB0')
print("IMU initialized")

n = 1000

pitch_data = np.zeros(n)

for i in range(n):
    # Reading in Orientation Data
    attitude = board.getData(MultiWii.ATTITUDE)
    roll = math.radians(attitude['angy'])
    pitch_data[i] = roll

pitch_stats = {}
pitch_stats["min"] = np.min(pitch_data)
pitch_stats["max"] = np.max(pitch_data)
pitch_stats["mean"] = np.mean(pitch_data)
pitch_stats["var"] = np.var(pitch_data)

print(pitch_stats)



