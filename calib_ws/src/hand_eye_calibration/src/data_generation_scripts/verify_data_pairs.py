# Python script to verify that point-to-point correspondence are being generated properly in data_generation_calibration.py

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

# Load synthetic scans (X_sets)
scans = []
with open("../../data/calibration/synthetic_scans.dat", "r") as f:
    lines = f.readlines()
    scan = []
    for i, line in enumerate(lines):
        if(i == 0):
            continue

        if line.strip():
            scan.append(np.array(list(map(float, line.split()))))
        else:
            scans.append(np.array(scan))  # Store one scan set
            scan = []

# Load board points (X_prime_sets)
board_points = np.loadtxt("../../data/calibration/calibration_board_points.dat")

# Pick one scan for visualization
scan_idx = 0  # Change index to check other scans
X_set = scans[scan_idx]
X_prime_set = board_points

# Plot points and connect them
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(X_set[:, 0], X_set[:, 1], X_set[:, 2], c='red', label='Scanned Points (X_set)')
ax.scatter(X_prime_set[:, 0], X_prime_set[:, 1], X_prime_set[:, 2], c='blue', label='Board Points (X_prime_set)')

# Connect corresponding points with lines
for i in range(len(X_set)):
    color = (random.random(), random.random(), random.random())  # Generate a random RGB color
    ax.plot([X_set[i, 0], X_prime_set[i, 0]],
            [X_set[i, 1], X_prime_set[i, 1]],
            [X_set[i, 2], X_prime_set[i, 2]], color=color, alpha=0.7, linewidth=1.5)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
ax.set_title(f"Visual Check of Correspondences (Scan {scan_idx})")

plt.show()
