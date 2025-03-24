# Python script to generate simulated scan data for hand-eye calibration

import numpy as np
import scipy.spatial.transform as transform
import matplotlib.pyplot as plt

np.random.seed(42)

# Number of scans to visualize
num_scans_to_visualize = 10

# Define workspace bounds for flange poses
X_RANGE = (-0.5, 0.5)
Y_RANGE = (-0.5, 0.5)
Z_RANGE = (0.2, 0.8)

# Calibration board parameters
CALIB_BOARD_ROWS = 10
CALIB_BOARD_COLS = 10
CALIB_BOARD_SPACING = 0.02

# Simulated transform from end-effector to camera
T_ee_cam = np.eye(4)
T_ee_cam[:3, 3] = [0, 0, 0.05]  # Camera offset (50mm forward in EE frame)

# Generate flange poses that satisfy the constraint
num_poses = 500
flange_poses = np.zeros((num_poses, 6))

for i in range(num_poses):
    pos_ee = np.array([
        np.random.uniform(*X_RANGE),
        np.random.uniform(*Y_RANGE),
        np.random.uniform(*Z_RANGE)
    ])

    cam_pos_world = (np.eye(4) @ T_ee_cam)[:3, 3] + pos_ee
    z_axis = -cam_pos_world / np.linalg.norm(cam_pos_world)

    y_axis = np.array([0, 1, 0]) if abs(z_axis[2]) < 0.9 else np.array([1, 0, 0])
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    R_ee_world = np.column_stack((x_axis, y_axis, z_axis))
    rpy = transform.Rotation.from_matrix(R_ee_world).as_euler('xyz')

    flange_poses[i, :3] = pos_ee
    flange_poses[i, 3:] = rpy

# Convert flange poses to transformation matrices
def pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, :3] = transform.Rotation.from_euler('xyz', pose[3:]).as_matrix()
    T[:3, 3] = pose[:3]
    return T

# Generate calibration pattern
def generate_circle_pattern(rows, cols, spacing):
    pattern = []

    board_width = (rows - 1) * CALIB_BOARD_SPACING
    board_length = (cols - 1) * CALIB_BOARD_SPACING

    for i in range(rows):
        for j in range(cols):
            x = j * spacing - board_width / 2
            y = i * spacing - board_length / 2
            pattern.append([x, y, 0, 1])

    pattern = np.array(pattern)

    return pattern

calibration_pattern = generate_circle_pattern(CALIB_BOARD_ROWS, CALIB_BOARD_COLS, CALIB_BOARD_SPACING)

# Fix calibration board in the world frame
T_board_world = np.eye(4)

# Transform board points into world coordinates
calibration_pattern_world = (T_board_world @ calibration_pattern.T).T

# Simulating scan data
synthetic_scans = []
for pose in flange_poses:
    T_world_ee = pose_to_matrix(pose)
    T_world_cam = T_world_ee @ T_ee_cam  # Transformation from world to camera

    transformed_points_h = (np.linalg.inv(T_world_cam) @ calibration_pattern.T).T
    transformed_points = transformed_points_h[:, :3]

    noise = np.random.normal(scale=0.002, size=transformed_points.shape)  # 2mm Gaussian noise
    transformed_points += noise

    synthetic_scans.append(transformed_points_h)



# Save flange poses to .dat file
with open("../../data/calibration/flange_poses.dat", "w") as f:
    f.write("Data saved as (X Y Z Roll Pitch Yaw)\n")
    for pose in flange_poses:
        f.write(" ".join(map(str, pose)) + "\n")

# Save synthetic scans to .dat file (X Y Z points for each scan)
with open("../../data/calibration/synthetic_scans.dat", "w") as f:
    f.write(f"Number of poses: {num_poses}; Calibration board pattern: {CALIB_BOARD_ROWS}x{CALIB_BOARD_COLS}\n")  # Header: NumScans NumPointsPerScan
    for scan in synthetic_scans:
        for point in scan:
            f.write(" ".join(map(str, point)) + "\n")
        f.write("\n")  # Blank line to separate scans


# Save calibration board location in world coordinates
np.savetxt("../../data/calibration/calibration_board_points.dat", calibration_pattern, delimiter=' ', fmt='%f')
    

print("Simulated pose and scan files saved")

print("T_ee_cam: ", T_ee_cam)
print("T_cam_ee: ", np.linalg.inv(T_ee_cam))







# Visualization functions
def plot_frame(ax, T, label, size=0.05):
    origin = T[:3, 3]
    x_axis = origin + T[:3, 0] * size
    y_axis = origin + T[:3, 1] * size
    z_axis = origin + T[:3, 2] * size

    ax.quiver(*origin, *(x_axis - origin), color='r', arrow_length_ratio=0.2)
    ax.quiver(*origin, *(y_axis - origin), color='g', arrow_length_ratio=0.2)
    ax.quiver(*origin, *(z_axis - origin), color='b', arrow_length_ratio=0.2)

    ax.text(*origin, label, fontsize=8, color='black')

# Create side-by-side subplots
fig, axes = plt.subplots(1, 2, figsize=(14, 6), subplot_kw={'projection': '3d'})

# Left plot: Robot poses & camera orientations in world coordinates
ax1 = axes[0]
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_zlabel("Z (m)")
ax1.set_title("Robot Poses, Camera Orientations, and Calibration Board")

# Plot world frame and board frame
plot_frame(ax1, np.eye(4), "Base", size=0.1)
plot_frame(ax1, T_board_world, "Board", size=0.1)

# Plot calibration board points in world frame
ax1.scatter(calibration_pattern_world[:, 0], calibration_pattern_world[:, 1], calibration_pattern_world[:, 2], 
            s=5, color="black", label="Board Points")

# Select random poses for visualization
indices = np.random.choice(num_poses, num_scans_to_visualize, replace=False)

for idx in indices:
    T_world_ee = pose_to_matrix(flange_poses[idx])
    T_world_cam = T_world_ee @ T_ee_cam

    plot_frame(ax1, T_world_cam, f"Cam {idx}")

    # Transform scans to world frame
    scan_points = synthetic_scans[idx]
    scan_points_world_h = (T_world_cam @ scan_points.T).T
    scan_points_world = scan_points_world_h[:, :3]

    # Plot transformed scan points in world frame
    ax1.scatter(scan_points_world[:, 0], scan_points_world[:, 1], scan_points_world[:, 2], 
                s=10, label=f"Scan {idx}")

ax1.legend()

# Right plot: Synthetic scans in camera frame
ax2 = axes[1]
ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.set_zlabel("Z (m)")
ax2.set_title("Synthetic Scans in Camera Frame")

for idx in indices:
    scan_points = synthetic_scans[idx]
    ax2.scatter(scan_points[:, 0], scan_points[:, 1], scan_points[:, 2], s=10, label=f"Scan {idx}")

ax2.legend()

# Ensure equal scaling of both plots
def set_axes_equal(ax):
    """Set the aspect ratio of 3D plot to be equal for all axes."""
    x_limits = ax.get_xlim()
    y_limits = ax.get_ylim()
    z_limits = ax.get_zlim()

    max_range = np.max([x_limits[1] - x_limits[0], y_limits[1] - y_limits[0], z_limits[1] - z_limits[0]])

    x_center = (x_limits[0] + x_limits[1]) / 2
    y_center = (y_limits[0] + y_limits[1]) / 2
    z_center = (z_limits[0] + z_limits[1]) / 2

    ax.set_xlim([x_center - max_range / 2, x_center + max_range / 2])
    ax.set_ylim([y_center - max_range / 2, y_center + max_range / 2])
    ax.set_zlim([z_center - max_range / 2, z_center + max_range / 2])

set_axes_equal(ax1)
set_axes_equal(ax2)

plt.show()
