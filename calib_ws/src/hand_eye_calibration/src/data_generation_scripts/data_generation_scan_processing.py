# Python script to generate simulated data for scan processing task - generates 2 different point clouds with overlap + rotation and translation

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


np.random.seed(42)

def generate_point_cloud(num_points=10000, noise=0.02):
    """Generates a random 3D point cloud with some noise."""
    points = np.random.rand(num_points, 3)  # Base shape
    points[:, 0] *= 2  # Scale x
    points[:, 1] *= 1.5  # Scale y
    points[:, 2] *= 1  # Scale z
    return points

def random_transform(points):
    """Applies a random translation and rotation to a point cloud."""
    translation = np.random.uniform(-0.5, 0.5, size=(3,))
    rotation_angles = np.random.uniform(-np.pi/4, np.pi/4, size=(3,))  
    rotation = R.from_euler('xyz', rotation_angles).as_matrix()

    
    transformed_points = np.dot(points, rotation.T) + translation
    return transformed_points, rotation, translation

def add_unique_noise(points, noise_level=0.02):
    """Adds unique Gaussian noise to a point cloud."""
    noise_values = np.random.normal(0, noise_level, points.shape)
    return points + noise_values

def generate_tail(base_points, tail_length=3, num_tail_points=5000):
    """Creates an extended 'tail' feature in the local x-direction of point cloud 2."""
    tail_x = np.linspace(-tail_length, 2, num_tail_points)
    tail_y = np.random.uniform(0, 0.25, num_tail_points)  # Small width
    tail_z = np.random.uniform(0, 0.25, num_tail_points)  # Small height
    tail = np.vstack((tail_x, tail_y, tail_z)).T  # Shape (num_tail_points, 3)
    return tail

def save_point_cloud(filename, points):
    """Saves a point cloud to a .dat file."""
    np.savetxt(filename, points, delimiter=" ")


def plot_point_clouds(clouds, colors, labels, title="3D Point Cloud Visualization"):
    """Plots multiple point clouds in 3D space with equal axis scaling."""
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    for cloud, color, label in zip(clouds, colors, labels):
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], c=color, s=1, label=label)

    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    ax.legend()

    # --- Force Equal Axis Scale ---
    x_limits = [min(np.min(cloud[:, 0]) for cloud in clouds), max(np.max(cloud[:, 0]) for cloud in clouds)]
    y_limits = [min(np.min(cloud[:, 1]) for cloud in clouds), max(np.max(cloud[:, 1]) for cloud in clouds)]
    z_limits = [min(np.min(cloud[:, 2]) for cloud in clouds), max(np.max(cloud[:, 2]) for cloud in clouds)]

    max_range = max(x_limits[1] - x_limits[0], y_limits[1] - y_limits[0], z_limits[1] - z_limits[0]) / 2.0

    mid_x = np.mean(x_limits)
    mid_y = np.mean(y_limits)
    mid_z = np.mean(z_limits)

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()


# Generate the base point cloud
pcd1 = generate_point_cloud()

pcd2 = pcd1.copy()

# Adding tail to point cloud 2 to aid in ICP visualization later
tail = generate_tail(pcd1)

# # Combine pcd1 with tail in local space before transformation
pcd2 = np.vstack((pcd1, tail))

# Apply a single rigid transformation to both pcd1 and tail
pcd2, rotation, translation = random_transform(pcd2)


pcd1 = add_unique_noise(pcd1, 0.02)
pcd2 = add_unique_noise(pcd2, 0.02)

# Save the point clouds
save_point_cloud("../../data/scan_processing/scan1.dat", pcd1)
save_point_cloud("../../data/scan_processing/scan2.dat", pcd2)

print("pointclouds saved")

# Visualize
plot_point_clouds(
    clouds=[pcd1, pcd2],
    colors=["red", "blue"],
    labels=["Scan 1 (Original)", "Scan 2 (Transformed + Tail)"]
)
