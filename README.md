# **Robotic Engineer Homework Assignment: 3D Scanning & Calibration**

## **Overview**
At Machina Labs, our robotics engineers regularly develop ROS 2-based calibration pipelines and point cloud processing workflows to enable accurate manufacturing processes.
This assignment evaluates your understanding of **robotic metrology**, **eye-in-hand calibration**, and **3D scanning data processing**. You will:
1. **Calibrate a robot-mounted scanner using two different methods and compare results.**
2. **Process noisy scan data and align multiple scans.**

**Implementation Requirement:**

- This assignment must be implemented in C++ within a ROS 2 package.
- Use Eigen for matrix operations and PCL (Point Cloud Library) for point cloud processing.
- Provide a CMake-based ROS 2 package (CMakeLists.txt and package.xml included).
- Use ROS 2 nodes to structure your implementation:
    - One node for calibration
    - One node for scan processing
    - A launch file to run everything together.


## **Part 1: Eye-in-Hand Calibration**

### **Scenario**
Your 3D scanner is mounted on a **6-axis robotic arm (e.g., UR5)**. To ensure accurate scanning, you must determine the **transformation matrix** between the scanner and the robot’s flange.

### **Given Data**
- Robot’s **flange poses** recorded at 500 positions: `X, Y, Z, Roll, Pitch, Yaw`
- Scanner’s **detected pattern poses**
- Calibration **Calibration pattern** (The calibration geometry and it's 3D scan is available). You can use any 3D surface and and some reasonable noise to it. 

### **Task**
1. **Implement two different hand-eye calibration methods** and compare their results.
2. Compute the **transformation matrix (T_cam^ee)** from the scanner to the robot’s flange.
3. Validate by projecting a known pattern into the **robot base frame**.
4. **Compare the accuracy and stability of both methods.**

### **ROS 2 Implementation Details**
- Implement a ROS 2 node for eye-in-hand calibration.
- The node should compute T_cam^ee using two different calibration methods.
- Provide a mechanism to trigger the calibration and retrieve results.

### **Expected Output**
- Two `T_cam_ee` (4x4 homogeneous transformation matrices) from different methods
- **Comparison of both methods** (accuracy, error metrics, and visualization)
- ROS 2 messages with computed transformations.

---

## **Part 2: Noisy Scan Data Processing**

### **Scenario**
Raw point cloud data contains **noise and misalignment**. You will filter and align **two overlapping scans**.

### **Task**
1. **Filter noise** using:
   - Statistical Outlier Removal (SOR)
   - Moving Average Filtering
2. **Align two scans using Iterative Closest Point (ICP).**
3. **Output the merged, aligned point cloud.**

### **ROS 2 Implementation Details**
- Implement a ROS 2 node for point cloud filtering and alignment.
- The node should process two overlapping scans and output a merged, aligned point cloud.
- Use noise filtering techniques and ICP-based alignment.
- The final output should be available for visualization and further use.

### **Expected Output**
📄 `aligned_scan.csv` (filtered & registered point cloud)
📊 **Before & After Point Cloud Alignment Visualization**

---

## **Bonus Challenge (Optional)**
🔹 **Detect occlusions dynamically** and adjust scan data accordingly.

---

## **Submission Requirements**
- C++ scripts for:
  - **Eye-in-hand calibration (two methods & comparison)**
  - **Point cloud filtering & alignment**
- `aligned_scan.csv`
- **Visualizations** of:
  - Calibration matrices (`T_cam^ee` for both methods)
  - Filtered & aligned point cloud

---

## **Evaluation Criteria**
✔️ **Correctness of the calibration matrices** (`T_cam^ee` from two methods)
✔️ **Comparison and analysis of the two methods**
✔️ **Effectiveness of noise filtering & scan alignment**
✔️ **Code readability & visualization clarity**


## MY IMPLEMENTATION
- Instructions:
This repo contains a ROS 2 Humble workspace that performs the two tasks. Each task can be utilized by uncommenting its appropriate node in the launch file. This workspace runs on Ubuntu 22.04, and utilizes Eigen, PCL, and VTK.
Data generation is inside of the data_generation_scripts directory. These data generation scripts were not designed to be ROS nodes, as they are ideally run before any calibration or data processing occurs.
    - data_generation_calibration.py generates the 500 robot poses and scans for hand-eye calibration
    - data_generation_scan_processing generates the 2 point clouds for ICP
    - verify_data_pairs.py shows the point-to-point correspondences for a single scan for debugging

- Calibration:
I tried many different strategies to perform hand-eye calibration, yet none of them worked very well to match the original T_cam_ee I set (for reference I did an identity rotation and 0.05 translation which can be seen in data_generation.py). I originally tried setting up a correspondence matrix of the form AX = 0, where x was the vectorized coefficients of T_cam_world, and A was a matrix made up of point-to-point correspondences for each pose. Then, for each pose, I could do T_world_ee (the pose of the robot arm in 4x4 matrix form) * X_{4x4} to solve for T_cam_ee. This was producing not so great results, as SVD was numerically unstable, with small variance between the smallest n singular values, and despite normalization efforts, I could not get an accurate T_cam^ee. My guess as to what went wrong is that my simulated data is not collecting the scans as an actual scanner would see them. If a scanner were facing at the calibration board, then points on the left would be present on the left of the image and points on the right would be on the right. However, in order to collect "simulated scans" I took the calibration board and transformed it along the kinematic chain to the camera coordinates, thereby essentailly producing a mirror of what the scan would actually look like if we were to use a real camera. This phenomenon can be seen when running verify_data_pairs.py. I tried making a dummy intrinsic matrix for the camera to use to solve this issue and actually simulate the scans properly, but realized that without any real scanner extrinsic matrix to go on, the 3D data could not be reconstructed from the pixels.
However, I continued trying, and implemented a covariance matrix-based approach, again pose-wise where I would compute T_cam_world for each pose and then take the left product with the robot arm pose (T_world_ee * T_cam_world) to get a T_cam_ee. My thought behind this was that this is a very popular method of estimating a transform and it can allow for reflection of the data, by allowing the determinant of R to be -1. It also does not require the lienarlization of the data I validated my T_cam_ee by backprojecting the camera points into world using X_world = T_ee_world * T_cam_ee * X_cam, and computing RMSE for each pose, and averaging the RMSE across all poses. I got a final averagedRMSE across the 500 poses to be 0.43 meters, which is not good at all. I still think that this is not matching properly due to the scanned data not simulating what the actual scan would see. This bad transform can be seen in the visualization of the backprojections (run after T_cam_ee is estimated), with the original board points in blue, the captured scan points in green, and the backprojected points in white.

If I were to do this task again, I would definitely spend more time trying to figure out how to properly simulate a 3D scan within the camera's coordinate frame. I learned from this that it is not as simple as a rigid transformation, there needs to be some mirroring to account for the fact that the camera is looking at the object. I would probably revisit my intrinsic matrix idea, and use that to determine X and Y values of the point in the camera frame, and then use the rigid transform along the kinematic chain to collect the Z-value of each point for the simulated scan.


- Scan Processing:
This task went great, I was able to generate the proper data and perform alignment using ICP after noise removal with SOR and MAF. SOR seems to work quite a lot better than MAF, as it is more agnostic to local data being different from one another. In MAF, because we have a moving window, if there is a big change in the geometry (as seen in the tail in point cloud 2), it mutates the data quite a bit, even with a small window size. As a result, the input data is modified more than when SOR was used, although I was able to align both point clouds using both strategies pretty accurately anyways.
---
