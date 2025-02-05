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

---
