#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/AutoDiff>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <sstream>
#include <vector>

bool visualize_sample_poses = false;

using namespace std;
using namespace Eigen;
using namespace pcl;



class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode() : Node("scan_processing_node") {

        // Root path to package's share directory
        string package_share_directory = ament_index_cpp::get_package_share_directory("hand_eye_calibration");

        // Paths to saved .dat files from data generation
        string flange_poses_path = package_share_directory + "/data/calibration/flange_poses.dat";
        string scans_path = package_share_directory + "/data/calibration/synthetic_scans.dat";
        string calibration_board_points_path = package_share_directory + "/data/calibration/calibration_board_points.dat";

        // Load data
        loadFlangePoses(flange_poses_path);
        loadScans(scans_path);
        loadBoardPoints(calibration_board_points_path);


        // Perform hand-eye calibration and print result
        Matrix4d T_cam_ee = compute_T_cam_ee(flange_poses, scans, board_points);
        cout << "T_cam_ee: \n" << T_cam_ee << endl;

    }

private:
    vector<Matrix4d> flange_poses;
    vector<vector<Vector4d>> scans;
    vector<vector<Vector4d>> board_points;

    int numPoses = 0;
    int numScanPoints = 0;

    bool calibration_visualization = false; // flag for potential visualization

    // loading in robot poses and converting to 4d matrix
    void loadFlangePoses(string filename) {
        ifstream file(filename);

        string line;
        getline(file, line); // Skip first line header

        while (getline(file, line)) {
            
            if(!line.empty()) {
                istringstream iss(line);
                VectorXd pose(6);
                for (int i = 0; i < 6; i++) {
                    iss >> pose[i];
                }
                flange_poses.push_back(poseToMatrix(pose));
                numPoses++;
            }
        }

        file.close();
    }

    // loading in scans and storing in vector
    void loadScans(string filename) {
        ifstream file(filename);
        string line;
        getline(file, line); // Skip first line header
        
        vector<Vector4d> scan_points; // storage for scanned calibration board points

        while (getline(file, line)) {
            if(!line.empty()) {
                // Extract points and store in scan_points
                istringstream iss(line);
                Vector4d point;
                for (int i = 0; i < 4; i++) {
                    iss >> point[i];
                }
                scan_points.push_back(point);
            } else {
                // Empty line signifies end of scan, so push completed scan and clear to store next scan
                scans.push_back(scan_points);
                scan_points.clear();
            }
        }

        if(scans.size() > 0) {
            numScanPoints += scans[0].size();
        }
        
        file.close();
    }

    // loading in known calibration board points and making same number of copies as there are camera scans for point-to-point correspondence matching
    void loadBoardPoints(string filename) {
        
        ifstream file(filename);
        string line;

        vector<Vector4d> board_points_set;

        while (getline(file, line)) {
            if(!line.empty()) {
                // Extract points and store in scan_points
                istringstream iss(line);
                Vector4d point;
                for (int i = 0; i < 4; i++) {
                    iss >> point[i];
                }

                board_points_set.push_back(point);
            }
        }

        file.close();

        for(int i = 0; i < numPoses; i++) {
            board_points.push_back(board_points_set);
        }

        board_points_set.clear();
    }


    // converting pose from xyz roll pitch yaw to 4d matrix
    Matrix4d poseToMatrix(VectorXd pose) {
        Matrix4d matrix = Matrix4d::Identity();

        AngleAxisd rollRotation = AngleAxisd(pose(3), Vector3d::UnitX());
        AngleAxisd pitchRotation = AngleAxisd(pose(4), Vector3d::UnitY());
        AngleAxisd yawRotation = AngleAxisd(pose(5), Vector3d::UnitZ());

        Matrix3d rotationMat;
        rotationMat = yawRotation * pitchRotation * rollRotation;

        matrix.block<3, 3>(0, 0) = rotationMat;
        matrix.block<3, 1>(0, 3) = pose.head(3);

        return matrix;
    }


    // Helper to stage rgb values in the manner the PCL needs to XYZRGB points
    inline float PackRGB(uint8_t r,uint8_t g,uint8_t b) {
      uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
      return *reinterpret_cast<float*>(&color_uint);
    }

    // Visualization (not currently working properly)
    // void visualizeTransform(vector<Matrix4d> pose_list, vector<vector<Vector4d>> scan_points) {
    //     vtkObject::GlobalWarningDisplayOff();
    //     visualization::PCLVisualizer viewer("Transform Visualization");

    //     // Adding the known T_ee_cam here for visualization purposes, not used in estimation
    //     Matrix4d T_ee_cam;
    //     T_ee_cam << 1, 0, 0, 0,
    //                          0, 1, 0, 0,
    //                          0, 0, 1, 0.05,
    //                          0, 0, 0, 1;

    //     for(int i = 0; i < 1; i++) {
    //         Matrix4d pose = pose_list[i];
    //         vector<Vector4d> point_set = scan_points[i];

    //         cout << "robot pose is: \n" << pose << endl;

    //         PointCloud<PointXYZRGB>::Ptr camera_cloud(new PointCloud<PointXYZRGB>);
    //         PointCloud<PointXYZRGB>::Ptr ee_cloud(new PointCloud<PointXYZRGB>);
    //         PointCloud<PointXYZ>::Ptr world_cloud(new PointCloud<PointXYZ>);

    //         for(int point_index = 0; point_index < point_set.size(); point_index++) {
    //             Vector4d point = point_set[point_index];

    //             Vector4d point_ee = T_ee_cam * point;

    //             Vector4d point_world = pose.inverse() * point_ee;

    //             point_world /= point_world(3);

    //             PointXYZRGB pcl_point_camera;
    //             PointXYZRGB pcl_point_ee;
    //             PointXYZ pcl_point_world;

    //             // Add camera points
    //             pcl_point_camera.x = point(0);
    //             pcl_point_camera.y = point(1);
    //             pcl_point_camera.z = point(2);
    //             pcl_point_camera.rgb = PackRGB(0, 255, 0);
    //             camera_cloud->points.push_back(pcl_point_camera);

    //             // Add end effector points
    //             pcl_point_ee.x = point_ee(0);
    //             pcl_point_ee.y = point_ee(1);
    //             pcl_point_ee.z = point_ee(2);
    //             pcl_point_ee.rgb = PackRGB(255, 0, 0);
    //             ee_cloud->points.push_back(pcl_point_ee);

    //             // Add points transformed to world
    //             pcl_point_world.x = point_world(0);
    //             pcl_point_world.y = point_world(1);
    //             pcl_point_world.z = point_world(2);
    //             world_cloud->points.push_back(pcl_point_world);

    //         }

    //         string camera_name = "origianl_" + to_string(i);
    //         string ee_name = "ee_" + to_string(i);
    //         string world_name = "world_" + to_string(i);
    //         viewer.addPointCloud(camera_cloud, camera_name);
    //         viewer.addPointCloud(ee_cloud, ee_name);
    //         viewer.addPointCloud(world_cloud, world_name);

    //         Affine3f affine_pose = Affine3f(pose.cast<float>());

    //         cout << "affine robot pose is: \n" << affine_pose.matrix() << endl;
    //         viewer.addCoordinateSystem(0.1, affine_pose);

    //     }

    //     viewer.setBackgroundColor(0, 0, 0);
    //     viewer.addCoordinateSystem(0.5);

    //     // Give OpenGL some time to initialize
    //     this_thread::sleep_for(chrono::milliseconds(500));

    //     viewer.spin();
    // }


    // Function to make rotation matrix orthonormal
    void fixRotationMatrix(Matrix3d& R) {
        // Perform SVD on the rotation matrix R
        JacobiSVD<Matrix3d> svd(R, ComputeFullU | ComputeFullV);
        Matrix3d U = svd.matrixU();
        Matrix3d V = svd.matrixV();

        // Reconstruct the rotation matrix with orthogonality enforced
        R = U * V.transpose();
    }


    void buildCorrespondenceMatrix(MatrixXd& correspondenceMatrix, vector<Vector4d> X_set, vector<Vector4d> X_prime_set) {

        for (int i = 0; i < numScanPoints; i++) {
            Vector4d x = X_set[i];
            Vector4d x_prime = X_prime_set[i];

            int row = 3 * i;
            MatrixXd correspondenceBlock(3, 16);
            correspondenceBlock << 
                x[0], x[1], x[2], 1, 0, 0, 0, 0, 0, 0, 0, 0, -x[0] * x_prime[0], -x[1] * x_prime[0], -x[2] * x_prime[0], -x_prime[0],
                0, 0, 0, 0, x[0], x[1], x[2], 1, 0, 0, 0, 0, -x[0] * x_prime[1], -x[1] * x_prime[1], -x[2] * x_prime[1], -x_prime[1],
                0, 0, 0, 0, 0, 0, 0, 0, x[0], x[1], x[2], 1, -x[0] * x_prime[2], -x[1] * x_prime[2], -x[2] * x_prime[2], -x_prime[2];

            correspondenceMatrix.block<3, 16>(row, 0) = correspondenceBlock;
        }
    }

    void reshapeAndCondition(Matrix4d& T_cam_ee, VectorXd& T_vector, Matrix4d& pose) {    
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                T_cam_ee(i, j) = T_vector(i * 4 + j);
            }
        }

        T_cam_ee /= T_cam_ee(3, 3); // Normalize to make sure bottom-right is 1
        
        // Make rotation orthonormal
        Matrix3d R = T_cam_ee.block<3, 3>(0, 0);
        fixRotationMatrix(R);

        // Update the transformation matrix with the amended rotation matrix
        T_cam_ee.block<3, 3>(0, 0) = R;

        // Make last row 0, 0, 0, 1 for affine transform
        T_cam_ee.row(3) = Vector4d(0, 0, 0, 1);

        // Left-product with pose because T_cam^w = pose^(-1) * T_cam^ee
        T_cam_ee = pose * T_cam_ee;
    }


    Matrix4d compute_T_cam_ee(vector<Matrix4d> pose_list, vector<vector<Vector4d>>& X_sets, vector<vector<Vector4d>>& X_prime_sets) {

        Matrix4d T_ee_cam_accumulator = Matrix4d::Zero();

        vector<Matrix4d> T_world_cam_list;

        cout << "num poses: " << numPoses << endl;

        for (int setIdx = 0; setIdx < numPoses; setIdx++) {
            Matrix4d pose = pose_list[setIdx]; 
            Matrix4d pose_inverse = pose.inverse();

            // Build correspondence matrix using normalized points
            MatrixXd correspondenceMatrix(3 * numScanPoints, 16);

            // Using point-to-point correspondences to construct Ax = 0 system of equations, where x is the vectorized coefficients of T_cam_ee
            buildCorrespondenceMatrix(correspondenceMatrix, X_sets[setIdx], X_prime_sets[setIdx]);


            // Solve using SVD
            JacobiSVD<MatrixXd> svd(correspondenceMatrix, ComputeFullV);

            // Compute the transformation matrix from the SVD result
            VectorXd T_vector = svd.matrixV().col(15); // Extracting the smallest singular value solution
            
            // Reshape and condition SVD result into 4d matrix
            Matrix4d T_cam_ee;
            reshapeAndCondition(T_cam_ee, T_vector, pose);

            // Add 4d result to parent matrix for averaging over all poses
            T_ee_cam_accumulator += T_cam_ee;

        }


        // Take the average of parent sum matrix to arrive at final T_cam_ee estimate
        T_ee_cam_accumulator /= numPoses;

        return T_ee_cam_accumulator;
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
