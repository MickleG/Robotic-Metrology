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

        float averageRMSE = validate_T(T_cam_ee, flange_poses, scans, board_points);

        cout << "Average RMSE: " << averageRMSE << endl;

        visualizeBackprojection(flange_poses, T_cam_ee, scans, board_points);

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
    void visualizeBackprojection(vector<Matrix4d> pose_list, Matrix4d T_cam_ee, vector<vector<Vector4d>> scan_points, vector<vector<Vector4d>> board_points) {
        vtkObject::GlobalWarningDisplayOff();
        visualization::PCLVisualizer viewer("Transform Visualization");


        for(int i = 0; i < 5; i++) {
            Matrix4d pose = pose_list[i];
            vector<Vector4d> point_set = scan_points[i];
            vector<Vector4d> board_point_set = board_points[i];


            PointCloud<PointXYZRGB>::Ptr camera_cloud(new PointCloud<PointXYZRGB>);
            PointCloud<PointXYZRGB>::Ptr ee_cloud(new PointCloud<PointXYZRGB>);
            PointCloud<PointXYZ>::Ptr backprojected_cloud(new PointCloud<PointXYZ>);
            PointCloud<PointXYZRGB>::Ptr board_cloud(new PointCloud<PointXYZRGB>);


            for(int point_index = 0; point_index < point_set.size(); point_index++) {
                Vector4d point = point_set[point_index];
                Vector4d point_board = board_point_set[point_index];

                Vector4d point_ee = T_cam_ee * point;

                Vector4d point_backprojected = pose.inverse() * point_ee;

                point_backprojected /= point_backprojected(3);

                PointXYZRGB pcl_point_camera;
                PointXYZ pcl_point_backprojected;
                PointXYZRGB pcl_point_board;

                // Add camera points
                pcl_point_camera.x = point(0);
                pcl_point_camera.y = point(1);
                pcl_point_camera.z = point(2);
                pcl_point_camera.rgb = PackRGB(0, 255, 0);
                camera_cloud->points.push_back(pcl_point_camera);

                // Add backprojected points
                pcl_point_backprojected.x = point_backprojected(0);
                pcl_point_backprojected.y = point_backprojected(1);
                pcl_point_backprojected.z = point_backprojected(2);
                backprojected_cloud->points.push_back(pcl_point_backprojected);

                // Add original board points
                pcl_point_board.x = point_board(0);
                pcl_point_board.y = point_board(1);
                pcl_point_board.z = point_board(2);
                pcl_point_board.rgb = PackRGB(0, 0, 255);
                board_cloud->points.push_back(pcl_point_board);

            }

            string camera_name = "origianl_" + to_string(i);
            string backprojected_name = "backprojected_" + to_string(i);
            string board_name = "board_" + to_string(i);

            viewer.addPointCloud(camera_cloud, camera_name);
            viewer.addPointCloud(backprojected_cloud, backprojected_name);
            viewer.addPointCloud(board_cloud, board_name);

            Affine3f affine_pose = Affine3f(pose.cast<float>());

            viewer.addCoordinateSystem(0.1, affine_pose);

        }

        viewer.setBackgroundColor(0, 0, 0);
        viewer.addCoordinateSystem(0.5);

        // Give OpenGL some time to initialize
        this_thread::sleep_for(chrono::milliseconds(500));

        viewer.spin();
    }




    Matrix4d compute_T_cam_ee(vector<Matrix4d> pose_list, vector<vector<Vector4d>>& X_sets, vector<vector<Vector4d>>& X_prime_sets) {

        Matrix4d T_ee_cam_accumulator = Matrix4d::Zero();

        vector<Matrix4d> T_world_cam_list;

        cout << "num poses: " << numPoses << endl;

        for (int setIdx = 0; setIdx < numPoses; setIdx++) {

            vector<Vector4d> X_set = X_sets[setIdx];
            vector<Vector4d> X_prime_set = X_prime_sets[setIdx];


            int setSize = X_set.size();

            // Collect pose for T_cam_ee extraction from T_cam_world
            Matrix4d pose = pose_list[setIdx]; 
            Matrix4d pose_inverse = pose.inverse();

            // Compute centroids of set in pose
            Vector3d centroid_X = Vector3d::Zero();
            Vector3d centroid_X_prime = Vector3d::Zero();

            for (int i = 0; i < setSize; ++i) {
                centroid_X += X_set[i].head<3>();
                centroid_X_prime += X_prime_set[i].head<3>();
            }

            centroid_X /= setSize;
            centroid_X_prime /= setSize;


            // Construct covariance matrix
            MatrixXd X_mat(setSize, 3);
            MatrixXd X_prime_mat(setSize, 3);

            for (int i = 0; i < setSize; ++i) {
                X_mat.row(i) = (X_set[i].head<3>() - centroid_X).transpose();
                X_prime_mat.row(i) = (X_prime_set[i].head<3>() - centroid_X_prime).transpose();
            }

            Matrix3d matrixProduct = X_mat.transpose() * X_prime_mat;


            // Perform SVD on covariance matrix - if det(V) == -1 then we account for reflection for our T_cam_world (which should work for real scan data)
            JacobiSVD<Matrix3d> svd(matrixProduct, ComputeFullU | ComputeFullV);
            Matrix3d U = svd.matrixU();
            Matrix3d V = svd.matrixV();

            Matrix3d R = V.transpose() * U;

            // Calculate translation vector
            Vector3d t = centroid_X_prime - R * centroid_X;


            Matrix4d T_cam_world = Matrix4d::Identity(); // Initialize as identity matrix

            // Assign rotation
            T_cam_world.block<3,3>(0,0) = R;

            // Assign translation
            T_cam_world.block<3,1>(0,3) = t;

            // Left product by robot pose to collect T_cam_ee as T_cam_world = pose.inverse() * T_cam_ee
            Matrix4d T_cam_ee = pose * T_cam_world;

            // noramlize so last row is 0, 0, 0, 1
            T_cam_ee /= T_cam_ee(3, 3);


            // Add 4d result to parent matrix for averaging over all poses
            T_ee_cam_accumulator += T_cam_ee;

        }


        // Take the average of parent sum matrix to arrive at final T_cam_ee estimate
        T_ee_cam_accumulator /= numPoses;

        return T_ee_cam_accumulator;
    }


    float validate_T(Matrix4d T_cam_ee, vector<Matrix4d> flange_poses, vector<vector<Vector4d>> scans, vector<vector<Vector4d>> board_points) {

        
        float averageRMSE = 0.0;

        for(int n = 0; n < numPoses; n++) {
            Matrix4d poseInverse = flange_poses[n].inverse();
            Matrix4d T_cam_world = poseInverse * T_cam_ee;

            float sumSquaredErrors = 0.0;

            vector<Vector4d> cameraPointSet = scans[n];
            vector<Vector4d> worldPointSet = board_points[n];

            for(int i = 0; i < cameraPointSet.size(); i++) {
                Vector4d backProjectedPoint = T_cam_world * cameraPointSet[i];
                Vector4d originalWorldPoint = worldPointSet[i];

                float squaredError = (backProjectedPoint.head<3>() - originalWorldPoint.head<3>()).squaredNorm();

                sumSquaredErrors += squaredError;
            }

            // Calculate the Mean Squared Error (MSE)
            float meanSquaredError = sumSquaredErrors / cameraPointSet.size();

            // Calculate the Root Mean Squared Error (RMSE)
            float RMSE = sqrt(meanSquaredError);

            averageRMSE += RMSE;
        }

        
        return averageRMSE / numPoses;
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
