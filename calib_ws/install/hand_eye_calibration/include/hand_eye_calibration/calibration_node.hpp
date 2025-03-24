#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

class CalibrationNode : public rclcpp::Node {
public:
    // Constructor
    CalibrationNode();


private:
    vector<Matrix4d> flange_poses;
    vector<vector<Vector4d>> scans;
    vector<vector<Vector4d>> board_points;

    int numPoses;
    int numScanPoints;
    
    bool calibration_visualization;
};
