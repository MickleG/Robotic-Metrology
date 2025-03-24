#include <rclcpp/rclcpp.hpp>


#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor();

private:
    PointCloud::Ptr cloud1;
    PointCloud::Ptr cloud2;
    PointCloud::Ptr aligned_cloud;

    bool scan_processing_visualization;
};