#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <pcl/visualization/pcl_visualizer.h> // Include for visualization

using namespace pcl;
using namespace std;

inline float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
    return *reinterpret_cast<float*>(&color_uint);
}

// Function to load a point cloud from a .dat file
PointCloud<PointXYZRGB>::Ptr loadPointCloud(const std::string &filename, vector<int> pointcloud_color) {
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    
    std::ifstream file(filename);

    float x, y, z;
    while (file >> x >> y >> z) {
        PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.rgb = PackRGB(pointcloud_color[0], pointcloud_color[1], pointcloud_color[2]);
        cloud->points.emplace_back(point);
    }

    file.close();
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    std::cout << "Loaded " << cloud->points.size() << " points from " << filename << std::endl;
    return cloud;
}

// Function to apply Statistical Outlier Removal (SOR)
void applySOR(PointCloud<PointXYZRGB>::Ptr &cloud) {
    StatisticalOutlierRemoval<PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
}

// Function to apply Moving Average Filtering (MAF)
void applyMAF(PointCloud<PointXYZRGB>::Ptr &cloud, int window_size = 3) {
    PointCloud<PointXYZRGB>::Ptr smoothed(new PointCloud<PointXYZRGB>);

    for (size_t i = 0; i < cloud->size(); i++) {
        PointXYZRGB smoothed_point;
        float sum_x = 0, sum_y = 0, sum_z = 0;

        // Apply a simple moving average filter
        int count = 0;
        for (int j = -window_size / 2; j <= window_size / 2; j++) {
            size_t index = i + j;
            if (index >= 0 && index < cloud->size()) {
                sum_x += cloud->points[index].x;
                sum_y += cloud->points[index].y;
                sum_z += cloud->points[index].z;
                count++;
            }
        }

        smoothed_point.x = sum_x / count;
        smoothed_point.y = sum_y / count;
        smoothed_point.z = sum_z / count;

        smoothed_point.rgb = cloud->points[i].rgb;  // Retain color

        smoothed->points.push_back(smoothed_point);
    }

    cloud = smoothed;
}

// Function to align point clouds using ICP
PointCloud<PointXYZRGB>::Ptr alignPointClouds(PointCloud<PointXYZRGB>::Ptr source, PointCloud<PointXYZRGB>::Ptr target) {
    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setEuclideanFitnessEpsilon(1e-4);
    icp.setMaximumIterations(100);
    
    PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);
    icp.align(*aligned);

    if (icp.hasConverged()) {
        std::cout << "ICP Converged, Score: " << icp.getFitnessScore() << std::endl;

        PointCloud<PointXYZRGB>::Ptr combined(new PointCloud<PointXYZRGB>);
        *combined = *target; // Start with the target
        *combined += *aligned; // Append aligned source


        return combined;

    } else {
        std::cerr << "ICP did not converge!" << std::endl;

        return nullptr;
    }
}

// Function to save aligned point cloud as CSV
void savePointCloud(const std::string &filename, const PointCloud<PointXYZRGB>::Ptr cloud) {
    std::ofstream file(filename);
    for (const auto &point : cloud->points) {
        file << point.x << "," << point.y << "," << point.z << "\n";
    }
    file.close();
    std::cout << "Saved aligned point cloud to " << filename << std::endl;
}

// Function to visualize point cloud
void visualizeMultiplePointClouds(vector<PointCloud<PointXYZRGB>::Ptr> clouds) {
    
    visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);  // Set background color to black

    for(int i = 0; i < clouds.size(); i++) {
        string name = "cloud_" + to_string(i);

        viewer.addPointCloud<PointXYZRGB>(clouds[i], name);
    }
    
    viewer.spin();
}

void visualizeSinglePointCloud(PointCloud<PointXYZRGB>::Ptr cloud) {
    
    visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);  // Set background color to black

    viewer.addPointCloud<PointXYZRGB>(cloud, "pointcloud");
    
    viewer.spin();
}

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("pointcloud_processor") {
        string package_share_directory = ament_index_cpp::get_package_share_directory("hand_eye_calibration");

        vector<int> pc1_color = {255, 0, 0};  // Red color for cloud1
        vector<int> pc2_color = {0, 255, 0};  // Green color for cloud2

        // Load point clouds
        cloud1 = loadPointCloud(package_share_directory + "/data/scan_processing/scan1.dat", pc1_color);
        cloud2 = loadPointCloud(package_share_directory + "/data/scan_processing/scan2.dat", pc2_color);


        if (cloud1->empty() || cloud2->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error: One or both point clouds could not be loaded!");
            return;
        }

        // Select filter type (Statistical Outlier Removal "SOR" or Moving Average Filter "MAF")
        string filter_type = "SOR";

        if (filter_type == "SOR") {
            applySOR(cloud1);
            applySOR(cloud2);
        } else if (filter_type == "MAF") {
            applyMAF(cloud1);
            applyMAF(cloud2);
        }


        cout << "pc1 size: " << cloud1->size() << endl;
        cout << "pc2 size: " << cloud2->size() << endl;

        vector<PointCloud<PointXYZRGB>::Ptr> misaligned_clouds = {cloud1, cloud2};

        // Align clouds using ICP
        aligned_cloud = alignPointClouds(cloud1, cloud2);

        // // Save the merged, aligned point cloud to CSV
        savePointCloud("aligned_scan.csv", aligned_cloud);

        if(scan_processing_visualization) {
            // Visualize the aligned point cloud
            visualizeSinglePointCloud(aligned_cloud);
        }
    }

private:
    PointCloud<PointXYZRGB>::Ptr cloud1;
    PointCloud<PointXYZRGB>::Ptr cloud2;
    PointCloud<PointXYZRGB>::Ptr aligned_cloud;

    bool scan_processing_visualization = true;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
