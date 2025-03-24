cmake_minimum_required(VERSION 3.5)
project(hand_eye_calibration)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


include_directories("/usr/include/eigen3/unsupported")


# Specify the source files
add_executable(scan_processing_node src/scan_processing_node.cpp)
add_executable(calibration_node src/calibration_node.cpp)

ament_target_dependencies(scan_processing_node rclcpp PCL Eigen3)
ament_target_dependencies(calibration_node rclcpp PCL Eigen3)

target_link_libraries(scan_processing_node ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(calibration_node ${PCL_LIBRARIES} ${VTK_LIBRARIES})


# Install executables
install(TARGETS
  scan_processing_node
  calibration_node
  DESTINATION lib/${PROJECT_NAME})


# Install include directories
install(DIRECTORY include/
  DESTINATION include)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install simulated data
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_package()