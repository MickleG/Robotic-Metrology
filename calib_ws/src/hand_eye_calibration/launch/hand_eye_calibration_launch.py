import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Uncomment nodes to run them (not really designed to run at the same time, so make sure other node is commented before running)
        
        Node(
            package='hand_eye_calibration',
            executable='scan_processing_node',
            name='scan_processing_node',
            output='screen',
        ),
        # Node(
        #     package='hand_eye_calibration',
        #     executable='calibration_node',
        #     name='calibration_node',
        #     output='screen',
        # ),
    ])
