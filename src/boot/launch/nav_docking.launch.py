from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


    

def generate_launch_description():
    # Get the path to the pid_params.yaml file
    pid_params_file = os.path.join(
        get_package_share_directory('nav_docking'),
        'config',  # Make sure the config directory exists
        'docking_pid_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav_docking',
            executable='nav_docking_node',
            name='nav_docking_node',
            parameters=[pid_params_file,
                {"base_frame": "base_link"},
                {"camera_frame": "camera_front_frame"},
                {"desired_aruco_marker_id": 23},
                {"aruco_distance_offset": 0.26},
                {"aruco_left_right_offset": -0.045},
                {"marker_topic_front": "aruco_detect/markers_front"},
                {"marker_topic_left": "aruco_detect/markers_left"},
                {"marker_topic_right": "aruco_detect/markers_right"},
            ],
            remappings=[
                ('goal_pose', 'goal_pose')
                ]
        )
    ])


