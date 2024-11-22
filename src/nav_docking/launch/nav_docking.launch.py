from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


    

def generate_launch_description():
    desired_aruco_marker_id_front = 20
    desired_aruco_marker_id_left = desired_aruco_marker_id_front + 1
    desired_aruco_marker_id_right = desired_aruco_marker_id_front + 2
    aruco_distance_offset_front = 0.29
    aruco_left_right_offset_front = 0.17
    aruco_distance_offset_dual = 0.2046
    aruco_center_offset_dual = 0.00

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
                {"camera_front_frame": "camera_front_frame"},
                {"camera_left_frame": "camera_left_frame"},
                {"camera_right_frame": "camera_right_frame"},
                {"desired_aruco_marker_id_front": desired_aruco_marker_id_front},
                {"desired_aruco_marker_id_left": desired_aruco_marker_id_left},
                {"desired_aruco_marker_id_right": desired_aruco_marker_id_right},
                {"aruco_distance_offset_front": aruco_distance_offset_front },
                {"aruco_left_right_offset_front": aruco_left_right_offset_front},
                {"aruco_distance_offset_dual": aruco_distance_offset_dual},
                {"aruco_center_offset_dual": aruco_center_offset_dual},
                {"marker_topic_front": "aruco_detect/markers_front"},
                {"marker_topic_left": "aruco_detect/markers_left"},
                {"marker_topic_right": "aruco_detect/markers_right"},
            ],
            remappings=[
                ('goal_pose', 'goal_pose')
                ]
        )
    ])


