from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_goal',
            executable='nav_goal_node',
            name='nav_goal_node',
            parameters=[
                {"map_frame": "map"},
                {"camera_frame": "camera_front_frame"},
                {"desired_aruco_marker_id": 23},
                {"aruco_distance_offset": -0.5},
                {"aruco_left_right_offset": -0.00},
                {"marker_topic_front": "aruco_detect/markers_front"},
            ],
            remappings=[
                ('goal_pose', 'goal_pose')
            ]
        )
    ])


