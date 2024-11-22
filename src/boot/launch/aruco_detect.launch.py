from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_detect',
            executable='aruco_detect_node',
            name='aruco_detect_front_node',
            parameters=[
                {"marker_width": 0.05},
                {"camera_topic": "/camera/color/image_raw_front"},
                {"camera_info": "/camera/color/camera_info_front"}
            ],
            remappings=[
                ('aruco_detect/markers', 'aruco_detect/markers_front')
            ]
            ),
        Node(
                package='aruco_detect',
                executable='aruco_detect_node',
                name='aruco_detect_left_node',
                parameters=[
                    {"marker_width": 0.05},
                    {"camera_topic": "/camera/color/image_raw_left"},
                    {"camera_info": "/camera/color/camera_info_left"}
                ],
                remappings=[
                    ('aruco_detect/markers', 'aruco_detect/markers_left')
                ]
            ),
        Node(
                package='aruco_detect',
                executable='aruco_detect_node',
                name='aruco_detect_right_node',
                parameters=[
                    {"marker_width": 0.05},
                    {"camera_topic": "/camera/color/image_raw_right"},
                    {"camera_info": "/camera/color/camera_info_right"}
                ],
                remappings=[
                    ('aruco_detect/markers', 'aruco_detect/markers_right')
                ]
            ),

    ] 

    )
