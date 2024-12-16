from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Directory for configuration files
    boot_dir = get_package_share_directory('boot')

    return LaunchDescription([
        Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name='crop_box_filter',
            parameters=[os.path.join(boot_dir, 'config', 'ego_filter.yaml')],
            remappings=[
                ('input', 'scan_pointcloud'),
                ('output', 'scan_filtered')
            ],
            # Add QoS settings to match the subscriber node
            arguments=['--qos-reliability', 'reliable', '--qos-durability', 'transient_local']
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 40.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', 'scan_filtered'),
                ('scan', 'scan')
            ],
        )
    ])
