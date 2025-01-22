# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    boot_dir = get_package_share_directory('boot')
    launch_dir = os.path.join(boot_dir, 'launch')
    map_dir = os.path.join(boot_dir, 'maps')
    model_name = "multigo"
    model_sdf_file = "model.sdf"
    model_urdf_file = "multigo.urdf"

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Our own map from map_dir is set here
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            boot_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(boot_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # DOCKING NODES VARIABLES
    LENGTH_ROTATION_CENTER_SOLO = 0.0
    LENGTH_ROTATION_CENTER_DOCKING = 0.25
    LENGTH_ROTATION_CENTER_COMBINE_CHAIR = 0.5
    aurco_marker_width = 0.05
    desired_aruco_marker_id_left = 20
    desired_aruco_marker_id_right = desired_aruco_marker_id_left + 1
    nav2_distance_offset = 0.50
    aruco_distance_offset = 0.305
    aruco_left_right_offset = 0.17
    aruco_distance_offset_dual = 0.430
    aruco_center_offset_dual = 0.0
    # Get the path to the pid_params.yaml file
    pid_params_file = os.path.join(
        get_package_share_directory('nav_docking'),
        'config',  # Make sure the config directory exists
        'docking_pid_params.yaml'
    )


    # LOCALIZATION PARAMETERS
    lifecycle_nodes_localization = ['map_server',
                       'pointcloud_to_laserscan',
                       'robot_state_publisher',
                       'rtabmap_localizer']
    remappings = [('/tf', '/tf'),
                  ('/tf_static', '/tf_static'), 
                  ('/scan', '/scan_laserscan')]
    parameters_rtabmap={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'use_action_for_goal':False,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.5',
          'Grid/RangeMax':'10.0',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Mem/RawDescriptorsKept':'true',
          'Grid/CellSize': '0.05',  # Larger cell size reduces the resolution
          # To use localization mode, 'Mem/IncrementalMemory':'false'
          'Mem/IncrementalMemory':'false',
          'Rtabmap/WorkingDirectory': map_dir,  # Path to the pre-saved RTAB-Map database (map)
          'database_path': os.path.join(map_dir, 'rtabmap.db'),
          }

    remappings_rtabmap = [
        ('scan', '/scan_laserscan'),
        ('cloud_in', '/scan_filtered'),
        ('odom', '/odom'),
        ('map', '/map'),
        ('goal_pose', '/goal_pose'),
        ('cmd_vel', '/cmd_vel'),
        ('local_costmap/costmap', '/local_costmap/costmap'),
        ('global_costmap/costmap', '/global_costmap/costmap'),
        ]
    delay_duration = 2.0  # Delay for launching nodes
    # Define the robot URDF path using PathJoinSubstitution
    robot_urdf_path = os.path.join(boot_dir,'models' , model_name, model_urdf_file)
    print(robot_urdf_path)
    # Open the URDF file
    urdf_path = robot_urdf_path
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()


    # NAVIGATION PARAMS
    lifecycle_nodes_navigation = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']


    ##### Group DOCKING NODES #####
    load_docking_nodes = GroupAction(
        actions=[
            #NAV CONTROL NODE
            Node(
                package='nav_control',
                executable='nav_control_node',
                name='nav_control_node',
                parameters=[
                    {"input_topic": "cmd_vel_final"},
                    {"output_topic": "cmd_vel"},
                    {"mode_drive": "DOCKING"},
                    {"LENGTH_ROTATION_CENTER_SOLO": LENGTH_ROTATION_CENTER_SOLO},
                    {"LENGTH_ROTATION_CENTER_DOCKING": LENGTH_ROTATION_CENTER_DOCKING},
                    {"LENGTH_ROTATION_CENTER_COMBINE_CHAIR": LENGTH_ROTATION_CENTER_COMBINE_CHAIR},
                ],
                remappings=[]),
            
            
            #ARUCO DETECT NODES - LEFT AND RIGHT
            Node(
                package='aruco_detect',
                executable='aruco_detect_node',
                name='aruco_detect_left_node',
                parameters=[
                    {"desired_aruco_marker_id": desired_aruco_marker_id_left},
                    {"marker_width": aurco_marker_width},
                    {"camera_topic": "/camera_left/color/image_raw"},
                    {"camera_info": "/camera_left/color/camera_info"}
                ],
                remappings=[
                    ('aruco_detect/markers', 'aruco_detect/markers_left')
                ]),
            Node(
                package='aruco_detect',
                executable='aruco_detect_node',
                name='aruco_detect_right_node',
                parameters=[
                    {"desired_aruco_marker_id": desired_aruco_marker_id_right},
                    {"marker_width": aurco_marker_width},
                    {"camera_topic": "/camera_right/color/image_raw"},
                    {"camera_info": "/camera_left/color/camera_info"}
                ],
                remappings=[
                    ('aruco_detect/markers', 'aruco_detect/markers_right')
                ]),
            
            
            # NAV2 GOAL PUB NODE
            Node(
                package='nav_goal',
                executable='nav_goal_node',
                name='nav_goal_node',
                parameters=[
                    {"map_frame": "map"},
                    {"camera_front_left_frame": "camera_front_left_frame"},
                    {"camera_front_right_frame": "camera_front_right_frame"},
                    {"desired_aruco_marker_id_left": desired_aruco_marker_id_left},
                    {"desired_aruco_marker_id_right": desired_aruco_marker_id_right},
                    {"aruco_distance_offset": -nav2_distance_offset},
                    {"aruco_left_right_offset": -aruco_left_right_offset},
                    {"marker_topic_front_left": "/aruco_detect/markers_left"},
                    {"marker_topic_front_right": "/aruco_detect/markers_right"},
                ],
                remappings=[('goal_pose', 'goal_pose')]),
            
            # NAV DOCKING NODE
            Node(
                package='nav_docking',
                executable='nav_docking_node',
                name='nav_docking_node',
                parameters=[pid_params_file,
                    {"base_frame": "base_link"},
                    {"camera_left_frame": "camera_front_left_frame"},
                    {"camera_right_frame": "camera_front_right_frame"},
                    {"desired_aruco_marker_id_left": desired_aruco_marker_id_left},
                    {"desired_aruco_marker_id_right": desired_aruco_marker_id_right},
                    {"aruco_distance_offset": aruco_distance_offset},
                    {"aruco_left_right_offset": aruco_left_right_offset},
                    {"aruco_distance_offset_dual": aruco_distance_offset_dual},
                    {"aruco_center_offset_dual": aruco_center_offset_dual},
                    {"marker_topic_left": "aruco_detect/markers_left"},
                    {"marker_topic_right": "aruco_detect/markers_right"},
                ],
                remappings=[
                    ('goal_pose', 'goal_pose')]),  

        ]
    )


    #####LOCALIZATION NODES #####
    load_localization_nodes = GroupAction(
        actions=[
            #Map Server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            
            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                            'robot_description': robot_description}],
                remappings=remappings),
            
           # Pointcloud filter
            Node(
                package='ego_pcl_filter',
                executable='ego_pcl_filter_node',
                name='ego_pcl_filter_node',
                parameters=[
                            {'min_x': -0.20},
                            {'max_x': 0.20},
                            {'min_y': -0.15},
                            {'max_y': 0.15},
                            {'min_z': -0.3},
                            {'max_z': 1.5},
                            {'keep_organized': False},
                            {'negative': True}, ## Negative parameter (set to True to keep data outside the box)
                            {'input_frame': 'base_scan'},
                            {'output_frame': 'base_link'},
                            {'input_topic': 'scan_pointcloud'},
                            {'output_topic': 'scan_filtered'}
                            ],
                # Add QoS settings to match the subscriber node
                arguments=['--qos-reliability', 'reliable', '--qos-durability', 'transient_local']
            ),
            
            # 3d pointcloud to 2d laserscan conversion
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.01,
                    'min_height': 0.1,
                    'max_height': 3.0,
                    'angle_min': -3.1416,  # -M_PI/2
                    'angle_max': 3.1416,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.1,
                    'range_min': 0.00023,
                    'range_max': 40.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                remappings=[
                    ('cloud_in', 'scan_filtered'),
                    ('scan', 'scan_laserscan')
                ],
            ),
            
            # Node lifecylce manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_localization}]),

            # Timer action to delay RTAB-Map launch
            TimerAction(
                period=delay_duration,
                actions=[
                    # RTAB-Map Localization Node
                    Node(
                        package='rtabmap_slam',
                        executable='rtabmap',
                        name='rtabmap_localizer',
                        output='screen',
                        parameters=[parameters_rtabmap],
                        remappings=remappings_rtabmap,
                        arguments=[]
                    ),
                    
                    #A MCL localization
                    # Node(
                    #     package='nav2_amcl',
                    #     executable='amcl',
                    #     name='amcl',
                    #     output='screen',
                    #     respawn=use_respawn,
                    #     respawn_delay=2.0,
                    #     parameters=[configured_params],
                    #     arguments=['--ros-args', '--log-level', log_level],
                    #     remappings=remappings),
                ]
            ),
        ]
    )


    ##### LOAD NAVIGATION NODES #####
    load_navigation_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_navigation}]),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_localization_nodes)
    ld.add_action(load_navigation_nodes)
    ld.add_action(load_docking_nodes)

    return ld
