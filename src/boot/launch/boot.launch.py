from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Loop through camera devices from /dev/video0 to /dev/video7 and set format to MJPG
    for i in range(8):
        device_path = f'/dev/video{i}'
        if os.path.exists(device_path):
            try:
                v4l2_command_set = ['v4l2-ctl', '--device', device_path, '--set-fmt-video', 'pixelformat=MJPG']
                subprocess.run(v4l2_command_set, check=True, capture_output=True)
                v4l2_command_get = ['v4l2-ctl', '--device', device_path, '--get-fmt-video']
                result = subprocess.run(v4l2_command_get, check=True, capture_output=True, text=True)
                for line in result.stdout.splitlines():
                    if "Pixel Format" in line:
                        pixel_format = line.split(":")[1].strip()
                        print(f"Index {i}: {pixel_format}\n-")
            except subprocess.CalledProcessError:
                print(f"Index {i}: {pixel_format}: No camera to set\n-")

    # Get the path to the pid_params.yaml file
    camera_calib_file = os.path.join(
        get_package_share_directory('camera_publisher'),
        'config',  # Make sure the config directory exists
        'calib.yaml')
    print(f"Camera calibration path: {camera_calib_file}\n-")

    return LaunchDescription([
        Node(
            package="mecanum_wheels",
            executable="phidgets_control",
            name="phidgets_control_node",
            output='screen',
            parameters=[
            ]
        ),
        Node(
            package="camera_publisher",
            executable="publisher_from_video",
            name="camera_left_publisher_node",
            output='screen',
            namespace="camera_left",
            parameters=[
                {'camera_calib_file': camera_calib_file},
                {"camera_index": 6},
                {"frame_id": "camera_front_left_frame"},
                {"camera_topic": "color/image_raw"},
                {"camera_info_topic": "color/camera_info"},
                {"desired_fps": 30},
                {"force_desired_fps": False},
            ]
        ),
        Node(
            package="camera_publisher",
            executable="publisher_from_video",
            name="camera_right_publisher_node",
            output='screen',
            namespace="camera_right",
            parameters=[
                {'camera_calib_file': camera_calib_file},
                {"camera_index": 0},  # 4
                {"frame_id": "camera_front_right_frame"},
                {"camera_topic": "color/image_raw"},
                {"camera_info_topic": "color/camera_info"},
                {"desired_fps": 30},
                {"force_desired_fps": False},
            ]
        ),
    ])
