from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import time
import os

def generate_launch_description():
    
    # Loop through camera devices from /dev/video0 to /dev/video5 and set format to MJPG
    for i in range(8):  # for /dev/video0 to /dev/video7
        device_path = f'/dev/video{i}'
        if os.path.exists(device_path):
            try:
                # Set MJPEG pixel format
                v4l2_command_set = ['v4l2-ctl', '--device', device_path, '--set-fmt-video', 'pixelformat=MJPG']
                subprocess.run(v4l2_command_set, check=True, capture_output=True)
                # Query the current pixel format
                v4l2_command_get = ['v4l2-ctl', '--device', device_path, '--get-fmt-video']
                result = subprocess.run(v4l2_command_get, check=True, capture_output=True, text=True)
                # Extract pixel format and print index
                for line in result.stdout.splitlines():
                    if "Pixel Format" in line:
                        pixel_format = line.split(":")[1].strip()
                        print(f"Index {i}: {pixel_format}")
            
            except subprocess.CalledProcessError:
                print(f"Index {i}: {pixel_format}: No camera to set")


    time.sleep(0.5)
    
    return LaunchDescription([
        Node(
            package="camera_publisher",
            executable="publisher_from_video",
            name="camera_left_publisher_node",
            namespace="camera_left",
            parameters=[
                {"camera_index": 6},
                {"frame_id": "camera_front_left_frame"},
                {"camera_topic": "color/image_raw"},  # Relative topic name
                {"camera_info_topic": "color/camera_info"},
                {"desired_fps": 30},
                {"force_desired_fps": False},
            ]
        ),
        Node(
            package="camera_publisher",
            executable="publisher_from_video",
            name="camera_right_publisher_node",
            namespace="camera_right",
            parameters=[
                {"camera_index": 0},  # 4
                {"frame_id": "camera_front_right_frame"},
                {"camera_topic": "color/image_raw"},  # Relative topic name
                {"camera_info_topic": "color/camera_info"},
                {"desired_fps": 30},
                {"force_desired_fps": False},
            ]
        ),

        
    ])
