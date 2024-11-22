# Multi-Go

## Overview
Multi-Go autonomous robot for wheelchair and cart mobility. 

**Keywords:** ARuco, Navigation stack, docking


## Prerequisites

- ROS2 Humble / Ubuntu 22.04 / NAV2 / Gazebo Simulator with ROS2 packages


## Installation

### Cloning

    git clone -b feature/wheelchair_docking https://github.com/ttvines/multigo.git
    
    cd ~/multigo
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install

Open model folder and move multigo and wheelchair models to .gazebo/models folder. 

## Run	
  Run following commands:
  
    cd ~/multigo
    source install/setup.bash 
    ros2 launch boot multigo_simulation_launch.py 

New tab

    source install/setup.bash 
    ros2 launch aruco_detect aruco_detect.launch.py 


New tab

    source install/setup.bash 
    ros2 launch nav_docking nav_docking.launch.py
