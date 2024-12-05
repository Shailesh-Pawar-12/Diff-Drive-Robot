# Diff Drive Robot

## Overview

This repository implements a Differential Drive Robot system using ROS 2, designed for autonomous navigation and behavior-based control. The robot supports linear and rotational movements, integrates with ROS 2's navigation stack, and uses Behavior Trees (BT) for decision-making.

**Components**: 
1. diff_drive_behaviors: Implements behavior-based control for the robot, including linear and rotational movement behaviors.
2. diff_drive_behavior_tree: Defines the behavior tree actions for linear and rotational movements, and the task planner.
3. diff_drive_bringup: Launch files to bring up the robot system and visualize it in RViz.
4. diff_drive_description: Contains the robot’s URDF and visualization files for simulation and visualization in Gazebo and RViz.
5. diff_drive_interfaces: Custom ROS 2 action messages for linear and rotational movements.
6. diff_drive_robo_localization: Implements localization using EKF and AMCL algorithms.
7. diff_drive_robo_navigation: Configures the navigation stack for autonomous navigation.
8. ekf: Implements the Extended Kalman Filter (EKF) for sensor fusion. (Still under development)
   
## ROS 2 File Structure
```
.
├── diff_drive_robot
│   ├── diff_drive_behaviors
│   │   ├── behavior_plugin.xml
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── diff_drive_behaviors
│   │   │       ├── linear.hpp
│   │   │       ├── rotate.hpp
│   │   │       └── timed_behavior.hpp
│   │   ├── package.xml
│   │   └── src
│   │       ├── linear.cpp
│   │       └── rotate.cpp
│   ├── diff_drive_behavior_tree
│   │   ├── behavior_trees
│   │   │   ├── linear_rotate_action.xml
│   │   │   └── start.xml
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── diff_drive_behavior_tree
│   │   │       ├── bt_macros.hpp
│   │   │       ├── linear_action.hpp
│   │   │       └── rotate_action.hpp
│   │   ├── launch
│   │   │   └── sample_bt_executor.launch.xml
│   │   ├── package.xml
│   │   └── src
│   │       ├── bt_task_planner.cpp
│   │       ├── linear_action.cpp
│   │       └── rotate_action.cpp
│   ├── diff_drive_bringup
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── diff_drive_bringup.launch.py
│   │   │   └── diff_drive_rviz.launch.py
│   │   └── package.xml
│   ├── diff_drive_description
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── diff_drive_controller.yaml
│   │   │   └── gazebo_params.yaml
│   │   ├── include
│   │   │   └── diff_drive_description
│   │   ├── launch
│   │   │   └── diff_drive.launch.py
│   │   ├── package.xml
│   │   ├── rviz
│   │   │   └── diff_drive.rviz
│   │   ├── src
│   │   │   └── ekf_node.cpp
│   │   └── urdf
│   │       ├── diff_drive.xacro.urdf
│   │       └── home.world
│   ├── diff_drive_interfaces
│   │   └── bt_interfaces
│   │       ├── action
│   │       │   ├── Linear.action
│   │       │   └── Rotate.action
│   │       ├── CMakeLists.txt
│   │       ├── msg
│   │       └── package.xml
│   ├── diff_drive_robo_localization
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── amcl.yaml
│   │   │   └── ekf.yaml
│   │   ├── launch
│   │   │   ├── diff_drive_ekf.launch.py
│   │   │   └── diff_drive_map_amcl.launch.py
│   │   ├── map
│   │   │   ├── room.pgm
│   │   │   └── room.yaml
│   │   └── package.xml
│   ├── diff_drive_robo_navigation
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── nav2_params.yaml
│   │   ├── launch
│   │   │   ├── diff_drive_navigation.launch.py
│   │   │   └── navigation.launch.py
│   │   └── package.xml
│   ├── ekf
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── ekf
│   │   │       └── ekf.hpp
│   │   ├── package.xml
│   │   └── src
│   │       └── ekf.cpp
│   └── third_party
├── images
│   └── output.gif
└── README.md

```

## Prerequisites
1. Ubuntu 22.04 
2. Ros 2 Humble 
3. Nav 2

## Setup

##### Open a 3 terminal 

**1. In terminal A**
  ```
  colcon build
  source install/setup.bash
  ros2 launch diff_drive_bringup diff_drive_bringup.launch.py
  ```

**2. In terminal B**
  ```
  source install/setup.bash
  ros2 launch diff_drive_bringup diff_drive_rviz.launch.py
  ```

**3. In terminal C**
  ```
  source install/setup.bash
  ros2 run diff_drive_behavior_tree bt_task_planner
  ```

## Output
1. Navigation: The robot will autonomously navigate the environment using the ROS 2 navigation stack.
2. Behavior-Based Movement: The robot will perform linear and rotational movements based on the behavior tree.
 <img src="/images/output.gif" width="1000" />

#### Developer Information

- **Name:** Shailesh Pawar
- **Contact:** shaileshpawar320@gmail.com