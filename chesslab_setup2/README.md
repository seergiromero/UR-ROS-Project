
# CHESSLAB SETUP 2

## Overview

This repository provides a ROS 2 workspace configuration for simulating and controlling a Universal Robots (UR) arm equipped with a Robotiq 85 gripper as the end-effector tool. This setup is designed specifically for a chess lab environment and utilizes ROS 2 Humble along with advanced simulation tools and custom interfaces to facilitate fast prototyping and testing.

The included launch files and configurations allow you to easily simulate your chosen UR robot model with the Robotiq 85 gripper. Both the robot arm and the gripper are fully integrated and can be controlled using the ros2_control framework within the Ignition Gazebo simulation environment.

## Prerequisites
- Ubuntu 22.04 LTS (recommended for ROS 2 Humble compatibility)
- ROS 2 Humble Hawksbill installed and sourced
- `git` and basic build tools installed


## Getting Started

### 1. Create a ROS 2 Workspace

First, create a new ROS 2 workspace if you don't already have one:
```
mkdir -p ~/ws_chesslab/src
cd ~/ws_chesslab/src
```

### 2. Clone Required Repositories

Clone the following repositories into the `src` directory of your workspace:
```
git clone https://gitioc.upc.edu/ros2tutorials/kinenikros2.git
git clone https://gitioc.upc.edu/ros2tutorials/chesslab_setup2_interfaces.git
git clone https://gitioc.upc.edu/ros2tutorials/robotiq_85_gripper.git
```
> **Important:**  
> Make sure to also clone the **current repository** (i.e., the repository containing this README and the main launch files) into your `src` directory.

## Building the Workspace
After cloning all required repositories, build your workspace:
```
cd ~/ws_chesslab
colcon build --symlink-install
source install/setup.bash
```

## Usage

To launch the simulation and robot setup, use the main launch file (e.g., `chesslab__gz_.launch.py`).  
You can customize the launch using several important arguments:

### Key Launch Arguments

- **`ur_type`**:  
  Specifies the model/series of Universal Robot to simulate.  
  **Options:** `ur3`, `ur3e`, `ur5`, `ur5e`, `ur7e`, `ur10`, `ur10e`, `ur12e`, `ur16e`, `ur20`, `ur30`  
  **Default:** `ur3e`

- **`launch_rviz`**:  
  Launches RViz for robot visualization if set to `true`.  
  **Options:** `true`, `false`  
  **Default:** `true`

- **`world_file`**:  
  The Gazebo world file to load. You must use a filename from the package folder worlds.  
  **Default:** `flat_chesslab.sdf`

- **`gazebo_gui`**:  
  Starts Gazebo with the graphical user interface if `true`; headless mode if `false`.  
  **Options:** `true`, `false`  
  **Default:** `true`

### Example Launch Commands

#### Launch with Default Settings
```
ros2 launch chesslab_setup2 chesslab_gz.launch.py
```

#### Launch a Different UR Robot (e.g., UR5e), Headless Gazebo, RViz, Custom World
```
ros2 launch chesslab_setup2 chesslab_gz.launch.py
ur_type:=ur3
launch_rviz:=true
gazebo_gui:=false
world_file:=my_custom_world.sdf
```


### Argument Summary Table

| Argument         | Description                                         | Default             | Example Value         |
|------------------|-----------------------------------------------------|---------------------|----------------------|
| `ur_type`        | UR robot model/series                               | `ur3e`              | `ur3`               |
| `launch_rviz`    | Launch RViz visualization?                         | `true`              | `true`              |
| `world_file`     | Gazebo world file to use                            | `flat_chesslab.sdf` | `my_custom_world.sdf`|
| `gazebo_gui`     | Launch Gazebo with GUI?                            | `true`              | `false`              |

> **Tip:** You can combine these arguments as needed to tailor the simulation to your requirements.

---

For more advanced options (controllers, prefixes, etc.), see the launch file or use `--show-args` with `ros2 launch`.

