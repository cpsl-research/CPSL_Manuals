# iRobot Create 3 (Gazebo Simulation) User Manual

The following guide was created to setup the PX4 simulation environment in order to test out various sensing/control software.

## [Setup/Installation] 

## - For Simulating iRobotCreate3 on Gazebo Harmonic with ROS2 Jazzy on Ubuntu 24.04 using a TurtleBot4

The following steps can be used to setup a gazebo simulation of a TurtleBot4 with the following versions/software. Note, that the TurtleBot4 is built on top of the Create3 and so the CPSL's create3 may not come with all of the hardware available on the TurtleBot4

- Gazebo Version: GZ Harmonic
- ROS Version: ROS2 Jazzy
- Ubuntu Version: Ubuntu 24.04

### 1. Install ROS2 Jazzy
1. Install ROS2 Jazzy by following the instructions here: [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
    - NOTE: If you are setting up ROS2 for your user acount on a machine that already has ROS2 installed on it, you should just be able to add the following command (or one similar to it) to your .bashrc file
    ```
    source /opt/ros/jazzy/setup.bash 
    ```

2. As we will be using a gazebo simulation, at times it will be helpful to read ground truth data from the gazebo simulation environment into our ROS environment. This can be accomplished by installing the ros_gz bridge using the following commands (see [ros_gz readme](https://github.com/gazebosim/ros_gz/tree/jazzy) for more details):
    - First, add packages.ros.org to the OS so that it can find the pre-build binary
    ```
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get update
    ```
    - Next, install the ros_gz package for ROS2 Jazzy
    ```
    sudo apt install ros-jazzy-ros-gz
    ```
3. Finally, in order to read/write commands/data to the Create3, we must install a series of packages
    ```
    git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_Create3
    ```

    If you forgot to clone the submodules as well, you can use the following command:
    ```
    git submodule update --init --recursive
    ```

    Once, cloned, the following commands can be used to build/install the necessary packages

    ```
    cd CPSL_ROS2_Create3
    colcon build --symlink-install
    source install/setup.bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```

### 2. Setup Turtlebot 4 simulator
1. Finally manually install the turtlebot3 simulator from source into the desired ROS2 workspace
    - First, navigate to the desired ROS2 workspace's source directory (replace turtlebot4_ws with the path to your workspace)
    ```
    cd turtlebot4_ws/src
    ```
    - Clone the turtlebot4_simulator repo, and load the jazzy branch
    ```
    git clone https://github.com/turtlebot/turtlebot4_simulator.git -b jazzy
    ```
    - Install the requisite dependencies:
    ```
    cd ~/turtlebot4_ws
    rosdep install --from-path src -yi
    ```
    - Finally, build the packages:
    ```
    colcon build --symlink-install
    ```

## Tutorials/Basic Usage

### 1. Start basic sumulator

To run a basic simulation on of the turtlebot4, perform the following steps:

```
cd turtlebot4_ws
source install/setup.bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

### 2. Undocking the robot:
To undock the robot, send the following action command:

```
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

### 3. Docking the robot:
Once donce with the robot, send the following action command to redock the robot:
```
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
```

### 4. Controlling the vehicle with keyboard operation
```
cd CPSL_ROS2_Create3
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_unstamped
```

### 5. Accessing the lidar scan
1. Open Rviz
```
rviz2
```
2. Add a LaserScan topic, and set it to subscribe to the "/scan" topic of the turtlebot. For better visualization, I recommend using the "Points" style. 
3. To view the lidar scan data in the frame of the turtlebot4, set the Fixed frame to ```turtlebot4/rplidar_link/rplidar```

## Helpful Instructions

### 1. Running NoMachine in a headless (i.e.; without a monitor) mode
Note, the following instructions only need to be performed once. To run NoMachine in a headless mode, running the following commands on the server's terminal:
```
sudo systemctl stop gdm
sudo /etc/NX/nxserver --restart
```
## Helpful Documentation
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [TurtleBot Gazebo Installation Manual](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html#source-installation)
- [TurtleBot4 simulator github](https://github.com/turtlebot/turtlebot4_simulator/tree/jazzy)
- [iRobot Create3 Simulator github](https://github.com/iRobotEducation/create3_sim/tree/jazzy)