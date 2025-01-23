# PX4 Simulator Setup

The following guide was created to setup the PX4 simulation environment in order to test out various sensing/control software.

## [Setup/Installation] For Simulating PX4 V1.15 on Gazebo Harmonic with ROS2 Jazzy on Ubuntu 24.04

The following steps can be used to setup a gazebo simulation of a PX4 X500 UAV with the following versions/software

- Gazebo Version: GZ Harmonic
- ROS Version: ROS2 Jazzy
- Ubuntu Version: Ubuntu 24.04
- PX4 Version: V1.15
- X500 Model: Pixhawk board: X500 v2 (Pixhawk 6X flight hardware)

### 1. Install QGroundControl inorder to connect to and control UAV

Before setting up the PX4/ROS, QGroundControl must also be installed. Follow these steps to ensure that its installed correctly. To install it, please follow the steps on the [QGroundControl Download and Install instructions guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html), and be sure to follow the instructions for "Ubuntu Linux".

### 2. Install ROS2 Jazzy
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

3. To enable connecting to the PX4 over ethernet using ROS2, the XRCE-DDS Client must be installed. To do so, follow the instructions for "Setup Micro XRCE-DDS Agent & Client" at the [PX4 ROS2 User Guide](https://docs.px4.io/v1.15/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client). Essentially the following commands will be executed:
    - Install the Micro-XRCE-DDS-Agent
    ```
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
    ```
4. Finally, in order to read/write PX4 message in ROS, the px4_ros_com and px4_msgs ROS2 packages must be included in ROS2 workspace's source directory (change ROS_PX4_2_WS to the desired ws)
    ```
    cd ROS_PX4_2_WS/src
    git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_PX4.git
    ```

If you forgot to clone the submodules as well, you can use the following command:
    ```
    git submodule update --init --recursive
    ```

Once, cloned, the following commands can be used to build/install the necessary packages

    ```
    cd ROS_PX4_2_WS
    colcon build
    source install/setup.bash
    ```

### 3. Install Gazebo Harmonic through the Gazebo/ROS Paring
To simulate the UAV using Gazebo, the correct version of Gazebo must be installed (in this case Gazebo Harmonic). To achieve this, follow the [Gazebo Installation Guide (via ROS2)](https://gazebosim.org/docs/latest/ros_installation/). Ultimately, the only command that should need to be run to get Gazebo up and running is the following:
```
sudo apt-get install ros-jazzy-ros-gz
```

### 4.Install/Build the PX4 firmware (needed for simulation)
Once ROS2, Gazebo, and QGroundControl are installed, we will then move to install the PX4 Source Code. While the following steps should help, helpful documentation can be found at [checkout release versions](https://docs.px4.io/v1.15/en/contribute/git_examples.html#get-a-specific-release) and [Ubuntu Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html). To do so, perform the following steps:
1. Clone the PX4 Source Code:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

2. Although not stated in the v1.15 guide, there should also be an Ubuntu setup script which can be used to make sure that all of the software is correctly setup for ubuntu. This can be run using the following command(as documented in [Ubuntu Setup (Main) guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)):
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools
```

Note: the --no-sim-tools is used because we already installed Gazebo

## [Running] Simulations

The following tutorials are useful for setting up the gazebo simulation

### 1. X500 with 2D Lidar Sensor ("walls" world) using ROS2 integration

1. Start QGroundControl

2. Navigate to to the PX4-Autopilot directory and start the PX4-Autopilot application
    ```
    cd /path/to/PX4-Autopilot
    PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_2d
    ```
    - Note: ```PX4_GZ_WORLD=walls``` specifies the "walls" environment instead of the default blank environment, and ```make px4_sitl gz_x500_lidar_2d``` specifies a X500 UAV equiped with a 2D lidar scanner

3. Start the MicroXRCEAgent to enable the ROS2 connection
    ```
    MicroXRCEAgent udp4 -p 8888
    ```


4. To publish data from the 2D lidar sensor into ROS, we need to need to use the ros_gz_bridge ROS2 package. This should be able to be accomplished using the following command (or one similar to it)
    ```
    ros2 run ros_gz_bridge parameter_bridge /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
    ```
    - If everything is working correctly, you should now be able to run ```ros2 topic list``` and should see the topic displayed. Additionally, you should be able to view it in RVIZ as well using the ```rviz2``` command.
    - Note: you will likely have to change the RVIZ2 frame id for this to work properly in the current context

5. Finally, as an example of using ROS2, you can run the following ROS2 command to show an example of a launch file integration.
    ```
    ros2 run px4_ros_com offboard_control
    ```

## [Tutorials]

Below are several helpful tutorials to access data from a gazebo simulation in ROS

### 1. ROS_GZ_Bridge
Below are two potential strategies for publishing topics from Gazebo to ROS. See [Using ROS 2 to interact with Gazebo](https://gazebosim.org/docs/harmonic/ros2_integration/) for more information on how this is done. 
1. To publish data in ROS2 directly from the Gazebo, the following CLI command can be used
    ```
    ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG_TYPE@GZ_MSG_TYPE
    ```
    - If you need to determine the type of a Gazebo message, you can use the following command
    ```
    gz topic -t -t /GAZEBO_TOPIC_NAME
    ```

2. **TBD**: Need to also add how to do this in a launch file 

### 2. ROS2 2 Control of PX4
A good example/preview for how to control the PX4 using ROS2 can be found in the [ROS 2 Offboard Control Example](https://docs.px4.io/v1.15/en/ros2/offboard_control.html) page. A summary of the key steps is featured below as an example. Make sure that the ROS2 nodes have already been setup as stated above and the you have run ```colcon build``` on any packages you intend to use.
1. Start QGroundControl

2. Start the MicroXRCEAgent to enable the ROS2 connection
    ```
    MicroXRCEAgent udp4 -p 8888
    ```
3. Navigate to to the PX4-Autopilot directory and start the PX4-Autopilot application
    ```
    cd /path/to/PX4-Autopilot
    make px4_sitl gz_x500
    ```
4. Launch the example (will arm, ascend 5 meters, then wait):
    ```
    ros2 run px4_ros_com offboard_control
    ```

## Helpful Links: 
- [Building PX4 firmware instructions](https://docs.px4.io/v1.15/en/dev_setup/building_px4.html#nuttx-pixhawk-based-boards)
- [Pixhawk 6X Px4 page](https://docs.px4.io/main/en/flight_controller/pixhawk6x.html)
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Installation Guide (via ROS2)](https://gazebosim.org/docs/latest/ros_installation/)
- [Gazebo Simulation Instructions](https://docs.px4.io/main/en/sim_gazebo_gz/#installation-ubuntu-linux)
- [ROS_GZ_Bridge Tutorial](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [ROS_GZ_Bridge Github readme](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)
- [Gazebo Tutorials](https://gazebosim.org/docs/harmonic/tutorials/)
- [uORB Message Definitions](https://docs.px4.io/main/en/msg_docs/)