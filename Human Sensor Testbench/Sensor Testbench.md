# Human Sensor Testbench Setup & Usage Guide

This guide describes how to set up and run the CPSL Human Sensor Testbench using ROS2 Jazzy on Ubuntu 24.04. It covers installation and configuration for the required sensors: Livox Lidar, TI Radar, Leap Motion, and Intel RealSense. Tutorials for collecting datasets and running the sensor suite are included.

## System Requirements
- Ubuntu 24.04
- ROS2 Jazzy
- Python Poetry
- Livox Lidar (Mid360 recommended)
- TI IWR Radar (IWR1843/DCA1000)
- Leap Motion Controller 2
- Intel RealSense Camera

---

## 1. Install ROS2 Jazzy
Follow the official instructions: [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Add the following to your `.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2. Install Livox Lidar SDK
1. Install GCC 9.4.0:
   ```bash
   sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
   sudo apt update
   sudo apt install gcc-9 g++-9 -y
   ```
2. Clone and build Livox-SDK2:
   ```bash
   git clone https://github.com/Livox-SDK/Livox-SDK2.git
   cd Livox-SDK2
   mkdir build && cd build
   cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 ..
   make
   sudo make install
   ```

---

## 3. Install TI Radar Dependencies
Follow the "Pre-requisite packages" instructions in the [CPSL_TI_Radar_cpp GitHub](https://github.com/davidmhunt/CPSL_TI_Radar/tree/main/CPSL_TI_Radar_cpp).

---

## 4. Install Intel RealSense SDK & ROS Nodes
1. Install SDK:
   ```bash
   sudo apt install ros-jazzy-librealsense2*
   ```
2. Install ROS nodes:
   ```bash
   sudo apt install ros-jazzy-realsense2-*
   ```

---

## 5. Install Leap Motion Dependencies
1. Add Ultraleap repo and install:
   ```bash
   wget -qO - https://repo.ultraleap.com/keys/apt/gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ultraleap.gpg
   echo 'deb [arch=amd64] https://repo.ultraleap.com/apt stable main' | sudo tee /etc/apt/sources.list.d/ultraleap.list
   sudo apt update
   sudo apt install ultraleap-hand-tracking
   ```
2. Test installation:
   ```bash
   ultraleap-hand-tracking-control-panel
   ```

---

## 6. Install & Build CPSL_ROS2_Sensors
1. Clone the repo:
   ```bash
   git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_Sensors
   cd CPSL_ROS2_Sensors
   git submodule update --init --recursive
   ```
2. Configure Poetry:
   ```bash
   poetry config virtualenvs.options.system-site-packages true
   poetry env use /usr/bin/python3.12 # or your ROS2 Python version
   poetry install # or poetry install --with leapmotion
   eval $(poetry env activate)
   ```
3. Build ROS packages:
   ```bash
   cd CPSL_ROS2_Sensors
   eval $(poetry env activate) #if not already in the poetry shell
   python -m colcon build --base-paths src --symlink-install
   source install/setup.bash
   ```

---

## 7. Sensor Configuration
- Configure Livox Lidar IP and JSON files as described in the Livox SDK section.
- Set up TI Radar configuration files in the appropriate configs directory.
- Ensure Leap Motion and RealSense are connected and recognized by the system.

---

## 7a. Hardware Setup: Livox Lidar

### LivoxMid360 Ethernet Configuration
**First time only:**
- Set your system's ethernet interface to a static IPv4 address of `192.168.1.XX` and netmask `255.255.255.0`.
- Replace `XX` with the last two digits of the LiDAR's serial number (found on the side under the QR code). If the digits start with 0 (e.g., 09), use just the last digit (e.g., 9).
- Update `cmd_data_ip`, `push_msg_ip`, `point_data_ip`, and `imu_data_ip` in the JSON files at:
  - `~/CPSL_ROS2_Sensors/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`
  - `~/CPSL_ROS2_Sensors/src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json`
- Change the `ip` in `lidar_configs` to `192.168.1.1XX` where `XX` is the last two digits of the IP address.

---

## 7b. Hardware Setup: TI Radar

### Radar Setup
See the instructions in [CPSL_TI_Radar_ROS2](./src/CPSL_TI_Radar_ROS2/README.md) to ensure each radar is set up correctly. Each radar should have the mmWaveSDK3.6 demo installed.

### Radar Connection Setup
Connect the radars in the correct order:
- Connect the radar, if successful the following commands should return as follows
  ```bash
  ls /dev/ttyACM* # for IWR18XX should return /dev/ttyACM0 /dev/ttyACM1
  ls /dev/ttyUSB* # for IWR68XX should return /dev/ttyUSB0 /dev/ttyUSB1
  ```
- If you have trouble connecting, try:
  ```bash
  sudo usermod -a -G dialout $USER
  sudo chmod 666 /dev/ttyACM0 # or the relevant port
  ```
  (You may need to log out and back in for changes to apply.)

### Radar .json Configuration
Set up the `radar_0_IWR6843_ods_human_movement.json` configuration files in:
`/src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs`
This only needs to be performed the first time you use the radar on your system.

---

## 8. Running the Human Sensor Testbench

**Note:** You can either run the following commands manually in separate terminals as described below, or use the tmux script to launch all required panes and processes automatically by running:
> ```bash
> ./StartSensorTestbench.sh
> ```

### 1. Start All Sensors
Open a terminal and run:
```bash
cd CPSL_ROS2_Sensors
eval $(poetry env activate)
source install/setup.bash
ros2 launch cpsl_ros2_sensors_bringup human_movement_sensor_bringup.launch.py 
```

#### Launch Parameters
You can override launch parameters using the `parameter:=value` notation. Key parameters:
| **Parameter**                | **Default**           | **Description**                                                        |
|------------------------------|-----------------------|------------------------------------------------------------------------|
| `namespace`                  | cpsl_human_movement   | Namespace for all launched nodes and topics                            |
| `lidar_enable`               | true                  | Launch the Livox lidar node                                            |
| `lidar_scan_enable`          | false                 | Publish a /LaserScan message on the /scan topic                        |
| `radar_enable`               | true                  | Launch the TI radars (front and back)                                  |
| `camera_enable`              | false                 | Launch the USB camera node                                             |
| `realsense_enable`           | true                  | Launch the Intel RealSense camera node                                 |
| `leapmotion_enable`          | true                  | Launch the Leap Motion sensor node                                     |
| `platform_description_enable`| true                  | Publish the robot description corresponding to sensor locations        |
| `rviz`                       | false                 | Display an RViz window with all odometry displayed                     |

---

### 2. Collect a Dataset
Open a second terminal and run:
```bash
cd CPSL_ROS2_Sensors
eval $(poetry env activate)
source install/setup.bash
ros2 launch dataset_generator record_dataset.launch.py namespace:=cpsl_human_movement param_file:=human_movement.yaml
```

#### Dataset Configuration Example
Update the .yaml configuration file in `~/CPSL_ROS2_Sensors/src/dataset_generator/configs` as needed. Example:
```yaml
dataset_generator:
  ros__parameters:
    radar_enable: True
    lidar_enable: True
    lidar_topic: "livox/lidar"
    camera_enable: True
    camera_topic: "camera/cpsl_realsense/color/image_raw"
    camera_depth_enable: True
    camera_depth_topic: "camera/cpsl_realsense/depth/image_rect_raw"
    hand_tracking_enable: True
    hand_tracking_left_topic: "left_hand_joints"
    hand_tracking_right_topic: "right_hand_joints"
    imu_enable: False 
    imu_topic: "imu"
    vicon_enable: False
    vehicle_odom_enable: False
    vehicle_odom_topic: "odom"
    base_frame: "cpsl_human_movement/base_frame"
    frame_rate_save_data: 10.0
    frame_rate_high_speed_sensors: 20.0
    dataset_path: "/home/cpsl/Downloads/datasets/hand_pose_test_2"
```

---

## Notes
- When specifying topics in the .yaml config, do not use a leading `/` to allow dynamic namespacing.
- Ensure all hardware is connected and recognized before launching nodes.
- For troubleshooting, refer to the sensor-specific documentation and the CPSL_ROS2_Sensors README.

---

## Helpful Links
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Livox-SDK2 GitHub](https://github.com/Livox-SDK/Livox-SDK2)
- [CPSL_TI_Radar_cpp GitHub](https://github.com/davidmhunt/CPSL_TI_Radar/tree/main/CPSL_TI_Radar_cpp)
- [Leap Motion Controller 2 Downloads](https://www.ultraleap.com/downloads/leap-motion-controller-2/)
- [Intel RealSense ROS Wiki](https://github.com/IntelRealSense/realsense-ros)