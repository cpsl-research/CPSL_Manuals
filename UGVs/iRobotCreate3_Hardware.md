# iRobot Create 3 (Hardware Usage) User Manual

## - For Using iRobotCreate3 on Raspberri Pi5 with ROS2 Jazzy on Ubuntu 24.04

The following steps can be used to scontrol an iRobot Create3 with the following versions/software.

- ROS Version: ROS2 Jazzy
- Ubuntu Version: Ubuntu 24.04

## [Setup/Installation] with RPi5 (ROS2 Jazzy)

### 1. Install and Setup Ubuntu 24.04

1. Use the Raspberry Pi Imager (available here: [Raspberry Pi Imager](https://www.raspberrypi.com/software/)) to install Ubuntu 24.04 LTS Desktop onto the RPi 5, and follow the instructions to complete the installation

2. Once Ubuntu is installed, additionally install NoMachine by using the following instructions:
    - Go to [Nomachine Download](https://downloads.nomachine.com/), and click on the NoMachine for ARM (for RPi5) link. From there, click on the "NoMachine for ARMv8" "DEB" file which will work for ubuntu.
    -  Follow the instructions on the download page (see here: [instructions](https://downloads.nomachine.com/download/?id=115&distro=ARM))
    - Finally, to run NoMachine in a headless (i.e. without a monitor) mode, see the instructions below under "Helpful Instructions"
    - Once NoMachine is installed on the RPi5, you must also install it on your local machine to access the remote display. NOTE: If you are located at another location, you must connect via the Duke VPN to access the RPi5 remotely.

### 2. Setup Rpi5 Ethernet Interface

1. The create3 communicates with the RPi5 using ethernet over usb-c. If you are powering the RPi5 via the create3's USB-C port, please follow the following steps to activate the RPI5's ethernet over USB-C port:
    
    - open the /boot/firmware/config.txt file and add the following lines to the end of the file
    ```
    dtoverlay=dwc2,dr_mode=peripheral
    ```
    - Next, open the /boot/firmware/cmdline.txt file and add the following after ```rootwait```:
    ```
    modules-load=dwc2,g_ether
    ```
    - Finally, we will setup the USB-C ethernet port to work with the RPI5. To do so, open the /etc/netplan/50-cloud-init.yaml file and add the following text (correct indents as needed):
    ```
        usb0:
            dhcp4: false
            optional: true
            addresses: [192.168.186.3/24]
    ```
    - To apply the changes to the netplan, run the following commands
    ```
    sudo netplan generate
    sudo netplan apply
    ```
3. On the iRobot Create3, ensure that the USB/BLE toggle is set to "USB" and not BLE mode(see [Create 3 Adapter Board Documentation ](https://iroboteducation.github.io/create3_docs/hw/adapter/) for how to do this).
4. At this point, you should be able to go to ```192.168.186.2``` in a browser on the RPI5 and see the consul. If this is successful, then the ethernet connection on the RPi5 has successfully been connected to the Create3.

### 3. Setup Network Time Protocol for Create3
To setup NTP (network time protocol) on the compute board in order to synchronize time between the create3 and the RPI5. To do so, follow the instructions found at the [Creat 3 NTP Setup Documentation](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/)
- Note that the create3 may not initially synchronize with the RPi5 and that the ```sudo chronyc clients``` may not immediately return any clients.

### 4. Install ROS2 Jazzy
1. Install ROS2 Jazzy by following the instructions here: [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. Once ROS is installed, install the create3 messages for ROS2 Jazzy using the following command:
    ```
    sudo apt install -y ros-jazzy-irobot-create-msgs
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
### 5. Setting ROS2 Middleware
In order to connect to the Create3 using ROS, the correct MiddleWare and discovery settings must be applied.. The 1st option should work just fine, but you may need to try the second option as well 

#### Option 1: Default
1. By default, the Create3 uses the rmw_fastrtps_cpp and uses the ROS2 Domain ID of 0 (confirm in the Create3 web interface by going to Application -> Configuration). In order to connect to the Create3 in the easiest manner, add the following commands to your .bashrc
```
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=192.168.186.2
```

2. Once this is completed, you should be able to follow the "Nominal Startup of RPi5 + Create3" instructions to confirm that everything is setup correctly. 

#### Option 2: fast-DDS .xml file
1. If the below option doesn't work, you should be able to follow the "Fast-DDS" instructions here [Fast-DDS instructions](https://iroboteducation.github.io/create3_docs/setup/xml-config/) to establish a connection correctly. For example, create a file called "fast_dds.xml" in your Documents folder and paste the following contents inside:
```
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
   <participant profile_name="unicast_connection" is_default_profile="true">
       <rtps>
           <builtin>
               <metatrafficUnicastLocatorList>
                   <locator/>
               </metatrafficUnicastLocatorList>
               <initialPeersList>
                   <locator>
                       <udpv4>
                           <address>192.168.186.2</address>
                       </udpv4>
                   </locator>
               </initialPeersList>
           </builtin>
       </rtps>
   </participant>
</profiles>
```
2. Then in your .bashrc file, paste the following (instead of Option1's content):
```
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/cpsl/Documents/fast_dds.xml
```
3. Once this is completed, you should be able to follow the "Nominal Startup of RPi5 + Create3" instructions to confirm that everything is setup correctly. 
## [Nominal Startup/Running] of RPi5 + Create3:
Once everything is installed, the following steps can be followed to use the iRobotCreate3:

### 1. Connecting to the iRobotCreate3

1. Power/Reboot RPi5
2. [If in headless mode] Restart the Nomachine/GDM server to be able to access NoMachine
3. Restart/check the NTP server
```
sudo service chrony restart
sudo chronyc clients
```
4. Open a browser on the RPi5 and go to ```192.168.186.2``` (this is the interface for the create3)
5. Go to Application->Restart Application to restart the create3's application. This will take a few minutes, and the restart is complete after the robot makes a noise.
6. If everything went well, you should be able to confirm that the Create3 is connected via ROS2 by using the following command: ```ros2 node list```. If this command doesn't return anything, try the following solutions:
    - Log out and then back into the RPi5
    - Restart the NTP server again and wait until there is a confirmed connection with Create3 (should take a few minutes)

### 2. Undocking the robot:
To undock the robot, send one of the following action commands:

```
#for custom namespace
ros2 action send_goal /cpslCreate3/undock irobot_create_msgs/action/Undock "{}"

#for no custom namespace
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" 
```

### 3. Docking the robot:
Once donce with the robot, send the following action command to redock the robot:
```
#with custom namespace
ros2 action send_goal /cpslCreate3/dock irobot_create_msgs/action/Dock "{}"

#for no custom namespace 
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
```

### 4. Resetting the robot pose (say at a particular origin)
If you want to reset the pose to a specific location, you can use the following service
```
#with custom namespace
ros2 service call /cpslCreate3/reset_pose irobot_create_msgs/srv/ResetPose "pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}"

#without custom namespace
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}"

```

### 5. Controlling the vehicle with keyboard operation
```
#with custom namespace
cd CPSL_ROS2_Create3
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpslCreate3/cmd_vel

#without custom namespace
cd CPSL_ROS2_Create3
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### 6. Republish tf tree from create3
If you want to access the tf tree from the create3 on another server or device, you can run this simple node. It will republish the /tf topic on /forwarded_tf
```
cd CPSL_ROS2_Create3
source install/setup.bash
ros2 run tf_repub tf_repub
```
This will re-publish the tf tree from the RPI5 and make it available to any other edge devices that may be connected to it

### 7. Start all sensors with CPSL_ROS2_Sensors
If already configured, the sensors can be started using the following command:
```
cd CPSL_ROS2_Sensors
source install/setup.bash
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py lidar_enable:=true lidar_scan_enable:=true radar_enable:=true platform_description_enable:=true rviz:=false
```
The parameters that can be used here are as follows: 
| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `namespace`   | ''  | the namespace of the robot |  
| `lidar_enable`| true | on True, starts the livox lidar node
| `lidar_scan_enable`| false | on True, publishes a laserscan version of the livox's PC2 topic on /livox/lidar
| `radar_enable`| true | On True, launch the (front and back) TI radars
| `platform_description_enable`| true | On true, publishes the UGV robot description tf tree
| `rviz`| true | On True, displays an RViz window of sensor data

### 8. Start GNN ROS2 Nodes
If the CPSL_ROS2_PCProcessing package has already been setup, the following code will start the relevant Nodes

1. go into the CPSL_ROS2_PC_Processing directory
```
cd CPSL_ROS2_PCProcessing
```
2. activate the poetry shell
```
poetry shell
```
4. Source the setup.bash file
```
source install/setup.bash
```
5. Finally, launch the ugv_gnn_bringup file
```
ros2 launch pc_processing ugv_gnn_bringup.launch.py scan_enable:=true
```

When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`namespace`|''|The robot's namespace|
|`param_file`| 'ugv_gnn.yaml'|YAML file with parameters for the nodes in the configs directory|
|`model_state_dict`| 'Sage_10fp_20fh_0_50_th_5mRng_0_2_res.pth'|.pth config file in the model_state_dicts folder|
|`scan_enable`| 'false'|If enabled, additionally publish a /LaserScan message on the radar_combined/scan topic|

### 6. Starting SLAM Stack (radar)
Once sensors are running, the following command can start the SLAM pipeline
```
cd CPSL_ROS2_Nav
source install/setup.bash
ros2 launch cpsl_nav slam.launch.py namespace:=/cpslCreate3 scan_topic:=/livox/scan
```
When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`use_sim_time`|false|Use the time from a Gazebo simulation|
|`sync`|true|use synchronous SLAM (slower than asyncrhonous SLAM)|
|`namespace`|''|The robot's namespace|
|`scan_topic`|'/scan'|The LaserScan topic to use for slam|
|`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
|`use_lifecycle_manager`| false| Enable bond connection during node activation| 
|`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
|`rviz`|false|Display an RViz window with navigation|


### 7 Starting localization (radar)

```
cd CPSL_ROS2_Nav
source install/setup.bash
ros2 launch cpsl_nav localization.launch.py scan_topic:=/radar_combined/scan map:=wilk_radar.yaml param_file:=localization_radar.yaml
```

### 8. Starting navigation 
```
cd CPSL_ROS2_Nav
soruce install/setup.bash
ros2 launch cpsl_nav nav2_backup.launch.py scan_topic:=/radar_combined/scan
```
### 6.Generating a map
1. [terminal 1] Undock the robot, reset navigate to the location that you want the map to be centered at:

```
cd CPSL_ROS2_Create3
ros2 action send_goal /cpslCreate3/undock irobot_create_msgs/action/Undock "{}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpslCreate3/cmd_vel
```

- If desired, the pose can also be reset to a specific point so that the odom and map origins start out as the same point using the following command 
    ```
    {ros2 service call /cpslCreate3/reset_pose irobot_create_msgs/srv/ResetPose "pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}"}
    ```

2. [terminal 2] Launch all of the requisite sensors using the CPSL_ROS2_Sensors package
```
cd CPSL_ROS2_Sensors
source install/setup.bash
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py namespace:=/cpslCreate3 lidar_enable:=true lidar_scan_enable:=true radar_enable:=true platform_description_enable:=true rviz:=false
```
The parameters that can be used here are as follows: 
| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `namespace`   | ''  | the namespace of the robot |  
| `lidar_enable`| true | on True, starts the livox lidar node
| `lidar_scan_enable`| false | on True, publishes a laserscan version of the livox's PC2 topic on /livox/lidar
| `radar_enable`| true | On True, launch the (front and back) TI radars
| `platform_description_enable`| true | On true, publishes the UGV robot description tf tree
| `rviz`| true | On True, displays an RViz window of sensor data

3. [terminal 3] Finally, run the slam pipeline using the CPSL_ROS2_Nav packages
```
cd CPSL_ROS2_Nav
source install/setup.bash
ros2 launch cpsl_nav slam.launch.py namespace:=/cpslCreate3 scan_topic:=/livox/scan
```
When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`use_sim_time`|false|Use the time from a Gazebo simulation|
|`sync`|true|use synchronous SLAM (slower than asyncrhonous SLAM)|
|`namespace`|''|The robot's namespace|
|`scan_topic`|'/scan'|The LaserScan topic to use for slam|
|`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
|`use_lifecycle_manager`| false| Enable bond connection during node activation| 
|`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
|`rviz`|false|Display an RViz window with navigation|

4.  Once finished, use one of the following two methods to save the generated map
    - If rviz is displayed, go into the SlamToolboxPlugin Window, specify the file name (e.g.;"building_1") without the .yaml/.pgm. and slick the "Save Map" button. The file will be saved in the current directory

## Helpful Instructions

### 1. Running NoMachine in a headless (i.e.; without a monitor) mode
Note, the following instructions only need to be performed once. To run NoMachine in a headless mode, running the following commands on the server's terminal:
```
sudo systemctl stop gdm
sudo /etc/NX/nxserver --restart
```


## Helpful Documentation
- [Create 3 RPi4 Setup Guide](https://iroboteducation.github.io/create3_docs/setup/pi4humble/): Note that this guide is meant for RPi4 running Ubuntu 22.04, as its recommended to only use this as reference and instead follow the instructions above if you need help.
- [Create 3 Adapter Board Documentation ](https://iroboteducation.github.io/create3_docs/hw/adapter/)
- [Creat 3 NTP Setup](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/)
- [Create 3 Fast-DDS setup](https://iroboteducation.github.io/create3_docs/setup/xml-config/)
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)