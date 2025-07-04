# iRobot Create 3 (Hardware Usage) User Manual

## - For Using iRobotCreate3 on GMKtec Nuc with ROS2 Jazzy on Ubuntu 24.04

The following steps can be used to control an iRobot Create3 with the following versions/software.

- ROS Version: ROS2 Jazzy
- Ubuntu Version: Ubuntu 24.04

## [Setup/Installation] with Nuc (ROS2 Jazzy)

### 1. Install and Setup Ubuntu 24.04

1. [If you are using a Raspberry Pi] the Raspberry Pi Imager (available here: [Raspberry Pi Imager](https://www.raspberrypi.com/software/)) to install Ubuntu 24.04 LTS Desktop onto the RPi 5, and follow the instructions to complete the installation

2. Once Ubuntu is installed, additionally install NoMachine by using the following instructions:
    - Go to [Nomachine Download](https://downloads.nomachine.com/), and click on the NoMachine for ARM (for Nuc) link. From there, click on the "NoMachine for ARMv8" "DEB" file which will work for Ubuntu.
    -  Follow the instructions on the download page (see here: [instructions](https://downloads.nomachine.com/download/?id=115&distro=ARM))
    - Finally, to run NoMachine in a headless (i.e. without a monitor) mode, see the instructions below under "Helpful Instructions"
    - Once NoMachine is installed on the Nuc, you must also install it on your local machine to access the remote display. NOTE: If you are located at another location, you must connect via the Duke VPN to access the Nuc remotely.

### 2. Setup Nuc Ethernet Interface

1. The create3 communicates with the Nuc using ethernet over USB. If you are powering the Nuc via the create3's USB-C port, please follow the following steps to activate the Nuc's ethernet over USB port:
    
    - open the /boot/firmware/config.txt file and add the following lines to the end of the file
    ```
    dtoverlay=dwc2,dr_mode=peripheral
    ```
    - Next, open the /boot/firmware/cmdline.txt file and add the following after ```rootwait```:
    ```
    modules-load=dwc2,g_ether
    ```
    - Finally, we will set up the USB ethernet port to work with the Nuc. To do so, open the /etc/netplan/50-cloud-init.yaml file and add the following text (correct indents as needed):
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
2. On the iRobot Create3, ensure that the USB/BLE toggle is set to "USB" and not BLE mode(see [Create 3 Adapter Board Documentation ](https://iroboteducation.github.io/create3_docs/hw/adapter/) for how to do this).
3. At this point, you should be able to go to ```192.168.186.2``` in a browser on the Nuc and see the consul. If this is successful, then the ethernet connection on the Nuc has successfully been connected to the Create3.

### 3. Setup Network Time Protocol for Create3
To setup NTP (network time protocol) on the compute board in order to synchronize time between the create3 and the Nuc. To do so, follow the instructions found at the [Create 3 NTP Setup Documentation](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/)
- Note that the create3 may not initially synchronize with the Nuc and that the ```sudo chronyc clients``` may not immediately return any clients.

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

    Once cloned, the following commands can be used to build/install the necessary packages

    ```
    cd CPSL_ROS2_Create3
    colcon build --symlink-install
    source install/setup.bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```

### 5. Installing additional ROS2 packages

If you want to support SLAM, Navigation, Sensing, and Pointcloud processing, use the following links to install (or clone) the following additional repositories:
1. [CPSL_ROS2_Sensors](https://github.com/cpsl-research/CPSL_ROS2_Sensors.git)
2. [CPSL_ROS2_Nav](https://github.com/cpsl-research/CPSL_ROS2_Nav)
3. [CPSL_ROS2_PCProcessing](https://github.com/cpsl-research/CPSL_ROS2_PCProcessing)

### 6. Setting ROS2 Middleware
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
## [Nominal Startup/Running] of Nuc + Create3:
Once everything is installed, the following steps can be followed to use the iRobotCreate3:

### 1. Connecting to the iRobotCreate3

1. Power/Reboot Nuc
2. Restart the Nomachine/GDM server to be able to access NoMachine, run the following commands on the server's terminal:
```
sudo systemctl stop gdm
sudo /etc/NX/nxserver --restart
```
3. Restart/check the NTP server
```
sudo service chrony restart
sudo chronyc clients
```
4. Open a browser on the Nuc and go to ```192.168.186.2``` (this is the interface for the create3)
5. Go to Application->Restart Application to restart the create3's application. This will take a few minutes, and the restart is complete after the robot makes a noise.
6. If everything went well, you should be able to confirm that the Create3 is connected via ROS2 by using the following command: ```ros2 node list```. If this command doesn't return anything, try the following solutions:
    - Log out and then back into the Nuc
    - Restart the NTP server again and wait until there is a confirmed connection with Create3 (should take a few minutes)

### 2. Undocking the robot:
To undock the robot, send one of the following action commands:

```
#(replace /cpsl_ugv_1 with the namespace of the UGV)
ros2 action send_goal /cpsl_ugv_1/undock irobot_create_msgs/action/Undock "{}"
```

### 3. Docking the robot:
Once done with the robot, send the following action command to redock the robot:
```
#(replace /cpsl_ugv_1 with the namespace of the UGV)
ros2 action send_goal /cpsl_ugv_1/dock irobot_create_msgs/action/Dock "{}"
```

### 4. Resetting the robot pose (say at a particular origin)
If you want to reset the pose to a specific location, you can use the following service
```
#(replace /cpsl_ugv_1 with the namespace of the UGV)
ros2 service call /cpsl_ugv_1/reset_pose irobot_create_msgs/srv/ResetPose "pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}"
```

### 5. Controlling the vehicle with keyboard operation
```
#(replace /cpsl_ugv_1 with the namespace of the UGV)
cd CPSL_ROS2_Create3
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpsl_ugv_1/cmd_vel
```
### 6. Bring up remaining Create3 functionality
To start the remaining UGV functionality, I've conveniently written a simple bringup package. It does things like re-publish the necessary tf tree information. To run this, run the following command. 
```
cd CPSL_ROS2_Create3
source install/setup.bash
ros2 launch create3_bringup create3_bringup.launch.py namespace:=cpsl_ugv_1
```
When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`namespace`|''|The robot's namespace|

### 7. Start all sensors with CPSL_ROS2_Sensors (radar disabled for now)
If already configured, the sensors can be started using the following command:
```
cd CPSL_ROS2_Sensors
source install/setup.bash
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py lidar_enable:=true lidar_scan_enable:=true camera_enable:=false radar_enable:=true platform_description_enable:=true rviz:=false namespace:=cpsl_ugv_1
```
The parameters that can be used here are as follows: 
| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `namespace`   | ''  | the namespace of the robot |  
| `camera_enable`| true | on True, starts the camera node
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
3. Source the setup.bash file
```
source install/setup.bash
```
4. Finally, launch the ugv_gnn_bringup file
```
ros2 launch pc_processing ugv_bringup.launch.py scan_enable:=true namespace:=cpsl_ugv_1
```

When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`namespace`|''|The robot's namespace|
|`param_file`| 'RaGNNarok_prob.yaml'|YAML file with parameters for the nodes in the configs directory|
|`model_state_dict`| 'Sage_10fp_20fh_0_50_th_5mRng_0_2_res.pth'|.pth config file in the model_state_dicts folder|
|`scan_enable`| 'false'|If enabled, additionally publish a /LaserScan message on the radar_combined/scan topic|

### 9. Starting SLAM Stack (radar)
Once sensors are running, the following steps can start the SLAM pipeline:

1. Before takeoff, use the steps at the end of this document to reset the ekf2 on the px4. When performing slam, it is important that mapping start at the origin (i.e.: UAV is as close as possible to 0,0).

2. Once the odometry has been reset, then type the following commands to start the slam stack
```
cd CPSL_ROS2_Nav
source install/setup.bash
ros2 launch cpsl_nav slam_sync.launch.py slam_params_file:=slam_radar.yaml scan_topic:=/radar_combined/scan namespace:=cpsl_ugv_1
```
When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`use_sim_time`|false|Use the time from a Gazebo simulation|
|`sync`|true|use synchronous SLAM (slower than asynchronous SLAM)|
|`namespace`|''|The robot's namespace|
|`scan_topic`|'/scan'|The LaserScan topic to use for slam (`/radar_combined/scan` for radar, `/livox/scan/` for lidar)|
|`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
|`use_lifecycle_manager`| false| Enable bond connection during node activation| 
|`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
|`rviz`|false|Display an RViz window with navigation|

Once finished, open Rviz and use the slam rviz configuration in the cpsl_ros2_nav2 package to view/save the map.
    - If rviz is displayed, go into the SlamToolboxPlugin Window, specify the file name (e.g.;"building_1") without the .yaml/.pgm. and click the "Save Map" button. The file will be saved in the current directory (CPSL_ROS2_Nav)

### 9. Starting SLAM Stack (lidar)
Once sensors are running, the following steps can start the SLAM pipeline:

1. Before takeoff, use the steps at the end of this document to reset the ekf2 on the px4. When performing slam, it is important that mapping start at the origin (i.e.: UAV is as close as possible to 0,0).

2. Once the odometry has been reset, then type the following commands to start the slam stack
```
cd CPSL_ROS2_Nav
source install/setup.bash
ros2 launch cpsl_nav slam_sync.launch.py slam_params_file:=slam_athena.yaml scan_topic:=/livox/scan namespace:=cpsl_ugv_1 base_frame_id:=base_link
```
When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`use_sim_time`|false|Use the time from a Gazebo simulation|
|`namespace`|''|The robot's namespace|
|`scan_topic`|'/scan'|The LaserScan topic to use for slam (`/radar_combined/scan` for radar, `/livox/scan/` for lidar)|
|`base_frame_id`|'base_link'|The frame ID of the base_link frame (without tf pre-fix)|
|`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
|`use_lifecycle_manager`| false| Enable bond connection during node activation| 
|`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
|`rviz`|false|Display an RViz window with navigation|

Once finished, open Rviz and use the slam rviz configuration in the cpsl_ros2_nav2 package to view/save the map.
    - If rviz is displayed, go into the SlamToolboxPlugin Window, specify the file name (e.g.;"building_1") without the .yaml/.pgm. and click the "Save Map" button. The file will be saved in the current directory (CPSL_ROS2_Nav)

To save the serialized map to a file, use the following code: 
```
ros2 service call /cpsl_ugv_1/slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph "{filename: 'uav_map'}"

#or
ros2 service call /cpsl_ugv_1/slam_toolbox/save_map slam_toolbox/SaveMap "{name: {data: 'cpsl_map'}}"
```

### 10. Starting localization (radar)
Instead of SLAM, you can run a localization pipeline and navigation (see next step). To start localization, Run the following steps:

1. Open RVIZ2 and use the either the nav_config.rviz or slam_config.rviz files in CPSL_ROS2_Nav/src/cpsl_nav/rviz_cfgs. This must be done first in order for the map to appear. When started, you will not see anything until the next step as the nav2 pipeline publishes the map->odom transformation.

2. Drive the UGV around to make sure that the point clouds are publishing correctly. 

3. Run the following commands. Change the map file to be the name of a map in the CPSL_ROS2_Nav/src/cpsl_nav/maps folder. 
```
cd CPSL_ROS2_Nav
source install/setup.bash
ros2 launch cpsl_nav localization.launch.py scan_topic:=/cpsl_ugv_1/radar_combined/scan map:=cpsl_radar.yaml param_file:=localization_radar.yaml
```
4. Once launched, you will then have to set a start location. Wait for the map to appear in the rviz window. Then use, rviz to specify the current location of the ground vehicle. Once this is done, everything in the rviz window should now appear. 

### 11. Starting navigation
Finally, to run navigation, run the following commands to start the navigation pipeline. 
```
cd CPSL_ROS2_Nav
soruce install/setup.bash
ros2 launch cpsl_nav nav2.launch.py namespace:=cpsl_ugv_1 params_file:=nav2_ugv.yaml
```

When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`use_sim_time`|false|Use the time from a Gazebo simulation|
|`namespace`|''|The robot's namespace|
|`params_file`| "nav2_ugv.yaml" | the .yaml config to use in the config folder
|`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
|`use_lifecycle_manager`| false| Enable bond connection during node activation| 
|`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
|`rviz`|false|Display an RViz window with navigation|

### 12. Collecting a dataset

For offline analysis or training, the following command can be used:
```
cd CPSL_ROS2_Sensors
source install/setup.bash
ros2 launch dataset_generator record_dataset.launch.py namespace:=cpsl_ugv_1 param_file:=ugv_dataset.yaml
```

The parameters that can be used by using the ```parameter:=value``` notation: 
| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `namespace`   | ''  | the namespace of the robot |  
| `param_file`| 'ugv_dataset.yaml' | the .yaml config file in the configs directory of the dataset_generator package.

## Helpful Instructions

### 1. Starting RViz on edge server
```
rviz2 --ros-args --remap /tf:=/forwarded_tf
```


## Helpful Documentation
- [Create 3 RPi4 Setup Guide](https://iroboteducation.github.io/create3_docs/setup/pi4humble/): Note that this guide is meant for RPi4 running Ubuntu 22.04, as it is recommended to only use this as a reference and instead follow the instructions above if you need help.
- [Create 3 Adapter Board Documentation ](https://iroboteducation.github.io/create3_docs/hw/adapter/)
- [Creat 3 NTP Setup](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/)
- [Create 3 Fast-DDS setup](https://iroboteducation.github.io/create3_docs/setup/xml-config/)
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
