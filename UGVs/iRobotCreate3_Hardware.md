# iRobot Create 3 (Hardware Usage) User Manual

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
To undock the robot, send the following action command:

```
ros2 action send_goal /cpslCreate3/undock irobot_create_msgs/action/Undock "{}"
```

### 3. Docking the robot:
Once donce with the robot, send the following action command to redock the robot:
```
ros2 action send_goal /cpslCreate3/dock irobot_create_msgs/action/Dock "{}"
```

### 4. Controlling the vehicle with keyboard operation
```
cd CPSL_ROS2_Create3
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpslCreate3/cmd_vel
```

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