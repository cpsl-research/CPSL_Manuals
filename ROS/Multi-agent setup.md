# ROS 2 Multi-Machine Setup Guide

This guide explains how to configure multiple machines to communicate with each other in a ROS 2 (e.g., Jazzy) environment. These steps are essential when working with distributed robotic systems such as fleets of robots, UAVs, or edge servers communicating over a shared network.

## Prerequisites

- All machines are on the same local network or VPN.
- ROS 2 (e.g., Jazzy) is installed on all machines.
- Time synchronization is handled across machines (e.g., using `chrony` or `ntp`).
- Firewalls are configured to allow ROS 2 DDS traffic (typically ports UDP 7400–7600).

---

## Step 1: Set Hostname-to-IP Mappings

To enable name-based communication instead of using raw IP addresses (which may change or be harder to manage), update the `/etc/hosts` file on **every machine** involved in the ROS 2 network.

```
sudo vim /etc/hosts
```

Add the IP and hostname entries for all participating machines, for example (be sure to include your machine as well):
```
172.28.158.83   machine_1
10.237.199.29   machine_2
```

## Step 2: Configure your ROS2 Environment
Next, ensure that your ROS2 environment is set up correctly by performing the following steps
1. Open your .bashrc file
```
cd
vim .bashrc
```

2. Next, ensure that you have the following in your .bashrc file
```
# Source ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Set the same domain ID across all machines
export ROS_DOMAIN_ID=0

# Recommended: limit discovery to localhost only (for debugging)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# If using localhost: define a list of static peer hostnames (alternative to full multicast discovery)
# export ROS_STATIC_PEERS="machine_1;machine_2"

# Source colcon_cd for ROS workspace navigation
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/jazzy/
```

3. Finally, after editing, reload the .bashrc
```
source ~/.bashrc
```

## Optional Notes
* ``ROS_DOMAIN_ID``: Ensures all agents are in the same communication group. If you are running multiple isolated systems on the same network, assign them different IDs.

* ``ROS_STATIC_PEERS``: You can uncomment and use this variable to avoid relying on multicast discovery. This is particularly useful for deterministic deployments or on networks where multicast is restricted.

* ``Time Sync``: ROS 2 requires all agents to have synchronized clocks for proper message timestamping and coordination. Use ntp or chrony.

* ``Firewall Rules``: You may need to open ports for DDS discovery and communication. Typical UDP port range: 7400–7600.

* ``ROS versions``: Finally, we recommend that you have all machines on the same ROS verion. While this may work across different ROS versions, consistency will likely result in the best performance