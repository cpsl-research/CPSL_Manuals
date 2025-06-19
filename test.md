### Tutorials for installing and running isaac sim with nvidia

## Starting the Docker container

1. Start the isaac sim container using the following command
```
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    nvcr.io/nvidia/isaac-sim:4.5.0
```

2. To start 

## ROS Tutorials

### Installing ROS2:
Majority of instructions taken from here [isaac sim ros installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html#running-ros-in-docker-containers): 

1. Install Rocker: see instructions here: [rocker github](https://github.com/osrf/rocker)
    ```
    sudo apt-get install python3-rocker
    ```
2. Start ROS container using Rocker (replace CONTAINER_NAME with the desired container name):
    ```
    rocker --nvidia --x11 --privileged --network host  --name CONTAINER_NAME osrf/ros:humble-desktop-full-jammy
    ```

3. Clone the docker workspace onto the machine
    ```
    git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces
    ```

### Helpful links

* [Isaac sim w/ ROS guides](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/ros2_landing_page.html)