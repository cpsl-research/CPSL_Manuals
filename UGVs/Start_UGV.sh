#!/bin/bash

SESSION_NAME="ROS"

# Check if the session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    echo "Creating new tmux ROS session..."

    tmux new-session -d -s $SESSION_NAME

    tmux split-window -v -t $SESSION_NAME:0.0
    tmux split-window -v -t $SESSION_NAME:0.1

    tmux split-window -h -t $SESSION_NAME:0.0
    tmux split-window -h -t $SESSION_NAME:0.2
    tmux split-window -h -t $SESSION_NAME:0.4

    # 0.0 (Top-Left)     | 0.1 (Top-Right)
    # 0.2 (Middle-Left)  | 0.3 (Middle-Right)
    # 0.4 (Bottom-Left)  | 0.5 (Bottom-Right)


    tmux send-keys -t $SESSION_NAME:0.0 "ros2 service call /cpsl_ugv_1/reset_pose irobot_create_msgs/srv/ResetPose \"pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}\""

    tmux send-keys -t $SESSION_NAME:0.1 "cd CPSL_ROS2_Create3 && source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpsl_ugv_1/cmd_vel" C-m

    tmux send-keys -t $SESSION_NAME:0.2 "cd CPSL_ROS2_Create3 && source install/setup.bash && ros2 launch create3_bringup create3_bringup.launch.py namespace:=cpsl_ugv_1" C-m

    tmux send-keys -t $SESSION_NAME:0.3 "cd CPSL_ROS2_Sensors && source install/setup.bash && ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py lidar_enable:=true lidar_scan_enable:=true camera_enable:=false radar_enable:=true platform_description_enable:=true rviz:=false namespace:=cpsl_ugv_1" C-m

    tmux send-keys -t $SESSION_NAME:0.4 "cd CPSL_ROS2_PCProcessing && poetry shell" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.4 "source install/setup.bash" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.4 "ros2 launch pc_processing ugv_bringup.launch.py scan_enable:=true namespace:=cpsl_ugv_1" C-m

    tmux select-pane -t $SESSION_NAME:0.5
fi

echo "Attaching to tmux ROS session..."
tmux attach-session -t $SESSION_NAME
