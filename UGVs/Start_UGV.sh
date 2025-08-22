#!/bin/bash

SESSION_NAME="ROS"

# Check if the session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    echo "Creating new tmux ROS session..."

    # Create session and first pane (top-left)
    tmux new-session -d -s $SESSION_NAME

    # Create 3 more horizontal panes → 4 total in the top row
    tmux split-window -v -t $SESSION_NAME:0.0       # create bottom row
    tmux split-window -h -t $SESSION_NAME:0.0       # 0.2
    tmux split-window -h -t $SESSION_NAME:0.1       # 0.3
    tmux split-window -h -t $SESSION_NAME:0.2       # 0.4
    tmux split-window -v -t $SESSION_NAME:0.0       # 0.1 (0,0) vertically

    #Now split the bottom row
    tmux split-window -h -t $SESSION_NAME:0.5       # 0.6
    tmux split-window -h -t $SESSION_NAME:0.6       # 0.7
    tmux split-window -h -t $SESSION_NAME:0.7       # 0.8

    echo "Resetting UGV Pose..."
    tmux send-keys -t $SESSION_NAME:0.0 "ros2 service call /cpsl_ugv_1/reset_pose irobot_create_msgs/srv/ResetPose \"pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}\""    

    echo "Starting remaining Create3 functionality..."
    tmux send-keys -t $SESSION_NAME:0.1 "cd CPSL_ROS2_Create3 && source install/setup.bash && ros2 launch create3_bringup create3_bringup.launch.py namespace:=cpsl_ugv_1" C-m

    echo "Starting keyop controller..."
    tmux send-keys -t $SESSION_NAME:0.2 "cd CPSL_ROS2_Create3 && source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpsl_ugv_1/cmd_vel" C-m


    echo "Starting UGV sensors..."
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
