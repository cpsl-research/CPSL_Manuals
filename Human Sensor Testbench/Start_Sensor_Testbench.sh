#!/bin/bash

SESSION_NAME="ROS"
PLATFORM_NAME="cpsl_human_movement"

tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    echo "Creating new tmux ROS session..."

    # Create session and first pane (top-left)
    tmux new-session -d -s $SESSION_NAME

    #Create a vertical split
    tmux split-window -h -t $SESSION_NAME:0.0       # create bottom row
    tmux split-window -v -t $SESSION_NAME:0.0

    #setting up sensors
    echo "setting up UAV sensors..."
    tmux send-keys -t $SESSION_NAME:0.0 "cd CPSL_ROS2_Sensors" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "eval \$(poetry env activate)" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.0 "source install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "ros2 launch cpsl_ros2_sensors_bringup human_movement_sensor_bringup.launch.py namespace:=$PLATFORM_NAME"
    

    #setting up dataset collection
    echo "setting up dataset collection..."
    tmux send-keys -t $SESSION_NAME:0.2 "cd CPSL_ROS2_Sensors" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "eval \$(poetry env activate)" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.2 "source install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "ros2 launch dataset_generator record_dataset.launch.py param_file:=human_movement.yaml namespace:=$PLATFORM_NAME dataset_name:=FirstLast_exo_high"

    #setting up leap motion viewer
    echo "setting up leap motion viewer..."
    tmux send-keys -t $SESSION_NAME:0.1 "ultraleap-hand-tracking-control-panel" C-m
fi

echo "Attaching to tmux ROS session..."
tmux attach-session -t $SESSION_NAME
