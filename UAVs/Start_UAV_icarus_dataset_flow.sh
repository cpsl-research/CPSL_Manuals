#!/bin/zsh

SESSION_NAME="ROS"
PLATFORM_NAME="cpsl_uav_8"

tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    echo "Creating new tmux ROS session..."

    # Create session and first pane (top-left)
    tmux new-session -d -s $SESSION_NAME

    # Create 3 more horizontal panes → 4 total in the top row
    tmux split-window -v -t $SESSION_NAME:0.0       # create bottom row
    tmux split-window -h -t $SESSION_NAME:0.0       # 0.2
    tmux split-window -v -t $SESSION_NAME:0.0       # 0.1 (0,0) vertically

    #Now split the bottom row
    tmux split-window -h -t $SESSION_NAME:0.3       # 0.4


    # Top left
    # tmux select-pane -t $SESSION_NAME:0.0
    echo "Starting Micro XRCE Agent..."
    tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m #means to run the command

    #top left (below the above)
    echo "Starting PS4 Controller Nodes..."
    tmux send-keys -t $SESSION_NAME:0.1 "cd CPSL_ROS2_PX4" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "ros2 launch px4_controller joy_control_launch.py joy_enable:=true control_enable:=true namespace:=$PLATFORM_NAME" C-m

    echo "setting up UAV sensors..."
    tmux send-keys -t $SESSION_NAME:0.2 "cd CPSL_ROS2_Sensors" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "eval \$(poetry env activate)" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.2 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "nice -n -10 ros2 launch cpsl_ros2_sensors_bringup uav_sensor_bringup_IcaRAus.launch.py lidar_enable:=false lidar_scan_enable:=false camera_enable:=false front_radar_enable:=true back_radar_enable:=true down_radar_enable:=true platform_description_enable:=true rviz:=false namespace:=$PLATFORM_NAME vicon_enable:=true" C-m
    sleep 10
    echo "sensors setup complete"

    echo "setting up point cloud combining"
    tmux send-keys -t $SESSION_NAME:0.3 "cd CPSL_ROS2_PCProcessing" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "eval \$(poetry env activate)" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.3 "source install/setup.zsh" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.3 "ros2 launch pc_processing uav_pc_combiner_bringup.launch.py namespace:=$PLATFORM_NAME" C-m
    sleep 10
    echo "point cloud combining setup complete"
    

    # echo "setting up flow estimation..."
    # tmux send-keys -t $SESSION_NAME:0.3 "cd CPSL_ROS2_RadStack" C-m
    # sleep 1
    # tmux send-keys -t $SESSION_NAME:0.3 "eval \$(poetry env activate)" C-m
    # sleep 1
    # tmux send-keys -t $SESSION_NAME:0.3 "source install/setup.zsh" C-m
    # tmux send-keys -t $SESSION_NAME:0.3 "ros2 launch mmwave_radar_processing_ros flow_estimator_ods_bringup.launch.py namespace:=$PLATFORM_NAME" C-m
    # sleep 20
    # echo "flow estimation setup complete"

    echo "setting up dataset collection"
    tmux send-keys -t $SESSION_NAME:0.4 "cd CPSL_ROS2_Sensors" C-m
    tmux send-keys -t $SESSION_NAME:0.4 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.4 "ros2 launch dataset_generator record_dataset.launch.py namespace:=$PLATFORM_NAME param_file:=uav_dataset_icaraus_dataset_flow.yaml dataset_subpath:=vicon_box_1"

    tmux select-pane -t $SESSION_NAME:0.4
fi

echo "Attaching to tmux ROS session..."
tmux attach-session -t $SESSION_NAME
