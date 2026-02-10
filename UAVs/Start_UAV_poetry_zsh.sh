#!/bin/zsh

SESSION_NAME="ROS"
PLATFORM_NAME="cpsl_uav_6"

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


    # Optional: focus top-left
    # tmux select-pane -t $SESSION_NAME:0.0




    echo "Starting Micro XRCE Agent..."
    tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m #means to run the command

    echo "Starting PS4 Controller Nodes..."
    tmux send-keys -t $SESSION_NAME:0.1 "cd CPSL_ROS2_PX4" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "ros2 launch px4_controller joy_control_launch.py joy_enable:=true control_enable:=true namespace:=$PLATFORM_NAME" C-m

    echo "setting up UAV sensors..."
    tmux send-keys -t $SESSION_NAME:0.2 "cd CPSL_ROS2_Sensors" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "eval \$(poetry env activate)" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.2 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "nice -n -10 ros2 launch cpsl_ros2_sensors_bringup uav_sensor_bringup.launch.py lidar_enable:=false lidar_scan_enable:=true camera_enable:=false front_radar_enable:=true back_radar_enable:=true down_radar_enable:=true platform_description_enable:=true rviz:=false namespace:=$PLATFORM_NAME"

    echo "setting up flow estimation..."
    tmux send-keys -t $SESSION_NAME:0.3 "cd CPSL_ROS2_RadStack" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.3 "eval \$(poetry env activate)" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.3 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "ros2 launch mmwave_radar_processing_ros flow_estimator_points_bringup.launch.py namespace:=$PLATFORM_NAME"

    echo "setting up PC Processing (GNN)"
    tmux send-keys -t $SESSION_NAME:0.4 "cd CPSL_ROS2_PCProcessing" C-m
    tmux send-keys -t $SESSION_NAME:0.4 "eval \$(poetry env activate)" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.4 "source install/setup.zsh" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:0.4 "ros2 launch pc_processing ugv_bringup.launch.py scan_enable:=true namespace:=$PLATFORM_NAME" 
    
    echo "setting up SLAM (lidar for now)"
    tmux send-keys -t $SESSION_NAME:0.5 "cd CPSL_ROS2_Nav" C-m
    tmux send-keys -t $SESSION_NAME:0.5 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.5 "ros2 launch cpsl_nav slam_sync.launch.py slam_params_file:=slam_athena.yaml scan_topic:=/livox/scan namespace:=$PLATFORM_NAME base_frame_id:=base_footprint repub_map:=true"

    echo "setting up map saving"
    tmux send-keys -t $SESSION_NAME:0.6 "ros2 service call /$PLATFORM_NAME/slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: 'test'}}\""

    echo "setting up navigation (lidar)"
    tmux send-keys -t $SESSION_NAME:0.7 "cd CPSL_ROS2_Nav" C-m
    tmux send-keys -t $SESSION_NAME:0.7 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.7 "ros2 launch cpsl_nav nav2.launch.py namespace:=$PLATFORM_NAME params_file:=nav2_uav.yaml"

    echo "setting up dataset collection"
    tmux send-keys -t $SESSION_NAME:0.8 "cd CPSL_ROS2_Sensors" C-m
    tmux send-keys -t $SESSION_NAME:0.8 "source install/setup.zsh" C-m
    tmux send-keys -t $SESSION_NAME:0.8 "ros2 launch dataset_generator record_dataset.launch.py namespace:=$PLATFORM_NAME param_file:=uav_dataset_radvel.yaml"

    tmux select-pane -t $SESSION_NAME:0.5
fi

echo "Attaching to tmux ROS session..."
tmux attach-session -t $SESSION_NAME
