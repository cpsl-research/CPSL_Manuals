#!/bin/bash

# Define the session name in one place
SESSION_NAME="ROS"

# Check if the session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null

# If the session does not exist, 'tmux has-session' returns a non-zero exit code.
if [ $? != 0 ]; then
    echo "Creating new tmux ROS session..."

    # Start a new detached session. All subsequent commands will target this session.
    tmux new-session -d -s $SESSION_NAME

    # --- Create the 2x3 Grid Layout ---
    # First, create the 3 rows
    # FIX: Use the correct variable $SESSION_NAME instead of $SESSION
    tmux split-window -v -t $SESSION_NAME:0.0
    tmux split-window -v -t $SESSION_NAME:0.1

    # Then, split each row to create the columns
    tmux split-window -h -t $SESSION_NAME:0.0
    tmux split-window -h -t $SESSION_NAME:0.1
    tmux split-window -h -t $SESSION_NAME:0.2

    # --- Send Commands to Panes ---
    # The pane numbering for this layout is:
    # 0.0 (Top-Left)     | 0.3 (Top-Right)
    # 0.1 (Middle-Left)  | 0.4 (Middle-Right)
    # 0.2 (Bottom-Left)  | 0.5 (Bottom-Right)

    # Pane 0.3 (Top-Right): Teleop Keyboard
    # IMPROVEMENT: Chain commands with '&&' to guarantee they run in order.
    tmux send-keys -t $SESSION_NAME:0.3 "cd CPSL_ROS2_Create3 && source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpsl_ugv_1/cmd_vel" C-m

    # Pane 0.1 (Middle-Left): Create 3 Bringup
    tmux send-keys -t $SESSION_NAME:0.1 "cd CPSL_ROS2_Create3 && source install/setup.bash && ros2 launch create3_bringup create3_bringup.launch.py namespace:=cpsl_ugv_1" C-m

    # Pane 0.4 (Middle-Right): Sensor Bringup
    # FIX: The entire command must be on a single line for send-keys.
    tmux send-keys -t $SESSION_NAME:0.4 "cd CPSL_ROS2_Sensors && source install/setup.bash && ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py lidar_enable:=true lidar_scan_enable:=true camera_enable:=false radar_enable:=true platform_description_enable:=true rviz:=false namespace:=cpsl_ugv_1" C-m

    # Pane 0.2 (Bottom-Left): Point Cloud Processing
    tmux send-keys -t $SESSION_NAME:0.2 "cd CPSL_ROS2_PCProcessing && poetry shell && source install/setup.bash && ros2 launch pc_processing ugv_bringup.launch.py scan_enable:=true namespace:=cpsl_ugv_1" C-m

    # --- Final Setup ---
    # SUGGESTION: You can use the empty panes for monitoring or general use.
    tmux send-keys -t $SESSION_NAME:0.0 "echo 'Main Shell'" C-m
    tmux send-keys -t $SESSION_NAME:0.5 "htop" C-m

    # Select the teleop pane so it's active when you attach
    tmux select-pane -t $SESSION_NAME:0.3
    
    # IMPROVEMENT: Removed the redundant attach-session from inside the 'if' block.
fi

# This command runs whether the session was just created or already existed.
echo "Attaching to tmux ROS session..."
tmux attach-session -t $SESSION_NAME
