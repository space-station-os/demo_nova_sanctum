#!/bin/bash

# Session name
SESSION="ros_session"

# ROS 2 package
PACKAGE="demo_nova_sanctum"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2ws/install/setup.bash

# Check if tmux session already exists
tmux has-session -t $SESSION 2>/dev/null
if [ $? == 0 ]; then
    echo "Session $SESSION already exists. Attaching..."
    tmux attach -t $SESSION
    exit 0
fi

# Create a new tmux session
tmux new-session -d -s $SESSION -n "collector"

# Start the air_collector node in the first pane
tmux send-keys "source /opt/ros/humble/setup.bash && source ~/ros2ws/install/setup.bash && ros2 run $PACKAGE collector" C-m

# Create a new pane for desiccant_bed
tmux split-window -v -t $SESSION
tmux send-keys "source /opt/ros/humble/setup.bash && source ~/ros2ws/install/setup.bash && ros2 run $PACKAGE desiccant" C-m

# Create another pane for adsorbent_bed
tmux split-window -h -t $SESSION
tmux send-keys "source /opt/ros/humble/setup.bash && source ~/ros2ws/install/setup.bash && ros2 run $PACKAGE adsorbent" C-m

# Select the first pane for user interaction
tmux select-pane -t 0

# Attach to the session
tmux attach -t $SESSION
