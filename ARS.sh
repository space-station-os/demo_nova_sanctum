#!/bin/bash

SESSION="iss_dashboard"

# Kill old session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Start a new tmux session
tmux new-session -d -s $SESSION

# Define relative path to setup.bash
SETUP_SCRIPT="../../install/setup.bash"

# 1. ARS Systems Launch
tmux new-window -t $SESSION:1 -n 'ARS_System'
tmux send-keys -t $SESSION:1 "source $SETUP_SCRIPT && ros2 launch demo_nova_sanctum ars_systems_v4.launch.py" C-m

# 2. STL Monitor Node
tmux new-window -t $SESSION:2 -n 'STL_Monitor'
tmux send-keys -t $SESSION:2 "source $SETUP_SCRIPT && ros2 run demo_nova_sanctum safety" C-m

# Give STL monitor time to initialize
sleep 10

# 3. ISS Simulator
tmux new-window -t $SESSION:3 -n 'Simulator'
tmux send-keys -t $SESSION:3 "source $SETUP_SCRIPT && ros2 run demo_nova_sanctum iss_simulator.py" C-m

# 4. Rosbridge Server
tmux new-window -t $SESSION:4 -n 'Rosbridge'
tmux send-keys -t $SESSION:4 "source $SETUP_SCRIPT && ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

# 5. Web Dashboard
tmux new-window -t $SESSION:5 -n 'Dashboard'
tmux send-keys -t $SESSION:5 "cd ../../src/demo_nova_sanctum/dashboard/dashboard-ui && npm run serve" C-m

# Attach to the session
tmux select-window -t $SESSION:1
tmux attach-session -t $SESSION
