#!/bin/bash

SESSION="iss_dashboard"

# Kill old session if exists
tmux kill-session -t $SESSION 2>/dev/null

# Start new session
tmux new-session -d -s $SESSION

# 1. ARS Systems Launch
tmux new-window -t $SESSION:1 -n 'ARS_System'
tmux send-keys -t $SESSION:1 'source ~/ros2ws/install/setup.bash && ros2 launch demo_nova_sanctum ars_systems_v4.launch.py' C-m

# 2. STL Monitor Node
tmux new-window -t $SESSION:2 -n 'STL_Monitor'
tmux send-keys -t $SESSION:2 'source ~/ros2ws/install/setup.bash && ros2 run demo_nova_sanctum safety' C-m

# 3. ISS Simulator
tmux new-window -t $SESSION:3 -n 'Simulator'
tmux send-keys -t $SESSION:3 'source ~/ros2ws/install/setup.bash && ros2 run demo_nova_sanctum iss_simulator.py' C-m

# 4. Rosbridge Server
tmux new-window -t $SESSION:4 -n 'Rosbridge'
tmux send-keys -t $SESSION:4 'source ~/ros2ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml' C-m

# 5. Web Dashboard (npm run serve)
tmux new-window -t $SESSION:5 -n 'Dashboard'
tmux send-keys -t $SESSION:5 'cd ~/ros2ws/src/demo_nova_sanctum/dashboard/dashboard-ui && npm run serve' C-m

# Attach to the session
tmux select-window -t $SESSION:1
tmux attach-session -t $SESSION
#!/bin/bash

SESSION="iss_dashboard"

# Kill old session if exists
tmux kill-session -t $SESSION 2>/dev/null

# Start new session
tmux new-session -d -s $SESSION

# 1. ARS Systems Launch
tmux new-window -t $SESSION:1 -n 'ARS_System'
tmux send-keys -t $SESSION:1 'source ~/ros2ws/install/setup.bash && ros2 launch demo_nova_sanctum ars_systems_v4.launch.py' C-m

# 2. STL Monitor Node
tmux new-window -t $SESSION:2 -n 'STL_Monitor'
tmux send-keys -t $SESSION:2 'source ~/ros2ws/install/setup.bash && ros2 run demo_nova_sanctum safety' C-m

sleep(10)
# 3. ISS Simulator
tmux new-window -t $SESSION:3 -n 'Simulator'
tmux send-keys -t $SESSION:3 'source ~/ros2ws/install/setup.bash && ros2 run demo_nova_sanctum iss_simulator.py' C-m

# 4. Rosbridge Server
tmux new-window -t $SESSION:4 -n 'Rosbridge'
tmux send-keys -t $SESSION:4 'source ~/ros2ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml' C-m

# 5. Web Dashboard (npm run serve)
tmux new-window -t $SESSION:5 -n 'Dashboard'
tmux send-keys -t $SESSION:5 'cd ~/ros2ws/src/demo_nova_sanctum/dashboard/dashboard-ui && npm run serve' C-m

# Attach to the session
tmux select-window -t $SESSION:1
tmux attach-session -t $SESSION
