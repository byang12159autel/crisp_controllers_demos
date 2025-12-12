#!/bin/bash

# Configuration

# Detect network interface based on hostname
# Find your device's network interface via "ip link show" for active network interface. 
HOSTNAME=$(hostname)
case "$HOSTNAME" in
    "laptop1")
        ROS_NETWORK_INTERFACE="wlo1"
        ;;
    "autel-ben-sim")
        ROS_NETWORK_INTERFACE="wlp15s0"
        ;;
    *)
        echo "Warning: Unknown hostname"
        echo "To configure your interface, add your hostname to this script's case statement"
        ;;
esac

echo "Using network interface: $ROS_NETWORK_INTERFACE (hostname: $HOSTNAME)"
CONTAINER_NAME="crisp_controllers_demos_launch_franka"
SESSION_NAME="franka_launch"

# Cleanup function
cleanup() {
    echo "Cleaning up docker containers..."
    docker compose down
}

# Set trap to cleanup on script exit
trap cleanup EXIT INT TERM

# Clean up any existing containers from previous runs
docker compose down

# Kill existing session if it exists
tmux kill-session -t "$SESSION_NAME" 2>/dev/null

# Create a new tmux session (detached)
tmux new-session -d -s "$SESSION_NAME"

# Split window vertically (side by side)
tmux split-window -h -t "$SESSION_NAME"

# Split the right pane horizontally (top and bottom)
tmux split-window -v -t "$SESSION_NAME:0.1"

# Send command to left pane (pane 0)
tmux send-keys -t "$SESSION_NAME:0.0" "ROBOT_IP=172.16.0.2 FRANKA_FAKE_HARDWARE=true RMW=cyclone ROS_NETWORK_INTERFACE=$ROS_NETWORK_INTERFACE docker compose up launch_franka" C-m

# Send command to top-right pane (pane 1) - with delay and ROS environment setup
tmux send-keys -t "$SESSION_NAME:0.1" "echo 'Waiting for container to start...'; sleep 8; docker exec -it $CONTAINER_NAME bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
export ROS_DOMAIN_ID=100 && exec bash' || bash" C-m

# Send commands to bottom-right pane (pane 2) - crisp_py environment
tmux send-keys -t "$SESSION_NAME:0.2" "cd ~/crisp_py" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "ros2 daemon stop" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "ros2 daemon start" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "pixi shell -e humble" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "export ROS_DOMAIN_ID=100" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "ros2 topic list" C-m

# Attach to the session
tmux attach-session -t "$SESSION_NAME"

# ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
# joint_names:
# - fr3_joint1
# - fr3_joint2
# - fr3_joint3
# - fr3_joint4
# - fr3_joint5
# - fr3_joint6
# - fr3_joint7
# points:
# - positions: [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5]
#   velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   time_from_start:
#     sec: 3
#     nanosec: 0
# "

# Simple Demo
# terminal 1
#  ROBOT_IP=172.16.0.2 FRANKA_FAKE_HARDWARE=true RMW=cyclone ROS_NETWORK_INTERFACE=wlp15s0 docker compose up launch_franka
# docker stop crisp_controllers_demos_launch_franka && docker rm crisp_controllers_demos_launch_franka

# # terminal 2
# cd ~/crisp_py
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export ROS_DOMAIN_ID=100
# ros2 daemon stop
# ros2 daemon start
# pixi shell -e humble
# ros2 topic list

# python examples/01_figure_eight.py
