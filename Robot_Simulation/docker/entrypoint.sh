#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

if [ "$#" -gt 0 ]; then
    exec "$@"
fi

echo "[1/5] Starting virtual display..."
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
Xvfb :1 -screen 0 1920x1080x24 -ac +extension GLX &
sleep 2

echo "[2/5] Starting window manager..."
openbox &
sleep 1

echo "[3/5] Starting VNC and noVNC..."
x11vnc -display :1 -nopw -forever -shared -bg -quiet
websockify --web /usr/share/novnc/ --daemon 6080 localhost:5900

echo "[4/5] Starting Gazebo and MoveIt..."
WORLD_FILE="${SIM_WORLD_FILE:-workcell.sdf}"
ros2 launch abb_irb120_gazebo gz_moveit.launch.py "world_file:=${WORLD_FILE}" &
SIM_PID=$!

sleep 15
echo "[5/5] Starting Gazebo GUI..."
gz sim -g &
echo "Gazebo/RViz desktop: http://localhost:6080/vnc.html"
echo "Run the builder with: docker compose -f docker/docker-compose.yml exec robot-sim bash"

wait "$SIM_PID"
