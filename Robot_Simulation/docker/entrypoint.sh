#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

if [ "$#" -gt 0 ]; then
    exec "$@"
fi

echo "[1/5] Starting virtual display..."
SIM_RESOLUTION="${SIM_RESOLUTION:-1280x720}"
GUI_FPS="${GUI_FPS:-20}"
if [[ ! "$SIM_RESOLUTION" =~ ^[1-9][0-9]{2,4}x[1-9][0-9]{2,4}$ ]]; then
    echo "SIM_RESOLUTION must be WIDTHxHEIGHT, for example 1280x720" >&2
    exit 1
fi
if [[ ! "$GUI_FPS" =~ ^[1-9][0-9]?$ ]] || (( GUI_FPS > 60 )); then
    echo "GUI_FPS must be an integer from 1 to 60" >&2
    exit 1
fi
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
Xvfb :1 -screen 0 "${SIM_RESOLUTION}x24" -ac +extension GLX &
sleep 2

echo "[2/5] Starting window manager..."
openbox &
sleep 1

echo "[3/5] Starting VNC and noVNC..."
x11vnc -display :1 -nopw -forever -shared -bg -quiet
websockify --web /usr/share/novnc/ --daemon 6080 localhost:5900

echo "[4/5] Starting Gazebo workcell..."
WORLD_FILE="${SIM_WORLD_FILE:-irb4600_workcell.sdf}"
ros2 launch abb_irb4600_gazebo simulation.launch.py "world_file:=${WORLD_FILE}" &
SIM_PID=$!

if [ "${SIM_GUI:-true}" = "true" ]; then
    sleep 15
    echo "[5/5] Starting Gazebo GUI at ${SIM_RESOLUTION}; target <= ${GUI_FPS} frames/s..."
    # Qt's basic render loop uses QWindow::requestUpdate. On Xvfb, its
    # supported update-idle timer avoids continuously drawing CPU-rendered
    # frames. This changes viewer scheduling only, never the physics clock.
    QSG_RENDER_LOOP=basic QT_QPA_UPDATE_IDLE_TIME="$(( (1000 + GUI_FPS - 1) / GUI_FPS ))" gz sim -g &
else
    echo "[5/5] Headless physics server; GUI disabled."
fi
echo "Gazebo desktop: http://localhost:6080/vnc.html"
echo "Run the builder with: docker compose -f docker/docker-compose.yml exec robot-sim bash"

wait "$SIM_PID"
