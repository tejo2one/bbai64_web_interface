#!/bin/bash
# Autonomous Robot Teleoperation & Face HUD Startup Script

# Bring up Wi-Fi interface if not up
ip link set wlan0 up 2>/dev/null || true

# Source ROS2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Ensure executable permissions
chmod +x /opt/robot_teleop/web_teleop_server 2>/dev/null || true

# Wait up to 10 seconds for hardware devices to settle
for i in $(seq 1 10); do
    if [ -e /dev/fb0 ]; then
        break
    fi
    sleep 1
done

# Run web teleop & HUD server directly from /opt/robot_teleop
cd /opt/robot_teleop
exec ./web_teleop_server 5000
