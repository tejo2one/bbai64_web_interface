#!/bin/bash
# Stop old instances
killall -9 web_teleop_server 2>/dev/null || true
fuser -k 5000/tcp 2>/dev/null || true
killall -9 firefox firefox-esr 2>/dev/null || true
sleep 1

# Wake display and disable screen blanking
export DISPLAY=:0
xset s off -dpms 2>/dev/null || true
xset dpms force on 2>/dev/null || true

cd /home/debian/web_teleop

# Start C++ server in background
nohup ./web_teleop_server 5000 </dev/null >server.log 2>&1 &
SERVER_PID=$!
echo "Started web_teleop_server PID: $SERVER_PID"

# Wait for server to start responding
for i in {1..15}; do
  if curl -s http://127.0.0.1:5000/api/status >/dev/null 2>&1; then
    echo "Server listening on port 5000"
    break
  fi
  sleep 0.4
done

# Launch Firefox in full-screen kiosk on BBAI-64 physical display
nohup firefox --kiosk http://127.0.0.1:5000/face </dev/null >firefox.log 2>&1 &
FIREFOX_PID=$!
echo "Started Firefox Kiosk PID: $FIREFOX_PID on DISPLAY=:0"

sleep 2
ps aux | grep -E "web_teleop_server|firefox" | grep -v grep
