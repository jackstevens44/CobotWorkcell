#!/usr/bin/env bash
# Restart the myCobot dashboard server.
#
# Run this after editing any .py file (web_server.py, workcell.py,
# mycobot_kinematics.py, mycobot_driver.py, mycobot_280_m5_uart.py) or any
# constant in them (e.g. GRASP_DEPTH_CALIBRATION_M, SPEED_DESCEND, HOME_ANGLES).
# You do NOT need this for frontend edits (static/*) or scene edits in the UI —
# just reload the browser for those.
#
# Override the defaults if needed:
#   ROBOT_PORT=/dev/cu.usbserial-XXXX WEB_PORT=8768 BAUD=115200 ./restart_server.sh
#   FOREGROUND=1 ./restart_server.sh

set -u
PORT="${ROBOT_PORT:-/dev/cu.usbserial-5B090250681}"
WEB_PORT="${WEB_PORT:-8768}"
BAUD="${BAUD:-115200}"
PY="${PYTHON_BIN:-/Users/jackstevens/opt/anaconda3/bin/python3}"
FOREGROUND="${FOREGROUND:-0}"
cd "$(dirname "$0")"

if [ ! -x "$PY" ]; then
  echo "Python not found or not executable: $PY"
  exit 1
fi

echo "Stopping any running server..."
pkill -f "web_server.py --port" 2>/dev/null || true
# Also force-kill anything still holding the web port or serial device — covers
# orphaned/detached instances whose command line pkill didn't match.
for pid in $(lsof -nP -iTCP:"$WEB_PORT" -t 2>/dev/null) $(lsof -nP -t "$PORT" 2>/dev/null); do
  kill -9 "$pid" 2>/dev/null || true
done
# Wait until the web port is actually free before rebinding.
for _ in $(seq 1 25); do
  [ -z "$(lsof -nP -iTCP:"$WEB_PORT" -t 2>/dev/null)" ] && break
  sleep 0.2
done

echo "Starting server on :$WEB_PORT  (robot $PORT @ $BAUD)..."
if [ "$FOREGROUND" = "1" ]; then
  echo "Running in foreground. Press Ctrl-C to stop."
  exec "$PY" web_server.py --port "$PORT" --baud "$BAUD" --web-port "$WEB_PORT"
fi

nohup "$PY" web_server.py --port "$PORT" --baud "$BAUD" --web-port "$WEB_PORT" > /tmp/mycobot_web.log 2>&1 &
server_pid="$!"

# Wait for it to answer (pymycobot import can take a couple seconds).
for _ in $(seq 1 20); do
  if ! kill -0 "$server_pid" 2>/dev/null; then
    echo "Server process exited early — check /tmp/mycobot_web.log"
    tail -40 /tmp/mycobot_web.log 2>/dev/null || true
    exit 1
  fi
  if curl -s "http://127.0.0.1:$WEB_PORT/api/status" >/dev/null 2>&1; then
    echo "Running.  Logs: /tmp/mycobot_web.log"
    echo "PID:      $server_pid"
    echo "Open:     http://127.0.0.1:$WEB_PORT"
    exit 0
  fi
  sleep 0.5
done
echo "Did not come up in time — check /tmp/mycobot_web.log"
tail -40 /tmp/mycobot_web.log 2>/dev/null || true
exit 1
