#!/usr/bin/env bash
# AirMouse GCS Web Dashboard runner:
# Starts rosbridge_websocket (port 9090) and the web HTTP server (port 8080)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Source environment
source "$SCRIPT_DIR/env.sh"

echo "======================================================"
echo " Starting AirMouse GCS Web Dashboard"
echo " Web UI:    http://localhost:8080"
echo " WebSocket: ws://localhost:9090"
echo "======================================================"

exec ros2 launch airmouse dashboard.launch.py "$@"
