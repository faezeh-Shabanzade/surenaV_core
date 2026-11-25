#!/bin/bash
# Switch Surena V between position and velocity control in Gazebo Classic.
#
# WHY A RELAUNCH: DefaultRobotHWSim registers each joint under the FIRST
# <hardwareInterface> tag of its transmission only. The position session has
# no velocity handles and vice versa, so switching modes requires respawning
# the robot with the other URDF variant. Within one session, use
# scripts/switch_control_mode.py (controller_manager/switch_controller) to
# move joints between individual and trajectory controllers.
#
# Usage:   ./switch_control_mode.sh position|velocity [gui:=false]
#
# This script kills the current Gazebo/ROS session started by the
# surenaV_control launch files and relaunches with the other variant.

set -e

MODE="${1:-}"
GUI="${2:-gui:=true}"
WS_DIR="${SURENA_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)}"

if [ "$MODE" != "position" ] && [ "$MODE" != "velocity" ]; then
    echo "Usage: $0 position|velocity [gui:=true|gui:=false]"
    exit 1
fi

echo "== Stopping any running Surena V session =="
# Graceful: stop controllers first so no command is mid-flight.
timeout 5 rosservice call /surenaV/controller_manager/switch_controller \
    "{}" "{}" 2>/dev/null || true
pkill -f gazebo_bridge_node      2>/dev/null || true
pkill -f controller_spawner      2>/dev/null || true
pkill -f gzserver                2>/dev/null || true
pkill -f gzclient                2>/dev/null || true
pkill -f roslaunch               2>/dev/null || true
pkill -f rosmaster               2>/dev/null || true
sleep 2

echo "== Starting $MODE control session =="
source "${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
if [ -f "$WS_DIR/devel/setup.bash" ]; then source "$WS_DIR/devel/setup.bash"; fi

roslaunch surenaV_control "gazebo_${MODE}.launch" "$GUI"
