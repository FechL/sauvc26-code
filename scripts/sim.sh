#!/bin/zsh

# SAUVC Simulation Launcher Script
# This script launches Gazebo, ArduPilot, and ROS2 MAVROS in parallel

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory and set log directory relative to sauvc26-code
SCRIPT_DIR="$( cd "$( dirname "${(%):-%x}" )" && pwd )"
LOG_DIR="$SCRIPT_DIR/../logs/sim_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

echo -e "${GREEN}Starting SAUVC Simulation...${NC}"
echo -e "${YELLOW}Logs will be saved to: $LOG_DIR${NC}"

# Array to store background process PIDs
PIDS=()

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Shutting down all processes...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Killing process $pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    wait
    echo -e "${GREEN}All processes terminated.${NC}"
    exit 0
}

# Set up trap to catch Ctrl+C and other termination signals
trap cleanup SIGINT SIGTERM EXIT

# 1. Launch ArduPilot FIRST (must be ready before Gazebo connects)
echo -e "${GREEN}[1/3] Launching ArduPilot SITL...${NC}"
cd "$HOME/ardupilot"
Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub -f vectored --model=JSON --out=udp:0.0.0.0:14550 --console > "$LOG_DIR/ardupilot.log" 2>&1 &
PIDS+=($!)
echo "  PID: ${PIDS[-1]} | Log: $LOG_DIR/ardupilot.log"
echo -e "${YELLOW}  Waiting for ArduPilot to fully initialize...${NC}"
sleep 10

# 2. Launch Gazebo (after ArduPilot is ready)
echo -e "${GREEN}[2/3] Launching Gazebo Simulator...${NC}"
gz sim -v 3 -r sauvc_final.world > "$LOG_DIR/gazebo.log" 2>&1 &
PIDS+=($!)
echo "  PID: ${PIDS[-1]} | Log: $LOG_DIR/gazebo.log"
sleep 5

# 3. Launch ROS2 MAVROS
echo -e "${GREEN}[3/3] Launching ROS2 MAVROS...${NC}"
# cd "$HOME/ros2_ws"
# source install/setup.zsh
ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost:14555 > "$LOG_DIR/mavros.log" 2>&1 &
PIDS+=($!)
echo "  PID: ${PIDS[-1]} | Log: $LOG_DIR/mavros.log"

echo -e "\n${GREEN}All processes launched successfully!${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
echo -e "\nProcess IDs:"
echo "  ArduPilot: ${PIDS[0]}"
echo "  Gazebo:    ${PIDS[1]}"
echo "  MAVROS:    ${PIDS[2]}"

# Wait for all background processes
wait
