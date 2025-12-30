#!/bin/bash
trap "kill 0" EXIT

echo "=========================================="
echo " Qt Cart System Launching... "
echo "=========================================="

# API 서버 실행 (백그라운드 &)
echo "Starting FastAPI Server (Port 8000)..."
python3 main.py &

sleep 2

# ROS 컨트롤러 실행 (백그라운드 &)
echo "Starting ROS Controller (UDP 55555/44444)..."
python3 ros_controller.py &

wait