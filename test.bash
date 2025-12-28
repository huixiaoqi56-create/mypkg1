#!/bin/bash
# SPDX-FileCopyrightText: 2025 hakozaki teruki
# SPDX-License-Identifier: BSD-3-Clause

set -e

echo "[TEST] start system health test"

WS_DIR="$HOME/ros2_ws"
LOG_DIR="/tmp"
CSV_LOG="$LOG_DIR/system_health_test.csv"

# 事前確認
if [ ! -d "$WS_DIR" ]; then
    echo "[FAIL] workspace not found: $WS_DIR"
    exit 1
fi

cd "$WS_DIR"

echo "[TEST] build workspace"
colcon build --symlink-install > /dev/null

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "[TEST] clean old log"
rm -f "$CSV_LOG"

echo "[TEST] start system_health_monitor"
ros2 run mypkg system_health_monitor \
    > "$LOG_DIR/monitor.log" 2>&1 &
MON_PID=$!

sleep 2

echo "[TEST] start system_health_listener"
ros2 run mypkg system_health_listener \
    --ros-args -p log_path:="$CSV_LOG" \
    > "$LOG_DIR/listener.log" 2>&1 &
LIS_PID=$!

sleep 5

echo "[TEST] stop monitor to simulate offline"
kill "$MON_PID"

sleep 5

echo "[TEST] check listener exited"
if ps -p "$LIS_PID" > /dev/null; then
    echo "[FAIL] listener is still running"
    exit 1
fi

echo "[TEST] check CSV log created"
if [ ! -f "$CSV_LOG" ]; then
    echo "[FAIL] CSV log not found"
    exit 1
fi

echo "[TEST] check CSV log content"
LINE_COUNT=$(wc -l < "$CSV_LOG")
if [ "$LINE_COUNT" -lt 2 ]; then
    echo "[FAIL] CSV log has no data"
    exit 1
fi

echo "[PASS] system health test succeeded"
exit 0
