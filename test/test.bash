#!/bin/bash
# SPDX-FileCopyrightText: 2025 hakozaki teruki
# SPDX-License-Identifier: BSD-3-Clause

set -e

echo "[TEST] build workspace"

cd ~/ros2_ws
colcon build --symlink-install > /dev/null
source install/setup.bash

echo "[TEST] start system_health_monitor"

ros2 run mypkg system_health_monitor > /tmp/monitor.log 2>&1 &
MON_PID=$!

sleep 2

echo "[TEST] start system_health_listener"

LOGFILE=/tmp/system_health_test.csv
rm -f $LOGFILE

ros2 run mypkg system_health_listener \
  --ros-args -p log_path:=$LOGFILE > /tmp/listener.log 2>&1 &
LIS_PID=$!

sleep 5

echo "[TEST] stop monitor to simulate offline"
kill $MON_PID

sleep 5

echo "[TEST] check listener exited"
if ps -p $LIS_PID > /dev/null; then
    echo "[FAIL] listener still running"
    exit 1
fi

echo "[TEST] check CSV log exists"
if [ ! -f $LOGFILE ]; then
    echo "[FAIL] CSV log not created"
    exit 1
fi

echo "[TEST] check CSV content"
LINES=$(wc -l < $LOGFILE)
if [ "$LINES" -lt 2 ]; then
    echo "[FAIL] CSV log has no data"
    exit 1
fi

echo "[PASS] system health monitor test succeeded"
exit 0
