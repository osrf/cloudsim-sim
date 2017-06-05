#!/usr/bin/env bash

# Fork command for timeout handling
{
    # Stop the container after xx seconds
    sleep 60
    echo "TIMEOUT TIMEOUT TIMEOUT"
    docker stop gazebo_run
} &
timer_pid=$!
echo "timer pid: $timer_pid"

docker kill --signal "SIGINT" gazebo_run
echo "sent SIGINT signal to gazebo_run container for smooth shutdown"
echo "about to wait for gazebo_run container to stop"
docker wait gazebo_run

# The container stopped after sigint, so let's cancel the timeout fork
kill -SIGKILL $timer_pid

echo "gazebo_run container stopped"

echo "stopping src monitor script"
kill -9 `pgrep -f src_monitor`

exit 0