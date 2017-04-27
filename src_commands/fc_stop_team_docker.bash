#!/usr/bin/env bash

# Fork command for timeout handling
{
    # Stop the container after 5 minutes
    sleep 300
    echo "TIMEOUT TIMEOUT TIMEOUT"
    docker stop team_container
} &
timer_pid=$!

docker kill --signal "SIGINT" team_container 
echo "sent SIGINT signal to 'team_container' container for smooth shutdown"
echo "about to wait for 'team_container' container to stop"
docker wait team_container
echo "team_container stopped"
# The container stopped after sigint, cancel the timeout fork
kill -SIGKILL $timer_pid
