#!/usr/bin/env bash

# Fork command for timeout handling
{
    # Stop the container after 2 minutes
    sleep 120
    echo "TIMEOUT TIMEOUT TIMEOUT"
    docker stop team_container
} &
timer_pid=$!
echo "timer pid: $timer_pid"

docker kill --signal "SIGINT" team_container 
echo "sent SIGINT signal to 'team_container' container for smooth shutdown"
echo "about to wait for 'team_container' container to stop"
docker wait team_container
echo "team_container stopped"
# The container stopped after sigint, cancel the timeout fork
kill -SIGKILL $timer_pid
echo "exiting fc_stop_team_docker"

echo "stopping src monitor script"
kill -9 `pgrep -f src_monitor`

echo "removing traffic shaper rules"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
sudo $DIR/src_tc_stop.rb -i tap0 > /home/ubuntu/code/out 2>&1


exit 0
