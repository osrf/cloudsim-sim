#!/usr/bin/env bash
WORLD_NAME=$1
dockerdir="/home/ubuntu/code/srcsim_docker/docker"
logsdir="/home/ubuntu/code/gazebo-logs/$WORLD_NAME"
echo 'uploading logs'
$dockerdir/run_s3bucket_container.bash s3bucket $logsdir
