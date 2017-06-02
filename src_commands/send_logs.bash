#!/usr/bin/env bash
# example run: ./send_logs.bash world_1_2015-05-05
# Parameters:
# world_name: a temporal world name used to identify the run. Tipically used as the name of the log folder.
set -e

WORLD_NAME=$1
s3dir="/home/ubuntu/s3"
logsdir="/home/ubuntu/code/simulator-logs/$WORLD_NAME"
s3mountdir="/mnt/s3bucket"
if [ -f $s3dir/bucketname-s3fs.txt ]
then
    echo 'uploading logs'
    mkdir -p $s3mountdir/$WORLD_NAME
    # gazebo state.log
    cp -r $logsdir/gazebo-logs $s3mountdir/$WORLD_NAME

    # gazebo server logs
    mkdir -p $s3mountdir/$WORLD_NAME/gazebo-server
    cp -r $logsdir/gazebo-server/diagnostics $s3mountdir/$WORLD_NAME/gazebo-server
    cp -r $logsdir/gazebo-server/server-11345 $s3mountdir/$WORLD_NAME/gazebo-server
    cp $logsdir/gazebo-server/ogre.log $s3mountdir/$WORLD_NAME/gazebo-server

    # ros logs
    cp -r $logsdir/ros $s3mountdir/$WORLD_NAME

    echo 'finished copying logs to s3 mnt folder'
else
    echo 'S3 is not configured. Cancel logs upload'
fi
