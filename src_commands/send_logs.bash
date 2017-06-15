#!/usr/bin/env bash
# example run: ./send_logs.bash world_1_2015-05-05
# Parameters:
# world_name: a temporal world name used to identify the run. Tipically used as the name of the log folder.

codedir="/home/ubuntu/code"
if [ ! -f $codedir/record_gazebo_log.cfg ]
then
    echo 'Logging to S3 disabled. Exiting...'
    exit
fi

WORLD_NAME=$1
s3dir="/home/ubuntu/s3"
logsdir="/home/ubuntu/code/simulator-logs/$WORLD_NAME"
s3mountdir="/mnt/s3bucket"
if [ -f $s3dir/bucketname-s3fs.txt ]
then
    echo 'uploading logs'

    s3bucket=`cat $s3dir/bucketname-s3fs.txt`
    echo "s3bucket $s3bucket"

    mkdir -p $s3mountdir/$WORLD_NAME

    /usr/bin/s3fs -o createbucket -o use_cache=/tmp $s3bucket:/$WORLD_NAME $s3mountdir/$WORLD_NAME 

    # gazebo state.log
    cp -r $logsdir/gazebo-logs $s3mountdir/$WORLD_NAME

    # ros logs
    cp -r $logsdir/ros $s3mountdir/$WORLD_NAME

    # src-monitor and docker logs
    cp /home/ubuntu/code/cloudsim-src-monitor.log $s3mountdir/$WORLD_NAME
    cp /home/ubuntu/code/cloudsim-docker.log $s3mountdir/$WORLD_NAME

    echo 'finished copying logs to s3 mnt folder'
else
    echo 'S3 is not configured. Cancel logs upload'
fi
