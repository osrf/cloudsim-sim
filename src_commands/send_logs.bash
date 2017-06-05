#!/usr/bin/env bash
# example run: ./send_logs.bash world_1_2015-05-05
# Parameters:
# world_name: a temporal world name used to identify the run. Tipically used as the name of the log folder.
set -e

if [ ! -f $codedir/record_gazebo_log.cfg ]
then
    echo 'Logging to S3 disabled. Exiting...'
    exit
fi

WORLD_NAME=$1
s3dir="/home/ubuntu/s3"
logsdir="/home/ubuntu/code/gazebo-logs/$WORLD_NAME"
s3mountdir="/mnt/s3bucket"
if [ -f $s3dir/bucketname-s3fs.txt ]
then
    echo 'uploading logs'
    cp -r $logsdir $s3mountdir
    echo 'finished copying logs to s3 mnt folder'
else
    echo 'S3 is not configured. Cancel logs upload'
fi
