#!/usr/bin/env bash
set -e

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
    echo 'S3 is not configured. Cancel log uploading'
fi
