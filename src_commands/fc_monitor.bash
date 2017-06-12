#!/bin/bash

codedir="/home/ubuntu/code"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $codedir/srcsim_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.2.1:11311

while true ; do
  # check if src_monitor is running
  monitor_id=`pgrep -f src_monitor`

  # check if ros master is available
  rostopic list > /dev/null 2>&1
  ros_master=`echo $?`

  # if src monitor script is running
  if [[ ! -z $monitor_id ]]
  then
    # and if we cannot find a ros master
    if [ $ros_master == 1 ]
    then
      echo "killing src monitor due to missing ROS master"
      sudo kill -9 `pgrep -f src_monitor`
      sudo kill -9 `pgrep src_monitor`
      echo "removing traffic shaper rules"
      sudo $DIR/src_tc_stop.rb -i tap0
    fi
  else
    # if there is no src script running but a ros master exists
    if [ $ros_master == 0 ]
    then
      echo "Starting src monitor"
      # launch script to monitor SRC tasks
      sudo kill -9 `pgrep -f src_monitor`
      sudo kill -9 `pgrep src_monitor`
      $DIR/src_monitor.bash &
    fi
  fi
  sleep 5
done