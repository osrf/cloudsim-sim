#!/usr/bin/env bash
# example run: ./run_gazebo.bash world_1_2015-05-05 1
# Parameters:
# world_name: a temporal world name used to identify this run. Tipically used as the name of the log folder.
# final_number: the SRC world number used in the finals (ie. a value 1..5).

WORLD_NAME=$1
FINAL_NUMBER=$2

team=`echo "$WORLD_NAME" |  grep -o 'SRC-[^-]*' | tr '[:upper:]' '[:lower:]'`

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
dockerdir="/home/ubuntu/code/srcsim_docker/docker"
codedir="/home/ubuntu/code"
current="$(pwd)"
echo "running gazebo docker container"
echo $current

if [ -f $codedir/record_gazebo_log.cfg ]
then
  LOG_PATH=/home/cloudsim/gazebo-logs/$WORLD_NAME
  ARGS="extra_gazebo_args:=\"-r --record_path $LOG_PATH\""
  echo "gazebo logging enabled"
fi

mkdir -p $codedir/simulator-logs/$WORLD_NAME/gazebo-logs
mkdir -p $codedir/simulator-logs/$WORLD_NAME/ros
chown -R ubuntu:ubuntu $codedir/simulator-logs

cat <<DELIM > launch_server.bash
#!/usr/bin/env bash
# Note: we use the 'exec' to allow gzserver be the PID 1 in the docker container,
# and thus, be able to receive process signals.

# (example with log) roslaunch srcsim finals.launch final_number:=2 extra_gazebo_args:="-r --record_path ~/gazebo-logs/myworldlog"
# source /opt/nasa/indigo/setup.bash
GAZEBO_IP_WHITE_LIST=127.0.0.1 exec roslaunch srcsim_finals final.launch team_name:=$team final_number:=$FINAL_NUMBER init:=true gui:=false grasping_init_wait_time:=70 $ARGS
DELIM
chmod a+x launch_server.bash

# remove previous container run, if any
docker rm gazebo_run

# Note: look for docker logs in host machine at /var/lib/docker/

# launch script to monitor SRC tasks
kill -9 `pgrep -f src_monitor`
$DIR/src_monitor.bash $FINAL_NUMBER $WORLD_NAME |& tee -a $codedir/cloudsim-src-monitor.log &

$dockerdir/run_container.bash \
    gazebo_run \
    src-cloudsim \
    "-v $codedir/simulator-logs/$WORLD_NAME/gazebo-logs:/home/cloudsim/gazebo-logs -v $codedir/simulator-logs/$WORLD_NAME/ros:/home/cloudsim/.ros -v $current:/home/cloudsim/commands --net=host -e ROS_IP=192.168.2.1 -e ROS_MASTER_URI=http://192.168.2.1:11311 --ulimit core=1000000000:1000000000" \
    "/home/cloudsim/commands/launch_server.bash" \
    |& tee -a $codedir/cloudsim-docker.log

