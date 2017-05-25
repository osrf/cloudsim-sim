#!/usr/bin/env bash
# example run: ./run_gazebo.bash true
# WORLD_NAME=$1
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
dockerdir="/home/ubuntu/code/srcsim_docker/docker"
codedir="/home/ubuntu/code"
current="$(pwd)"
echo "running gazebo docker container"
echo $current

LOGGING=${1:-true}
if [ $LOGGING = true ]
then
LOG_PATH=/home/cloudsim/gazebo-logs/
ARGS="extra_gazebo_args:=\"-r --record_path $LOG_PATH\""
fi


cat <<DELIM > launch_server.bash
#!/usr/bin/env bash
# Note: we use the 'exec' to allow gzserver be the PID 1 in the docker container,
# and thus, be able to receive process signals.

# (example with log) roslaunch srcsim finals.launch final_number:=2 extra_gazebo_args:="-r --record_path ~/gazebo-logs/myworldlog"
source /opt/nasa/indigo/setup.bash
#GAZEBO_IP_WHITE_LIST=127.0.0.1 exec roslaunch srcsim unique.launch init:="true" $ARGS
GAZEBO_IP_WHITE_LIST=127.0.0.1 exec roslaunch srcsim finals.launch final_number:=$FINAL_NUMBER init:=true gui:=false $ARGS

# roslaunch srcsim finals.launch final_number:=5 gui:=true extra_gazebo_args:="-r --record_path /home/cloudsim/gazebo-logs/"

DELIM
chmod a+x launch_server.bash

# remove previous container run, if any
docker rm gazebo_run

# Note: look for docker logs in host machine at /var/lib/docker/

# launch script to monitor SRC tasks
kill -9 `pgrep -f src_monitor`
$DIR/src_monitor.bash |& tee -a $codedir/cloudsim-src-monitor.log &

$dockerdir/run_container.bash \
    gazebo_run \
    src-cloudsim \
    "-v $codedir/gazebo-logs:/home/cloudsim/gazebo-logs -v $current:/home/cloudsim/commands --net=host -it" \
    "bash" \
    |& tee -a $codedir/cloudsim-docker.log

#    "/home/cloudsim/commands/launch_server.bash" \
#    "-v $codedir/gazebo-logs:/home/cloudsim/gazebo-logs -v $current:/home/cloudsim/commands --net=host -e ROS_IP=192.168.2.1 -e ROS_MASTER_URI=http://192.168.2.1:11311" \
