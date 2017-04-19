#!/usr/bin/env bash
# example run: ./run_gazebo.bash pioneer2dx
WORLD_NAME=$1
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
dockerdir="/home/ubuntu/code/srcsim_docker/docker"
codedir="/home/ubuntu/code"
current="$(pwd)"
echo "running gazebo docker container"
echo $current

cat <<DELIM > launch_server.bash
#!/usr/bin/env bash
# Note: we use the 'exec' to allow gzserver be the PID 1 in the docker container,
# and thus, be able to receive process signals.
#exec gzserver -r --record_path ~/gazebo-logs/$WORLD_NAME worlds/$WORLD_NAME.world
source /opt/nasa/indigo/setup.bash
roslaunch srcsim unique.launch init:="true"
DELIM
chmod a+x launch_server.bash

$dockerdir/run_container.bash \
    gazebo_run \
    src-cloudsim \
    "-v $codedir/gazebo-logs:/home/cloudsim/gazebo-logs -v $current:/home/cloudsim/commands" \
    "/home/cloudsim/commands/launch_server.bash"
