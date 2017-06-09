#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


$DIR/make_new_ami.bash

# SRC-specific cleanup
sudo rm -rf $DIR/../options.json
sudo rm -rf $DIR/../server/keys.zip
sudo rm -rf $DIR/../../simulator

# Clean logs folder but do not remove it
cd /home/ubuntu/code/gazebo-logs
sudo rm -rf *

# Remove simulator-logs folder too
sudo rm -rf /home/ubuntu/code/simulator-logs
# Remove other files used by scripts
sudo rm -rf /home/ubuntu/code/record_gazebo_log.cfg
sudo rm -rf /home/ubuntu/code/enable_traffic_shaper.cfg

cd $DIR
# stop and remove docker containers
docker stop $(docker ps -a -q)
docker rm $(docker ps -a -q)
# Clean unused docker images
$DIR/../../srcsim_docker/docker/remove-unused-images.sh
echo "done"
