#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


$DIR/make_new_ami.bash

# SRC-specific cleanup
sudo rm -rf $DIR/../options.json
sudo rm -rf $DIR/../server/keys.zip
sudo rm -rf $DIR/../../vpn
sudo rm -rf $DIR/../../simulator
# Clean logs folder but do not remove it
cd /home/ubuntu/code/gazebo-logs
sudo rm -rf *
cd $DIR
# Clean docker images
$DIR/../../srcsim_docker/docker/remove-unused-images.sh
echo "done"
