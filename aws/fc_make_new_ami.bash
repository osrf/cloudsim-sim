#!/usr/bin/env bash

# This script is used to make a new Field Computer ami from an existing one:
# - removes ssh keys
# - removes all docker containers and images

if [ "$USER" != "ubuntu" ]; then
  echo 'Please run as user "ubuntu"'
  exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sudo shred -u /etc/ssh/*_key /etc/ssh/*_key.pub
sudo shred -u ~/.*history
sudo shred -u /root/.ssh/authorized_keys
sudo shred -u /home/ubuntu/.ssh/authorized_keys

cd $DIR/..
sudo rm -f .env
sudo rm -f ../cloudsim-env.bash
sudo rm -f ../cloudsim.log
sudo rm -f ../cloudsim-options.json

sudo rm -f /var/log/cloud-init-output.log
sudo rm -f /var/log/cloud-init.log

# reinstall cloudsim-sim
sudo rm -rf node_modules
sudo npm install

# empty database
redis-cli flushdb

# print db content
redis-cli lrange cloudsim-sim 0 -1

# stop and remove docker containers
docker stop $(docker ps -a -q)
docker rm $(docker ps -a -q)
# Delete all images
docker rmi $(docker images -q)
