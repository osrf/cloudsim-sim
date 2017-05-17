#!/usr/bin/env bash

# This script is used to make a new ami from an existing one:
# - removes ssh keys
# - deletes the redis database
# - reinstalls cloudsim-sim software (but does not get the latest version)


if [ "$USER" != "ubuntu" ]; then
  echo 'Please run as user "ubuntu"'
  exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sudo shred -u /etc/ssh/*_key /etc/ssh/*_key.pub
sudo shred -u ~/.*history
sudo shred -u /root/.ssh/authorized_keys
sudo shred -u /home/ubuntu/.ssh/authorized_keys

# remove ros logs
sudo rm -fr /home/ubuntu/.ros
sudo rm -fr /root/.ros

# remove old env:
cd $DIR/..
sudo rm -f .env
sudo rm -f options.json
sudo rm -f ../cloudsim-env.bash
sudo rm -f ../cloudsim.log
sudo rm -f ../cloudsim-options.json
sudo rm -f ../cloudsim-docker.log
sudo rm -f ../cloudsim-src-monitor.log

sudo rm -f /var/log/cloud-init-output.log
sudo rm -f /var/log/cloud-init.log

# S3 related
sudo rm -rf ~/s3
sudo rm -rf /mnt/s3bucket
sudo rm -rf /etc/passwd-s3fs
sudo rm -rf /home/ubuntu/.passwd-s3fs
sudo rm -rf /home/ubuntu/.bucketname-s3fs

# reinstall cloudsim-sim
sudo rm -rf node_modules
sudo npm install

# clear pm2 logs
#pm2 flush

# empty database
redis-cli flushdb

# print db content
redis-cli lrange cloudsim-sim 0 -1
