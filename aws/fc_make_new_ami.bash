#!/usr/bin/env bash

# This script is used to make a new Field Computer ami from an existing one:
# - removes ssh keys
# - removes all docker containers and images

if [ "$USER" != "ubuntu" ]; then
  echo 'Please run as user "ubuntu"'
  exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

$DIR/make_new_ami.bash

# Remove ssh deploy key
rm -f ~/.ssh/deploy_key_rsa

# stop and remove docker containers
docker stop $(docker ps -a -q)
docker rm $(docker ps -a -q)
# Delete all images
docker rmi $(docker images -q)
