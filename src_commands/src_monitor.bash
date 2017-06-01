#!/bin/bash

codedir="/home/ubuntu/code"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source $codedir/srcsim_ws/devel/setup.bash

# Helper to parse options from cloudsim-options.json
get_option(){
  echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`
}
# This file is created by cloudsim when the machine is launched
optionsfile=$codedir/cloudsim-options.json

role=`get_option $optionsfile role`
token=`get_option $optionsfile token`

echo "role $role"
echo "token $token"

until rostopic list ; do sleep 1; done

echo "Starting SRC monitor"
python $DIR/src_monitor.py $role $token 