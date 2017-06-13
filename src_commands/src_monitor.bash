#!/bin/bash

final=$1
world=$2

codedir="/home/ubuntu/code"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source $codedir/srcsim_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.2.1:11311

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
echo "final $final"

until rostopic list ; do sleep 1; done

echo "Updating sim data"
if [ $role == "simulator" ]; then
  # get team name from world name
  team=`echo "$world" |  grep -o 'SRC-[^-]*' 
  data="{\"data\": {\"round\": \"$final\", \"team\":\"$team\"}}"
  curl -X POST --header "Content-Type: application/json" --header 'Accept: application/json' --header "authorization: $token" --data "$data" "http://localhost:4000/sim"
elif [ $role == "fieldcomputer" ]; then
  curl --header "Content-Type: application/json" --header 'Accept: application/json' --header "authorization: $token" "http://192.168.2.1:4000/sim" > $DIR/sim_data.json
  sed -i 's/\\//g' $DIR/sim_data.json
  sed -i 's/^.\(.*\).$/\1/' $DIR/sim_data.json
  round=`get_option $DIR/sim_data.json round`
  team=`get_option $DIR/sim_data.json team`
  echo "team: $team, round: $round"

  # workout team<->round<->world TC mapping
  $DIR/fc_tc_parse.py $DIR/tcFile $DIR/teamRoundMappingFile $team $round
else
  echo "Invalid role! $role"
fi

echo "Starting SRC monitor"
python $DIR/src_monitor.py $role $token $final
