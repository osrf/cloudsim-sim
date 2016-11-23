#!/bin/bash

# To be executed after the machine is created. It can read from cloudsim-options.json.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Helper to parse options from cloudsim-options.json
#get_option(){
#  echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`
#}

# This file is created by cloudsim when the machine is launched
optionsfile=$DIR/cloudsim-options.json

cp $DIR/cloudsim-env.bash $DIR/cloudsim-sim/.env
cp $DIR/cloudsim-options.json $DIR/cloudsim-sim/options.json

# Update cloudsim-sim
cd $DIR/cloudsim-sim
hg up
# Potentially install new deps
npm install
node $DIR/cloudsim-sim/server/cloudsim_sim.js &




