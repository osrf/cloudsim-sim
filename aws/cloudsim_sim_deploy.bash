#!/bin/bash

# To be executed after the machine is created. It can read from cloudsim-options.json.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
simdir="$DIR/.."
codedir="$DIR/../.."
# Helper to parse options from cloudsim-options.json
#get_option(){
#  echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`
#}


cp $codedir/cloudsim-env.bash $simdir/.env
cp $codedir/cloudsim-options.json $simdir/options.json

# Update cloudsim-sim
cd $simdir
hg pull
hg up
# Potentially install new deps
npm install
node $simdir/server/cloudsim_sim.js &


echo "cloudsim_sim_deploy.bash done"
