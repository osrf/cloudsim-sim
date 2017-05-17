#!/bin/bash

# To be executed after the machine is created. It can read from cloudsim-options.json.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
simdir="$DIR/.."
codedir="$DIR/../.."
# Helper to parse options from cloudsim-options.json
get_option(){
  echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`
}

optionsfile=$codedir/cloudsim-options.json
echo "optionsfile $optionsfile"
ssh_route=`get_option $optionsfile ssh`
token=`get_option $optionsfile token`

# if ssh_route is valid, download the key and place it in the server directory
if [ "$ssh_route" != "undefined" ]; then  
  echo "ssh key [$ssh_route]"
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" $ssh_route > $codedir/sshkeys.zip
  cp $codedir/sshkeys.zip $simdir/server/keys.zip
else
  echo "default ssh key"
fi

cp $codedir/cloudsim-env.bash $simdir/.env
cp $codedir/cloudsim-options.json $simdir/options.json

# Update cloudsim-sim
cd $simdir
# switch to user ubuntu before pulling changes
echo "switching to user ubuntu before pulling changes"
su ubuntu
hg pull
hg up
echo "exiting ubuntu user -- returning back to root user"
exit
# Potentially install new deps
npm install
# TODO: consider moving this run server line after src_deploy.bash was executed
# Potential place for this line: in the code/cloudsim_deploy.bash, after running both cloudsim_sim_deploy and src_deploy scripts.
node $simdir/server/cloudsim_sim.js &

echo "cloudsim_sim_deploy.bash done"
