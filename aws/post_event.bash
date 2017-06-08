#!/bin/bash

# This script posts an event to http://localhost:4000/events (cloudsim-sim server). 
# That event will be sent back automatically to Portal.
# Parameters:
# data: data to send as event. It should be in json. Eg: '{ "mydata": true }'

if [ $# -lt 1 ]; then
    echo '# This script posts an event to http://localhost:4000/events (cloudsim-sim server).'
    echo '# That event will be sent back automatically to Portal.'
    echo '# Parameters:'
    echo '# --- data: data to send as event. It should be in json format and enclosed with single quotes.'
    echo "# --- Eg: $ bash post_event.bash '{ \"mydata\": true }'"
    exit 1
fi

DATA=$1
echo "about to send: $DATA"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

codedir="$DIR/../.."

# Helper to parse options from cloudsim-options.json
get_option(){
  echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`
}

# This file is created by cloudsim when the machine is launched
optionsfile=$codedir/cloudsim-options.json

token=`get_option $optionsfile token`

#  Send event
set -x
curl -X POST --header "Content-Type: application/json" --header 'Accept: application/json' --header "authorization: $token" --data "$DATA" "http://localhost:4000/events"