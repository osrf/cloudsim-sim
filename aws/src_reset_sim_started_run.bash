#!/bin/bash

# Sends an event to reset run button 

number=$1

if [ -z "$number" ]; then
  echo "Missing arg. Please specify a round numnber"
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

$DIR/post_event.bash "{\"started_round_$number\":false}"
