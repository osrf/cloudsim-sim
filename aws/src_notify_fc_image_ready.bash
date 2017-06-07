#!/bin/bash

# Sends an event notifying the FC docker image was successfully built

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

$DIR/post_event.bash '{"fc_docker_image":"fcomputer"}'
