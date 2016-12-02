#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


$DIR/make_new_ami.bash

# SASC-specific cleanup
sudo rm -rf $DIR/../../blue
sudo rm -rf $DIR/../../gold
