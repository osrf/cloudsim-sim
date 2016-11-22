#!/bin/bash

# To be executed after the machine is created. It can read from cloudsim-options.json.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Helper to parse options from cloudsim-options.json 
get_option(){
  echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`
}

# This file is created by cloudsim when the machine is launched
optionsfile=$DIR/cloudsim-options.json

# Common options
role=`get_option $optionsfile role`
token=`get_option $optionsfile token`
# Arbiter options
blue_route=`get_option $optionsfile blue_route`
gold_route=`get_option $optionsfile gold_route`
blue_subnet=`get_option $optionsfile blue_subnet`
gold_subnet=`get_option $optionsfile gold_subnet`
# Payload options
client_route=`get_option $optionsfile client_route`
server_ip=`get_option $optionsfile server_ip`
client_id=`get_option $optionsfile client_id`

echo "role: $role"
echo "token: $token"
echo "blue_route: $blue_route"
echo "gold_route: $gold_route"
echo "blue_subnet: $blue_subnet"
echo "gold_subnet: $gold_subnet"
echo "server_ip: $server_ip"
echo "client_id: $client_id"

if [ $role == "arbiter" ]; then
  # Fetch bundles
  mkdir -p $DIR/blue $DIR/gold
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" $blue_route > $DIR/blue/bundle.tgz
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" $gold_route > $DIR/gold/bundle.tgz
  
  # Unpack bundles
  cd $DIR/blue
  tar xf bundle.tgz
  cd $DIR/gold
  tar xf bundle.tgz
  
  # Start servers
  cd $DIR/blue
  $DIR/blue/start_vpn.bash blue $blue_subnet openvpn.conf
  cd $DIR/gold
  $DIR/gold/start_vpn.bash blue $gold_subnet openvpn.conf
elif [ $role == "payload" ]; then
  # Fetch bundle
  mkdir -p $DIR/vpn
  echo curl -X GET --header 'Accept: application/json' --header "authorization: $token" "${client_route}?serverIp=${server_ip}&id=${client_id}"
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" "${client_route}?serverIp=${server_ip}&id=${client_id}" > $DIR/vpn/bundle.tgz
  
  # Unpack bundle
  cd $DIR/vpn
  tar xf bundle.tgz
  
  # Start server
  echo cd $DIR/vpn
  cd $DIR/vpn
  echo openvpn --config openvpn.conf --daemon
  openvpn --config openvpn.conf --daemon
else
  echo "ERROR: Unknown role \"$role\"."
fi
