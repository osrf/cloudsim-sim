#!/bin/bash

# To be executed after the machine is created. It can read from cloudsim-options.json.

set -x

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
codedir="$DIR/../.."

# Helper to parse options from cloudsim-options.json
get_option(){
  echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`
}

# This file is created by cloudsim when the machine is launched
optionsfile=$codedir/cloudsim-options.json

echo "codedir is $codedir"
# Common options
role=`get_option $optionsfile role`
token=`get_option $optionsfile token`
# Arbiter options
blue_route=`get_option $optionsfile blue_route`
gold_route=`get_option $optionsfile gold_route`
blue_subnet=`get_option $optionsfile blue_subnet`
gold_subnet=`get_option $optionsfile gold_subnet`
payload_count=`get_option $optionsfile payload_count`
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

#apt-get update
#apt-get install sasc-gazebo-sitl

if [ $role == "arbiter" ]; then

  # Fetch bundles
  mkdir -p $codedir/blue $codedir/gold
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" $blue_route > $codedir/blue/bundle.tgz
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" $gold_route > $codedir/gold/bundle.tgz

  # Unpack bundles
  cd $codedir/blue
  tar xf bundle.tgz
  cd $codedir/gold
  tar xf bundle.tgz

  # Create static IP configuration for each client on each of the two subnets
  mkdir -p $codedir/blue/staticclients
  mkdir -p $codedir/gold/staticclients
  # Just a bit of backward compatibility
  if [ $payload_count == "undefined" ]; then
    echo "No payload_count; falling back to old behavior of one payload per team."
    echo "ifconfig-push ${blue_subnet}.10 255.255.255.0" > $codedir/blue/staticclients/payload
    echo "ifconfig-push ${gold_subnet}.10 255.255.255.0" > $codedir/gold/staticclients/payload
  else
    echo "Creating static IPs for $payload_count payloads per team"
    for (( payload_num=0; payload_num<$payload_count; payload_num++ )); do
      echo "ifconfig-push ${blue_subnet}.$((10+payload_num)) 255.255.255.0" > $codedir/blue/staticclients/payload${payload_num}
      echo "ifconfig-push ${gold_subnet}.$((10+payload_num)) 255.255.255.0" > $codedir/gold/staticclients/payload${payload_num}
    done
  fi

  # Start servers
  cd $codedir/blue
  $codedir/blue/start_vpn.bash blue $blue_subnet openvpn.conf $gold_subnet
  cd $codedir/gold
  $codedir/gold/start_vpn.bash gold $gold_subnet openvpn.conf $blue_subnet

  # Make the servers come back up on reboot
  cat << EOF > /etc/rc.local
#!/bin/bash
cd $codedir/blue && $codedir/blue/start_vpn.bash blue $blue_subnet openvpn.conf $gold_subnet
cd $codedir/gold && $codedir/gold/start_vpn.bash gold $gold_subnet openvpn.conf $blue_subnet
exit 0
EOF
elif [ $role == "payload" ]; then
  # Fetch bundle
  mkdir -p $codedir/vpn
  echo curl -X GET --header 'Accept: application/json' --header "authorization: $token" "${client_route}?serverIp=${server_ip}&id=${client_id}"
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" "${client_route}?serverIp=${server_ip}&id=${client_id}" > $codedir/vpn/bundle.tgz

  # Unpack bundle
  cd $codedir/vpn
  tar xf bundle.tgz

  # Start server
  echo cd $codedir/vpn
  cd $codedir/vpn
  echo openvpn --config openvpn.conf --daemon
  openvpn --config openvpn.conf --daemon

  # Make the client come back up on reboot
  cat << EOF > /etc/rc.local
#!/bin/bash
cd $codedir/vpn && openvpn --config openvpn.conf --daemon" >> /etc/rc.local
exit 0
EOF
else
  echo "ERROR: Unknown role \"$role\"."
fi
