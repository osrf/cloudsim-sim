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
# simulator options
route=`get_option $optionsfile route`
subnet=`get_option $optionsfile subnet`

# field computer options
client_route=`get_option $optionsfile client_route`
server_ip=`get_option $optionsfile server_ip`
client_id=`get_option $optionsfile client_id`

echo "role: $role"
echo "token: $token"
echo "route: $route"
echo "subnet: $subnet"
echo "server_ip: $server_ip"
echo "client_id: $client_id"

if [ $role == "simulator" ]; then

  # Fetch bundles
  mkdir -p $codedir/simulator
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" $route > $codedir/simulator/bundle.tgz

  # Unpack bundles
  cd $codedir/simulator
  tar xf bundle.tgz

  # Create static IP configuration for each client on the subnet
  mkdir -p $codedir/simulator/staticclients

  echo "One field computer per team."
  echo "ifconfig-push ${subnet}.10 255.255.255.0" > $codedir/simulator/staticclients/fieldcomputer

  # Start servers
  # the last arg used to be the other_subnet but since there is only one subnet
  # we just pass the same variable
  cd $codedir/simulator
  $codedir/simulator/start_vpn.bash simulator $subnet openvpn.conf $subnet

  # Make the servers come back up on reboot
  cat << EOF > /etc/rc.local
#!/bin/bash
cd $codedir/simulator && $codedir/simulator/start_vpn.bash simulator $subnet openvpn.conf $subnet
exit 0
EOF
elif [ $role == "fieldcomputer" ]; then
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
cd $codedir/vpn && openvpn --config openvpn.conf --daemon
exit 0
EOF
else
  echo "ERROR: Unknown role \"$role\"."
fi