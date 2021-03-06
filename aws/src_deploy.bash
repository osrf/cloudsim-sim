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
  record_gazebo_log=`get_option $optionsfile record_gazebo_log`
  echo "record_gazebo_log: $record_gazebo_log"
  if [ "$record_gazebo_log" == "true" ]; then
    echo "true" > $codedir/record_gazebo_log.cfg
  fi

  # Fetch bundles
  mkdir -p $codedir/simulator
  curl -X GET --header 'Accept: application/json' --header "authorization: $token" $route > $codedir/simulator/bundle.tgz

  # Unpack bundles
  cd $codedir/simulator
  tar xf bundle.tgz

  # Create static IP configuration for each client on the subnet
  mkdir -p $codedir/simulator/staticclients

  echo "One field computer per team."
  echo "ifconfig-push ${subnet}.8 255.255.255.0" > $codedir/simulator/staticclients/fieldcomputer

  # Start servers
  # the last arg used to be the other_subnet but since there is only one subnet
  # we just pass the same variable
  cd $codedir/simulator
  $codedir/simulator/start_vpn.bash simulator $subnet openvpn.conf $subnet

  # allow only traffic from field computer to sim instance and block all others in the subnet
  iptables -I INPUT --src 192.168.2.1 -j ACCEPT
  iptables -I INPUT --src 192.168.2.8 -j ACCEPT # field computer host ip
  iptables -I INPUT --src 192.168.2.10 -j ACCEPT # field computer docker container ip
  iptables -A INPUT --src 192.168.2.0/24 -j DROP

  # Make the servers come back up on reboot
  cat << EOF > /etc/rc.local
#!/bin/bash
cd $codedir/simulator && $codedir/simulator/start_vpn.bash simulator $subnet openvpn.conf $subnet
exit 0
EOF

  srcsimulationid=`get_option $optionsfile srcsimulation_id`
  echo "srcsimulationid: $srcsimulationid"

  # Get S3 credentials, if any
  # /etc/passwd-s3fs: contains AWS keys in this format -> bucketname:public:private
  # /home/ubuntu/s3/bucketname-s3fs.txt: contains the bucket name to use in S3
  s3bucket=`get_option $optionsfile s3bucket`
  echo "s3bucket: $s3bucket"
  if [ "$s3bucket" != "undefined" ]; then
    s3dir="$codedir/../s3"
    mkdir -p $s3dir
    s3accesskey=`get_option $optionsfile s3accesskey`
    s3privatekey=`get_option $optionsfile s3privatekey`
    echo "s3accesskey: $s3accesskey"
    echo "s3privatekey: $s3privatekey"
    echo $s3bucket:$s3accesskey:$s3privatekey > /etc/passwd-s3fs
    chmod 640 /etc/passwd-s3fs
    echo $s3bucket > $s3dir/bucketname-s3fs.txt
    # Mount folder
    mkdir -p /mnt/s3bucket
    chmod 776 /mnt/s3bucket
    /usr/bin/s3fs -o createbucket -o use_cache=/tmp $s3bucket:/$srcsimulationid /mnt/s3bucket
  fi

elif [ $role == "fieldcomputer" ]; then

  enable_traffic_shaper=`get_option $optionsfile enable_traffic_shaper`
  echo "enable_traffic_shaper: $enable_traffic_shaper"
  if [ "$enable_traffic_shaper" == "true" ]; then
    echo "true" > $codedir/enable_traffic_shaper.cfg
  fi

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
  sleep 5

  echo "about to setup bridge for vpn"
  # Create docker network (vpn-br0) and corresponding OS bridge (br0)
  docker network create --driver=bridge --ip-range=192.168.2.8/31 --subnet=192.168.2.0/24 -o "com.docker.network.bridge.name=br0" vpn-br0

  # Make the client come back up on reboot
  cat << EOF > /etc/rc.local
#!/bin/bash
cd $codedir/vpn && openvpn --config openvpn.conf --daemon
exit 0
EOF

  # Download and build team's docker image
  dockerurl=`get_option $optionsfile dockerurl`
  github_deploy_key=`get_option $optionsfile github_deploy_key`

  echo "dockerurl: $dockerurl"
  echo "github_deploy_key: $github_deploy_key"

  if [ "$github_deploy_key" != "undefined" ]; then
    # Read and configure team's Deploy SSH Key (for github)
    key_path=~/.ssh/deploy_key_rsa
    node $DIR/read_deploy_key.js > $key_path
    chmod 400 $key_path
    # Start the ssh-agent in the background.
    eval "$(ssh-agent -s)"
    # Add ssh key to agent
    ssh-add $key_path
    # Add github to known hosts
    ssh-keyscan github.com >> ~/.ssh/known_hosts
  fi

  echo "downloading and building team's dockerfile"
  for ((i = 0; i < 5; ++i))
  do
    # Sleep on failure in case it is a temporary network issue
    sleep $(expr 5 \* $i)

    if docker build -t fcomputer:latest $dockerurl
    then
      echo "Image built successfully on try $i"
      break
    fi
    echo "Image failed to build on try $i"
  done


  if [ "$github_deploy_key" != "undefined" ]; then
    # Kill ssh agent
    trap "kill $SSH_AGENT_PID" exit
  fi

  # Finish setting up bridge for vpn
  # We assume by this time, docker and vpn are already "up"
  echo "about to finish setting up bridge for vpn"
  brctl addif br0 tap0
  brctl setfd br0 0
  ifconfig tap0 0.0.0.0 up
  ifconfig br0 192.168.2.8 netmask 255.255.255.0 broadcast 192.168.2.255

  if ! docker images | grep fcomputer
  then
    echo "Image not found!"
    # Notify cloudsim-sim that image failed to build
    curl -X POST --header "Content-Type: application/json" --header 'Accept: application/json' --header "authorization: $token" --data '{"fc_docker_image":""}' "http://localhost:4000/events"
    exit 5
  fi

  #  Notify cloudsim-sim server that the team's image has been built
  curl -X POST --header "Content-Type: application/json" --header 'Accept: application/json' --header "authorization: $token" --data '{"fc_docker_image":"fcomputer"}' "http://localhost:4000/events"

else
  echo "ERROR: Unknown role \"$role\"."
fi
