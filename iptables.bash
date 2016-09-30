#!/usr/bin/env bash

# must be run as root

# redirect port 443 incomming traffic (https) to the node server running
# on port 4000
#iptables -A PREROUTING -t nat -i eth0 -p tcp --dport 443 -j REDIRECT --to-port 4000

# http
iptables -A PREROUTING -t nat -i eth0 -p tcp --dport 80 -j REDIRECT --to-port 4000

# then, you can make the rule persistent by installing this package
# and answering yes to the 'copy existing rules' question
#
# sudo apt install iptables-persistent
# sudo npm install -g pm2
