#!/usr/bin/env bash

# must be run as root

# redirect port 443 incomming traffic (https) to the node server running
# on port 4000
iptables -A PREROUTING -t nat -i eth0 -p tcp --dport 443 -j REDIRECT --to-port 4000
