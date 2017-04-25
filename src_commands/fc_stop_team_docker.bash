#!/usr/bin/env bash
docker kill --signal "SIGINT" team_container 
echo "sent SIGINT signal to 'team_container' container for smooth shutdown"
echo "about to wait for 'team_container' container to stop"
docker wait team_container
echo "team_container stopped"
