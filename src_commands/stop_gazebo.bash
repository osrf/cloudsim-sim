#!/usr/bin/env bash
docker kill --signal "SIGINT" gazebo_run
echo "sent SIGINT signal to gazebo_run container for smooth shutdown"
echo "about to wait for gazebo_run container to stop"
docker wait gazebo_run
echo "gazebo_run container stopped"
