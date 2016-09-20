[ ![Codeship Status for osrf/cloudsim-sim](https://codeship.com/projects/c1074290-4c5e-0134-4ebf-52026d0c47d6/status?branch=default)](https://codeship.com/projects/170204)

[![Dependency Status](https://www.versioneye.com/user/projects/57ca1ead69d949002f38dc6f/badge.svg?style=flat-square)](https://www.versioneye.com/user/projects/57ca1ead69d949002f38dc6f)

[![Coverage Status](https://coveralls.io/repos/bitbucket/osrf/cloudsim-sim/badge.svg?branch=default)](https://coveralls.io/bitbucket/osrf/cloudsim-sim?branch=default)

# README #

This is the sim/robot control server for Cloudsim

### What is this repository for? ###

* A web app that starts and stops gazebo simulations
* Starts and stops ROS nodes
* A json web token delivery

### How do I get set up? ###

* install redis (sudo apt-get install redis-server)
* npm install
* gulp
* Dependencies: nodejs 4 and above, gulp (sudo npm install gulp -g)
* Database configuration: Redis for now
* Deployment instructions

## How do I run tests?

    gulp test

### Redirect port 443

Use the iptables.bash script to redirect https traffic to the node instance

### Contribution guidelines ###

### Who do I talk to? ###

* Repo owner or admin: hugo@osrfoundation.org
