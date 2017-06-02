#!/usr/bin/env python

import os
import subprocess
import sys
import requests
import json

import rospy
import time

topics = ["/ihmc_ros/valkyrie/output/robot_pose", "/joint_states"]
topicsValid = [False, False]

def callback0(data):
  rospy.loginfo("Received callback on topic %s", topics[0])
  topic[0] = True

def callback1(data):
  rospy.loginfo("Received callback on topic %s", topics[1])
  topic[1] = True

def main():
  rospy.init_node('val_monitor', anonymous=True)

  # Subscribe to the necessary topics
  rospy.Subscriber(topics[0], int, callback0)
  rospy.Subscriber(topics[1], int, callback1)

  sleepTime = 1
  maxIters = 120

  for i in range(maxIters):
    # Check all topics to see if they have received a message
    complete = True
    for t in topicsValid: 
      complete = complete && t

    # Wait for a bit
    time.sleep(sleepTime)


if __name__ == '__main__':
  main()
