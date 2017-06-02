#!/usr/bin/env python

import os
import subprocess
import sys
import requests
import json
import copy

import rospy
from srcsim.msg import Task

role = None
token = None
roundNumber = None
prevTaskId = -1
tasks = [None] * 3
prevTasks = None
scriptDir = os.path.dirname(os.path.realpath(__file__))

def simTaskCallback(data):

  global token, tasks, prevTaskId, prevTasks, roundNumber

  taskId = data.task
  currentCheckPoint = data.current_checkpoint
  startTime = data.start_time.to_sec()
  elapsedTime = data.elapsed_time.to_sec()
  timedOut = data.timed_out
  finished = data.finished
  checkpoint_durations = []
  for i in range(len(data.checkpoint_durations)):
    checkpoint_durations.append(data.checkpoint_durations[i].to_sec())
  checkpoint_penalties = []
  for i in range(len(data.checkpoint_penalties)):
    checkpoint_penalties.append(data.checkpoint_penalties[i].to_sec())

  # throttle update rate and publish only when data changes
  if prevTasks != None and taskId == prevTaskId and \
      prevTasks[taskId-1]["current_checkpoint"] == currentCheckPoint and \
      prevTasks[taskId-1]["timed_out"] == timedOut and \
      prevTasks[taskId-1]["finished"] == finished and \
      (elapsedTime - prevTasks[taskId-1]["elapsed_time"] < 5):
    return

  # log output
  rospy.loginfo("task: %u", taskId)
  rospy.loginfo("current_checkpoint: %u", currentCheckPoint)
  rospy.loginfo("start_time: %f", startTime)
  rospy.loginfo("elapsed_time: %f", elapsedTime)
  rospy.loginfo("timed_out: %s", timedOut)
  rospy.loginfo("finished: %s", finished)
  for i in range(len(checkpoint_durations)):
    rospy.loginfo("checkpoint_durations %i: %f", i, checkpoint_durations[i])
  for i in range(len(checkpoint_penalties)):
    rospy.loginfo("checkpoint_penalties %i: %f", i, checkpoint_penalties[i])

  # post to cloudsim-sim
  tasks[taskId-1] = {}
  tasks[taskId-1]["current_checkpoint"] = currentCheckPoint
  tasks[taskId-1]["start_time"] = startTime
  tasks[taskId-1]["elapsed_time"] = elapsedTime
  tasks[taskId-1]["timed_out"] = timedOut
  tasks[taskId-1]["finished"] = finished
  tasks[taskId-1]["checkpoint_durations"] = checkpoint_durations
  tasks[taskId-1]["checkpoint_penalties"] = checkpoint_penalties

  prevTaskId = taskId
  prevTasks = copy.deepcopy(tasks)

  roundName = "round_" + str(roundNumber)
  dataJson = json.dumps({roundName: tasks})

  rospy.loginfo("dataJson: %s", dataJson)

  headers = {'Content-type': 'application/json', 'Authorization': token}
  url="http://localhost:4000/events"
  response = requests.post(url, data=dataJson, headers=headers)
  if response.status_code != 200:
    rospy.loginfo('Unexpected response: %i ', response.status_code)
  rospy.loginfo('Response: %s ', response.text)

def fcTaskCallback(data):

  global prevTaskId, scriptDir

  taskId = data.task
  if taskId == prevTaskId:
    return

  prevTaskId = taskId

  # call traffic shaper script to update bandwidth limitation
  uplink = ""
  donwlink = ""

  if taskId == 1:
    uplink = "380kbit"
    downlink = "4kbit"
  elif taskId == 2:
    uplink = "2mbit"
    downlink = "30kbit"
  elif taskId == 3:
    uplink = "2mbit"
    downlink = "30kbit"

  cmd = scriptDir + "/src_tc.rb"

  rospy.loginfo("task: %u", taskId)
  rospy.loginfo("uplink/donwlink: %s/%s", uplink, downlink)

  out = subprocess.Popen(["sudo", cmd, "-i", "tap0", "-u", uplink, "-d", downlink, "-f", "192.168.2.150/26"])

def main():
  global token, roundNumber

  if len(sys.argv) < 2:
    print "Role missing!"
    sys.exit()

  role = sys.argv[1]
  rospy.loginfo("role: %s", role)

  rospy.init_node('task_monitor', anonymous=True)

  if role == "simulator":
    if len(sys.argv) < 4:
      print "token or round number missing!"
      sys.exit()
    token = sys.argv[2]
    roundNumber = sys.argv[3]
    rospy.loginfo("token: %s", token)
    rospy.loginfo("round number: %s", roundNumber)

    rospy.Subscriber("/srcsim/finals/task", Task , simTaskCallback)
  elif role == "fieldcomputer":
    rospy.Subscriber("/srcsim/finals/task", Task , fcTaskCallback)
  else:
    print "Invalid role!"
    sys.exit()

  rospy.spin()

if __name__ == '__main__':
  main()
