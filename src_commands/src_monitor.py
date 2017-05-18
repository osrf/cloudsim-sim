#!/usr/bin/env python

import sys
import requests
import json

import rospy
from srcsim.msg import Task

role = None
token = None
prevTaskId = -1
tasks = {}
prevTasks = None

def simTaskCallback(data):

  global token, tasks, prevTaskId, prevTasks

  taskId = data.task
  currentCheckPoint = data.current_checkpoint
  startTime = data.start_time.secs + data.start_time.nsecs / 1e9
  elapsedTime = data.elapsed_time.secs + data.elapsed_time.nsecs / 1e9
  timedOut = data.timed_out
  finished = data.finished
  checkpoints_completion = []
  for i in range(len(data.checkpoints_completion)):
    checkpoints_completion.append(data.checkpoints_completion[i].secs +
        data.checkpoints_completion[i].nsecs / 1e9)

  # throttle update rate and publish only when data changes
  if prevTasks != None and taskId == prevTaskId and \
      prevTasks[taskId]["current_checkpoint"] == currentCheckPoint and \
      prevTasks[taskId]["timed_out"] == timedOut and \
      prevTasks[taskId]["finished"] == finished and \
      (elapsedTime - prevTasks[taskId]["elapsed_time"] < 5):
    return

  rospy.loginfo("task: %u", taskId)
  rospy.loginfo("current_checkpoint: %u", currentCheckPoint)
  rospy.loginfo("start_time: %f", startTime)
  rospy.loginfo("elapsed_time: %f", elapsedTime)
  rospy.loginfo("timed_out: %s", timedOut)
  rospy.loginfo("finished: %s", finished)

  for i in range(len(checkpoints_completion)):
    rospy.loginfo("checkpoints_completion %i: %f", i, checkpoints_completion[i])

  # post to cloudsim-sim
  tasks[taskId] = {}
  tasks[taskId]["current_checkpoint"] = currentCheckPoint
  tasks[taskId]["start_time"] = startTime
  tasks[taskId]["elapsed_time"] = elapsedTime
  tasks[taskId]["timed_out"] = timedOut
  tasks[taskId]["finished"] = finished
  tasks[taskId]["checkpoints_completion"] = checkpoints_completion

  prevTaskId = taskId
  prevTasks = tasks.copy()

  # TODO get round name
  dataJon = json.dumps({"round": tasks})
  headers = {'Content-type': 'application/json', 'Authorization': token}
  url="http://localhost:4000/events"
  response = requests.post(url, data=dataJon, headers=headers)
  if response.status_code != 200:
    rospy.loginfo('Unexpected response: %i ', response.status_code)
  rospy.loginfo('Response: %s ', response.text)

def fcTaskCallback(data):

  global prevTaskId

  taskId = data.task
  if taskId == prevTaskId:
    return

  prevTaskId = taskId 

  rospy.loginfo("task: %u", taskId)
  # call traffic shaper script to update bandwidth limitation

def main():
  global token

  if len(sys.argv) < 2:
    print "Role missing!"
    sys.exit()

  role = sys.argv[1]
  rospy.loginfo("role: %s", role)

  rospy.init_node('task_monitor', anonymous=True)

  if role == "simulator":
    if len(sys.argv) < 3:
      print "token missing!"
      sys.exit()
    token = sys.argv[2]
    rospy.loginfo("token: %s", token)

    rospy.Subscriber("/srcsim/finals/task", Task , simTaskCallback)
  elif role == "fieldcomputer":
    rospy.Subscriber("/srcsim/finals/task", Task , fcTaskCallback)
  else:
    print "Invalid role!"
    sys.exit()

  rospy.spin()

if __name__ == '__main__':
  main()
