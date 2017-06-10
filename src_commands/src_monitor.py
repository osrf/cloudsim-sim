#!/usr/bin/env python

import os
import subprocess
import sys
import requests
import json
import copy
import threading

import time

import rospy
from srcsim.msg import Task
from srcsim.msg import Score
from srcsim.msg import Harness

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

role = None
token = None
roundName = None
prevTaskId = -1
tasks = [None] * 3
prevTasks = None
score = 0
prevScore = 0
totalCompletionTime = 0
uplink = "N/A"
downlink = "N/A"
latency = "N/A"

# harness monitor variable
prevHarnessStatus = -1

scriptDir = os.path.dirname(os.path.realpath(__file__))

# Set global variables holding traffic parameters, this is done
# for both FC and Sim
def setTrafficParams(_taskId):
  if _taskId == 1:
    uplink = "380kbit"
    downlink = "4kbit"
    latency = "TODO"
  elif _taskId == 2:
    uplink = "2mbit"
    downlink = "30kbit"
    latency = "TODO"
  elif _taskId == 3:
    uplink = "2mbit"
    downlink = "30kbit"
    latency = "TODO"


def postToSim():

  global token, tasks, score, totalCompletionTime, roundName, mutex

  mutex.acquire()
  dataJson = json.dumps({
    roundName: {
      "tasks": tasks,
      "score": score,
      "total_completion_time": totalCompletionTime,
      "uplink": uplink,
      "downlink": downlink,
      "latency": latency
    }
  })
  mutex.release()
  postData(dataJson)

def postData(dataJson):

  rospy.logwarn("dataJson: %s", dataJson)
  try:
    headers = {'Content-type': 'application/json', 'Authorization': token}
    url="http://localhost:4000/events"
    response = requests.post(url, data=dataJson, headers=headers)
    if response.status_code != 200:
      rospy.logwarn('Unexpected response: %i ', response.status_code)
    rospy.logwarn('Response: %s ', response.text)
  except:
    rospy.logwarn('Unable to post to cloudsim-sim. Is server running?')


def postControllerStatus(data):
  dataJson = json.dumps({"simulator_ready": data})
  postData(dataJson)

def checkControllerStatus():

  topicsWaitSleepTime = 20
  sleepTimeForMessageCheck = 15

  # Wait for a bit
  time.sleep(topicsWaitSleepTime)
  rospy.logwarn('Started monitoring controller topics')

  timeout = 120
  try:
    rospy.wait_for_message("/ihmc_ros/valkyrie/output/robot_pose", Odometry, timeout)
  except(rospy.ROSException), e:
    rospy.logwarn("Failed to receive ihmc robot_pose message, aborting...")
    postControllerStatus(-1)
    return

  rospy.logwarn('Received ihmc robot_pose message')

  try:
    rospy.wait_for_message("/joint_states", JointState, timeout)
  except(rospy.ROSException), e:
    rospy.logwarn("Failed to receive joint states message, aborting...")
    postControllerStatus(-1)
    return

  rospy.logwarn('Received joint states message')
  rospy.logwarn('Sleeping for more checks')

  time.sleep(sleepTimeForMessageCheck)

  # We should still be able to receive more messages
  try:
    rospy.wait_for_message("/joint_states", JointState, 10)
    rospy.wait_for_message("/ihmc_ros/valkyrie/output/robot_pose", Odometry, 10)
  except(rospy.ROSException), e:
    rospy.logwarn("Failed to receive more messages, aborting...")
    postControllerStatus(-1)
    return

  rospy.logwarn("Valid simulation")
  postControllerStatus(1)

def postHarnessStatus(data):
  dataJson = json.dumps({"harness_status": data})
  postData(dataJson)

def simHarnessCallback(data):

  global prevHarnessStatus

  harnessStatus = data.status
  if harnessStatus == prevHarnessStatus:
    return

  postHarnessStatus(data.status)
  prevHarnessStatus = harnessStatus

def checkHarnessStatus():

  topicWaitSleepTime = 100

  # Wait for a bit
  time.sleep(topicWaitSleepTime)
  rospy.logwarn('Started monitoring harness topic')

  rospy.Subscriber("/srcsim/finals/harness", Harness, simHarnessCallback)

  # monitor for whole sim duration
  while True:
    time.sleep(10)


def simTaskCallback(data):

  global token, tasks, prevTaskId, prevTasks, mutex

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

  # Update traffic params to notify CloudSim, this doesn't call TC
  setTrafficParams(taskId)

  # throttle update rate and publish only when data changes
  if prevTasks != None and taskId == prevTaskId and \
      prevTasks[taskId-1]["current_checkpoint"] == currentCheckPoint and \
      prevTasks[taskId-1]["timed_out"] == timedOut and \
      prevTasks[taskId-1]["finished"] == finished and \
      (elapsedTime - prevTasks[taskId-1]["elapsed_time"] < 5):
    return

  # log output
  rospy.logwarn("task: %u", taskId)
  rospy.logwarn("current_checkpoint: %u", currentCheckPoint)
  rospy.logwarn("start_time: %f", startTime)
  rospy.logwarn("elapsed_time: %f", elapsedTime)
  rospy.logwarn("timed_out: %s", timedOut)
  rospy.logwarn("finished: %s", finished)
  for i in range(len(checkpoint_durations)):
    rospy.logwarn("checkpoint_durations %i: %f", i, checkpoint_durations[i])
  for i in range(len(checkpoint_penalties)):
    rospy.logwarn("checkpoint_penalties %i: %f", i, checkpoint_penalties[i])

  mutex.acquire()
  # update tasks
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
  mutex.release()

  # post to cloudsim-sim
  rospy.logwarn("== Posting TASK data ==")
  postToSim()


def simScoreCallback(data):

  global score, prevScore, totalCompletionTime, mutex

  mutex.acquire()
  totalCompletionTime = data.total_completion_time.to_sec()
  mutex.release()

  if data.score == prevScore:
    return

  mutex.acquire()
  score = data.score
  prevScore = score
  mutex.release()

  # post to cloudsim-sim
  rospy.logwarn("== Posting SCORE data ==")
  postToSim()


def fcTaskCallback(data):

  global prevTaskId, scriptDir

  taskId = data.task
  if taskId == prevTaskId:
    return

  prevTaskId = taskId

  # call traffic shaper script to update bandwidth limitation
  setTrafficParams(taskId)

  cmd = scriptDir + "/src_tc.rb"

  rospy.logwarn("task: %u", taskId)
  rospy.logwarn("uplink/downlink/latency: %s/%s/", uplink, downlink, latency)

  # TODO: Use latency
  out = subprocess.Popen(["sudo", cmd, "-i", "tap0", "-u", uplink, "-d", downlink, "-f", "192.168.2.150/26"])

def main():
  global token, roundName, mutex

  if len(sys.argv) < 2:
    print "Role missing!"
    sys.exit()

  role = sys.argv[1]

  if role == "simulator":
    if len(sys.argv) < 4:
      print "token or round number missing!"
      sys.exit()

    time.sleep(20)
    rospy.init_node('task_monitor_sim', anonymous=True)

    token = sys.argv[2]
    roundNumber = sys.argv[3]
    rospy.logwarn("role: %s", role)
    rospy.logwarn("token: %s", token)
    rospy.logwarn("round number: %s", roundNumber)
    roundName = "round_" + str(roundNumber)

    mutex = threading.Lock()

    # Subscribe to topics for task and score data
    rospy.Subscriber("/srcsim/finals/task", Task, simTaskCallback)
    rospy.Subscriber("/srcsim/finals/score", Score, simScoreCallback)

    # Subscribe to topics for checking sim launch process
    postControllerStatus(0)
    controllerThread = threading.Thread(target=checkControllerStatus)
    controllerThread.start()
    # Subscribe to topic for monitoring harness status
    postHarnessStatus(-1)
    harnessThread = threading.Thread(target=checkHarnessStatus)
    harnessThread.start()

  elif role == "fieldcomputer":
    rospy.init_node('task_monitor_fc', anonymous=True)
    rospy.Subscriber("/srcsim/finals/task", Task, fcTaskCallback)
  else:
    print "Invalid role!"
    sys.exit()

  rospy.spin()

if __name__ == '__main__':
  main()
