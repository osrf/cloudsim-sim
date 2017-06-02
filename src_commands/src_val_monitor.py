#!/usr/bin/env python

import rospy
import time
import threading

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

sleepTime = 5
sleepTimeForMessageCheck = 30

# The topics to check
topics = ["/joint_states", "/ihmc_ros/valkyrie/output/robot_pose"]

# Message count for each topic
topicsMsgCount = [0, 0]

# A condition variable for each topic
cvs = [threading.Condition(), threading.Condition()]

# Callback for the topics
def callback(data, i):
  cvs[i].acquire()
  topicsMsgCount[i] = topicsMsgCount[i] + 1
  cvs[i].notify()
  cvs[i].release()

def main():

  rospy.init_node('val_monitor', anonymous=True)

  # Subscribe to the necessary topics
  rospy.Subscriber(topics[0], JointState, callback, 0)
  rospy.Subscriber(topics[1], Odometry, callback, 1)

  # Wait for a bit
  time.sleep(sleepTime)

  good = True
  for i, cv in enumerate(cvs):
    # Acquire the lock
    cv.acquire()

    # Store the current message count
    count = topicsMsgCount[i]

    # Wait for a bit for more messages to arrive
    cv.wait(sleepTimeForMessageCheck)

    # Check that messages have arrived
    good = good and topicsMsgCount[i] > count
    cv.release()

  if good:
    print("Valid simulation")
  else:
    print("Invalid simulation")

if __name__ == '__main__':
  main()
