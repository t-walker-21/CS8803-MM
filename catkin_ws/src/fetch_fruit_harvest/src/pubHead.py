#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from tf import TransformListener
import tf
import geometry_msgs

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal

def done_cb(status, result):
 pass

def active_cb():
 print ("active")

def feedback_cb(feedback):
 print (feedback)


def point():

 

 goal = PointHeadGoal()
 goal.target.header.stamp = rospy.Time.now()
 #goal.target.header.frame_id = "base_link"
 goal.target.header.frame_id = "map"
 ## look right
 goal.target.point.x = 0.3
 goal.target.point.y = -1.6
 goal.target.point.z = 0.6
 ##look right

 ## look forward
 goal.target.point.x = 0.5
 goal.target.point.y = 0
 goal.target.point.z = 0.6
 ##look forward

 ## look to an apple
 goal.target.point.x = 0.07
 goal.target.point.y = 2.75
 goal.target.point.z = 0.74
 ##look to an apple


 goal.min_duration = rospy.Duration(1.0)

 client = actionlib.SimpleActionClient('head_controller/point_head',PointHeadAction)
 wait = client.wait_for_server()
 
 if (wait):
  print ("succesfuly connected to action server")
  client.send_goal(goal)
  client.wait_for_result()
  print ("done")
 else:
  print ("failed to connect to action server")

 




if __name__=="__main__":
 rospy.init_node('head_pub',anonymous=False)
 point()
 rospy.spin()

	

