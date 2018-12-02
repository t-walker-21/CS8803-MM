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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


def done_cb(status, result):
 pass

def active_cb():
 pass

def feedback_cb(feedback):
 pass


def main():
 rospy.init_node('goal_pub',anonymous=False)
 goal = MoveBaseGoal()
 goal.target_pose.header.frame_id='map'
 goal.target_pose.header.stamp = rospy.Time.now()
 goal.target_pose.pose = Pose(Point(-0.57,2.75,0),Quaternion(0,0,0,1))
 # goal.target_pose.pose = Pose(Point(0,0,0),Quaternion(0,0,1,1)) home position
 client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 wait = client.wait_for_server(rospy.Duration(5))
 
 if (wait):
  print ("succesfuly connected to action server")
  client.send_goal(goal,done_cb,active_cb,feedback_cb)
  
 else:
  print ("failed to connect to action server")

 rospy.spin()




if __name__=="__main__":
 main()

	

