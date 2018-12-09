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
from control_msgs.msg import PointHeadAction, PointHeadGoal
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
 rospy.init_node('apple_fetch',anonymous=False)
 goal = MoveBaseGoal()
 goal.target_pose.header.frame_id='map'
 move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 wait = move_base_client.wait_for_server(rospy.Duration(5))


 head_point_client = actionlib.SimpleActionClient('head_controller/point_head',PointHeadAction)
 head_goal = PointHeadGoal()
 head_goal.target.header.stamp = rospy.Time.now()
 head_goal.target.header.frame_id = "base_link"
 ## look right
 head_goal.target.point.x = 0.3
 head_goal.target.point.y = -1.6
 head_goal.target.point.z = 0.8
 ##look right
 head_wait = head_point_client.wait_for_server()

 count = 0

 if (wait and head_wait):
  print ("succesfuly connected to action servers")
  while True:
   print ("count",count)
   #move fetch to start of orchard row
   goal.target_pose.header.stamp = rospy.Time.now()
   goal.target_pose.pose = Pose(Point(-1,0,0),Quaternion(0,0,1,1)) #row start position
   move_base_client.send_goal(goal,done_cb,active_cb,feedback_cb)
   move_base_client.wait_for_result()
   print ("reached row start")
   #make fetch look to the right
   head_goal.target.header.stamp = rospy.Time.now()
   head_goal.target.point.x = 0.3
   head_goal.target.point.y = -1.6
   head_goal.target.point.z = 0.8
   head_point_client.send_goal(head_goal)
   head_point_client.wait_for_result()
   print ("head turned right")
   #move fetch to end of orchard row
   goal.target_pose.header.stamp = rospy.Time.now()
   goal.target_pose.pose = Pose(Point(-1,9,0),Quaternion(0,0,1,1)) #row end position
   move_base_client.send_goal(goal,done_cb,active_cb,feedback_cb)
   move_base_client.wait_for_result()
   print ("reached row end")
   #make fetch look forward
   head_goal.target.header.stamp = rospy.Time.now()
   head_goal.target.point.x = 1
   head_goal.target.point.y = 0
   head_goal.target.point.z = 1
   head_point_client.send_goal(head_goal)
   head_point_client.wait_for_result()
   print ("head turned forward")
   #move fetch to start of orchard row
   goal.target_pose.header.stamp = rospy.Time.now()
   goal.target_pose.pose = Pose(Point(-1,0,0),Quaternion(0,0,1,1)) #row start position
   move_base_client.send_goal(goal,done_cb,active_cb,feedback_cb)
   move_base_client.wait_for_result()
   print ("reached row start")
   #goal.target_pose.pose = Pose(Point(-0.501,3.31,0),Quaternion(0,0,0,1))
   count += 1 
 
  rospy.spin()
 

 else:
  print ("failed to connect to action server")
  exit()

 




if __name__=="__main__":
 main()

	

