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
import time

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

global apples
apples = []
APPROACH_THRESH = 0.7

def find_groups(apples):
 THRESH = 0.2
 

 locMap = {}
 norms = []

 for app in apples:
  locMap[np.linalg.norm(app)] = app
  norms.append(np.linalg.norm(app))


 norms.sort()  
 norms = np.array(norms)
 appBak = norms

 #print len(appleLocs)
 #print appleLocs


 appleGroups = []
 groupCentroids = []


 i = 0
 while i < len(norms):
  temp = []
  #print ("starting new group with: ", appleLocs[i])
  temp.append(norms[i])
  j = i + 1
  while j < len(norms):
   #print (appleLocs[i],appleLocs[j])
   #print (i,j)
   if (abs(norms[i] - norms[j]) <= THRESH):
    #print "grouped"
    temp.append(norms[j])
   else:
    #print "too far, starting new group"
    appleGroups.append(temp)
    i = j-1
    break
  
   j += 1

   if j == len(norms):
    appleGroups.append(temp)
    break
  i += 1

 finalGroups = []
 print ("final groups: " + "(" + str(len(appleGroups)) + ")")
 for group in appleGroups:
  tempGroup = []
  print ("new")
  for item in group:
   print (locMap[item], item)
   tempGroup.append(locMap[item])
  finalGroups.append(tempGroup)


 return finalGroups

def done_cb(status, result):
 pass

def active_cb():
 pass

def feedback_cb(feedback):
 pass



def apple_seen_cb(data):
 print ("new apple seen")
 global apples
 apples.append([data.data[0],data.data[1],data.data[2]])


def main():
 rospy.init_node('apple_fetch',anonymous=False)
 sub = rospy.Subscriber("/apple_loc",numpy_msg(Floats),apple_seen_cb)
 goal = MoveBaseGoal()
 goal.target_pose.header.frame_id='map'
 move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 wait = move_base_client.wait_for_server(rospy.Duration(5))
 global apples

 head_point_client = actionlib.SimpleActionClient('head_controller/point_head',PointHeadAction)
 head_goal = PointHeadGoal()
 head_goal.target.header.stamp = rospy.Time.now()
 head_goal.target.header.frame_id = "base_link"
 ## look right
 head_goal.target.point.x = 0.3
 head_goal.target.point.y = -1.6
 head_goal.target.point.z = 1
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
   head_goal.target.header.frame_id = "base_link"
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
   head_goal.target.header.frame_id = "base_link"
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
   print ("I saw a total of: ", len(apples), " apples")
   goalGroups = find_groups(apples)
   #go to each group of apples
   gCount = 0
   for group in goalGroups:
    print ("going to appleGroup: " + str(gCount))
    #print (group[0])
    x = 0; y = 0; z = 0
    ptCt = 0
    for point in group:
     x += point[0]; y += point[1]; z += point[2]
     ptCt += 1
  
    x /= ptCt
    y /= ptCt
    z /= ptCt

    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x-APPROACH_THRESH,y,0),Quaternion(0,0,0,1)) #row end position
    move_base_client.send_goal(goal,done_cb,active_cb,feedback_cb)
    move_base_client.wait_for_result()
    print ("reached apple goal")
    gCount += 1
    head_goal.target.header.stamp = rospy.Time.now()
    head_goal.target.header.frame_id = "map"
    head_goal.target.point.x = x
    head_goal.target.point.y = y
    head_goal.target.point.z = z
    head_point_client.send_goal(head_goal)
    head_point_client.wait_for_result()
    time.sleep(5)



  rospy.spin()
 

 else:
  print ("failed to connect to action server")
  exit()

 




if __name__=="__main__":
 main()

	

