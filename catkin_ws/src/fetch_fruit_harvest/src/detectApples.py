#!/usr/bin/env python

from __future__ import print_function

import roslib
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
from geometry_msgs.msg import Pose
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class image_converter:

 def __init__(self):
  self.rgb_image_sub = message_filters.Subscriber('/head_camera/rgb/image_raw/',Image)
  self.depth_image_sub = message_filters.Subscriber('/head_camera/depth_registered/image_raw/',Image)

  self.ts = message_filters.TimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],10)
  self.ts.registerCallback(self.callback)
  self.apple_pub = rospy.Publisher("/apple_loc",numpy_msg(Floats), queue_size=10)
  self.bridge = CvBridge()
  self.applesSeen = []
  self.tf_listener_ = TransformListener()
  self.tx = tf.Transformer(True,rospy.Duration(10.))
  #self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.callback)

 def callback(self,RGBdata,depthData):
  
  try:
   cv_rgb_image = self.bridge.imgmsg_to_cv2(RGBdata,"bgr8")
   cv_depth_image = self.bridge.imgmsg_to_cv2(depthData)
   
  except CvBridgeError as e:
   print (e)

  
  depth_mask = cv2.inRange(cv_depth_image,0.01,2)
  
  
  #cv2.waitKey(1)
  
  hsv_image = cv2.cvtColor(cv_rgb_image,cv2.COLOR_BGR2HSV)
  color_mask = cv2.inRange(hsv_image[:,:,1],245,255)
  cv2.imshow('image',color_mask)
  
  mask = color_mask & depth_mask

  #t = self.tf_listener_.getLatestCommonTime("base_link","head_camera_link")
  p1 = geometry_msgs.msg.PoseStamped()
  p1.header.frame_id = "base_link"
  p1.pose.orientation.w = 1.0
  #pBase = self.tf_listener_.transformPose("head_camera_link",p1)
  
  now = rospy.Time(0)
  self.tf_listener_.waitForTransform("map", "head_camera_link", now, rospy.Duration(4.0)) 
  trans = self.tf_listener_.lookupTransform("map","head_camera_link",now)  

  ##better method
 
  txformer = tf.TransformerROS()
  mat = txformer.fromTranslationRotation(trans[0],trans[1])
  
  ##better method



  #print ("pose: ",trans[0])

  _, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

  # Find the index of the largest contour
  for c in contours:
   if cv2.contourArea(c) < 40:
    continue
   x,y,w,h = cv2.boundingRect(c)
   cv2.rectangle(cv_rgb_image,(x,y),(x+w,y+h),(0,255,0),2)
   #print ((x + w/2),(y + h/2))
   apple_center_depth = cv_depth_image[y+h/2][x+w/2] #+ 0.158
   #print (apple_center_depth)
     
   z_coord = apple_center_depth
   x_coord = -1*(apple_center_depth/554.0)*(x-320 + w/2)
   y_coord = (apple_center_depth/554.0)*(-1*(y-240) + h/2)

   apple_coordinate = np.array([z_coord,x_coord,y_coord,1])
   #apple_coordinate = np.array([0,0,0,1])
   #print (mat)
   #print ("\n")
   #print(np.dot(mat,apple_coordinate))
   apple_coordinate = np.dot(mat,apple_coordinate)
   #apple_coordinate = apple_coordinate - np.array(trans[0])
   #print ("appleWC: ",apple_coordinate)
   rare = True
   if len(self.applesSeen) == 0:
    self.applesSeen.append(apple_coordinate)
    print ("new apple added")
    print (apple_coordinate)
    self.apple_pub.publish(apple_coordinate)

   else:

    for loc in self.applesSeen:
     if np.linalg.norm(np.array(apple_coordinate)-np.array(loc)) <= 0.1:
      rare = False
    
    if rare:
     self.applesSeen.append(apple_coordinate)
     print ("new apple added")
     print (apple_coordinate)
     self.apple_pub.publish(apple_coordinate)
   
     cv2.circle(cv_rgb_image,((x + w/2),(y + h/2)),5,(0,0,0),3,8,0)

    else:
     cv2.circle(cv_rgb_image,((x + w/2),(y + h/2)),5,(255,0,0),3,8,0)

  cv2.imshow("rosIm",cv_rgb_image)

  cv2.waitKey(1)



def main():
 rospy.init_node('apple_detector',anonymous=False)
 ic = image_converter()
 rospy.spin()
 cv2.destroyAllWindows()




if __name__=="__main__":
 main()

	
