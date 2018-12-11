#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson
# Author: Di Sun

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from grasping_msgs.msg import GraspPlanningAction, GraspPlanningGoal, Object
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, Grasp
from std_msgs.msg import String

# Tools for grasping
class GraspingClient(object):
    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        grasp_topic = "fetch_grasp_planner_node/plan"
        rospy.loginfo("Waiting for %s..." % grasp_topic)
        self.grasp_planner_client = actionlib.SimpleActionClient(grasp_topic, GraspPlanningAction)
        self.grasp_planner_client.wait_for_server()
        rospy.Subscriber("fetch_fruit_harvest/apple_pose", Pose, self.apple_pose_callback)
        self.pub = rospy.Publisher("fetch_fruit_harvest/grasp_msg", String, queue_size=10)
        self.object = Object()
        self.grasps = Grasp()
        self.ready_for_grasp = False
        self.pick_place_finished = False
        self.first_time_grasp = True
        self.scene.addBox("ground", 300, 300, 0.02, 0, 0, 0)
        self.tuck()

    def apple_pose_callback(self, message):
        if self.pick_place_finished or self.first_time_grasp:
            self.first_time_grasp = False
            self.pick_place_finished = False
            print("apple_pose_callback")
            apple = SolidPrimitive()
            apple.type = SolidPrimitive.SPHERE
            apple.dimensions = [0.03, 0.03, 0.03]
            self.object.name = "apple"
            self.object.primitives = [apple]
            self.object.primitive_poses = [message]
            # add stamp and frame
            self.object.header.stamp = rospy.Time.now()
            self.object.header.frame_id = "base_link"
        
            goal = GraspPlanningGoal()
            goal.object = self.object
            self.grasp_planner_client.send_goal(goal)
            self.grasp_planner_client.wait_for_result(rospy.Duration(5.0))
            grasp_planner_result = self.grasp_planner_client.get_result()
            self.grasps = grasp_planner_result.grasps
            self.ready_for_grasp = True

    def updateScene(self):
        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        self.scene.addBox("ground", 300, 300, 0.02, 0, 0, 0)
        self.scene.addSolidPrimitive(self.object.name,
                                     self.object.primitives[0],
                                     self.object.primitive_poses[0],
                                     use_service = False)
        self.scene.waitForSync()

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def intermediate_stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
        
if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    grasping_client = GraspingClient()

    rate = rospy.Rate(0.1)
    apple_in_grapper = False

    while not rospy.is_shutdown():
        # Get apple to pick
        fail_ct = 0
        failed = False
        if not grasping_client.ready_for_grasp:
            continue

        grasping_client.stow()
        while not rospy.is_shutdown() and not apple_in_grapper:
            rospy.loginfo("Picking object...")
            grasping_client.updateScene()
            # Pick the apple
            if grasping_client.pick(grasping_client.object, grasping_client.grasps):
                apple_in_grapper = True
                break
            rospy.logwarn("Grasping failed.")
            grasping_client.stow()
            if fail_ct > 15:
                fail_ct = 0
                failed = True
                break
            fail_ct += 1

        # Place the apple
        while not rospy.is_shutdown() and apple_in_grapper:
            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = grasping_client.object.primitive_poses[0]
            pose.pose.position.y *= -1.0
            pose.pose.position.z += 0.02
            pose.header.frame_id = grasping_client.object.header.frame_id
            if grasping_client.place(grasping_client.object, pose):
                apple_in_grapper = False
                break
            rospy.logwarn("Placing failed.")
            grasping_client.intermediate_stow()
            grasping_client.stow()
            if fail_ct > 15:
                fail_ct = 0
                failed = True
                break
            fail_ct += 1

        # Tuck the arm, lower the torso
        grasping_client.tuck()
        if failed:
            done_str = "Failed in grasping at %s" % rospy.get_time()
            grasping_client.pub.publish(done_str)
            rospy.loginfo("Failed")
        else:
            done_str = "Succeed in grasping at %s" % rospy.get_time()
            grasping_client.pub.publish(done_str)
            rospy.loginfo("Succeed")

        grasping_client.pick_place_finished = True
        grasping_client.ready_for_grasp = False
        rate.sleep()
