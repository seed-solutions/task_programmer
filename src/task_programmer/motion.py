#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
import copy
import tf
import time
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3

##-- for hand control
from seed_r7_ros_controller.srv import*

#################################################
#Main Part
#################################################

class Motion():
  def __init__(self):
    #Configuration for MoveIt!
    robot = moveit_commander.RobotCommander()

    #Store the objects of each group name
    self.upper_body = moveit_commander.MoveGroupCommander("upper_body")
    self.lifter = moveit_commander.MoveGroupCommander("lifter")
    self.rarm_with_torso = moveit_commander.MoveGroupCommander("rarm_with_torso")

    self.upper_body.set_max_velocity_scaling_factor(1)
    self.lifter.set_max_velocity_scaling_factor(1)

    rospy.loginfo('waiting service')
    rospy.wait_for_service('/seed_r7_ros_controller/hand_control')
    self.service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)

  def grasp(self,on):
    try:
      if(on):
        response = self.service(0,'grasp',100)
        #response = self.service(1,'grasp',100)
      else:
        response = self.service(0,'release',100)
        #response = self.service(1,'release',100)
      return True
    except rospy.ServiceException as e:
      rospy.logerr('Service call failed: {}'.format(e))
      return False

  def init_pose(self,lifter=True,top=True):
    #Set Pose to Home Position
    joint_length = len(self.upper_body.get_current_joint_values())
    self.upper_body.set_joint_value_target(joint_length * [0]) #all joints are initialized at 0
    self.upper_body.set_joint_value_target('r_elbow_joint',-2.8)
    self.upper_body.set_joint_value_target('l_elbow_joint',-2.8)
    self.upper_body.go(wait=True)

    if(lifter and top):
      self.lifter.set_joint_value_target('ankle_joint',0)
      self.lifter.set_joint_value_target('knee_joint',0)
      self.lifter.go(wait=True)
    elif(lifter and not top):
      self.lifter.set_joint_value_target('ankle_joint',1.4)
      self.lifter.set_joint_value_target('knee_joint',-1.4)
      self.lifter.go(wait=True)

  def hello(self):
    self.upper_body.set_joint_value_target('r_elbow_joint',-2.0)
    self.upper_body.set_joint_value_target('l_elbow_joint',-2.8)
    self.upper_body.set_joint_value_target('r_shoulder_p_joint',-0.9)
    self.upper_body.set_joint_value_target('r_shoulder_r_joint',0)
    self.upper_body.set_joint_value_target('r_shoulder_y_joint',-0.5)
    self.upper_body.go(wait=True)

    self.upper_body.set_joint_value_target('waist_y_joint',0.5)
    self.upper_body.go(wait=True)
    for i in range(0,2):
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',0.3)
      self.upper_body.go(wait=True) 
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',-0.3)
      self.upper_body.go(wait=True) 

    self.upper_body.set_joint_value_target('waist_y_joint',-0.5)
    self.upper_body.go(wait=True)
    for i in range(0,2):
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',0.3)
      self.upper_body.go(wait=True) 
      self.upper_body.set_joint_value_target('r_shoulder_y_joint',-0.3)
      self.upper_body.go(wait=True)

    self.init_pose(lifter=False)

  def set_grasp_position(self, x, y, z, vel=1.0,direction="side"):
    self.group = moveit_commander.MoveGroupCommander("rarm_with_torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_planner_id( "RRTConnectkConfigDefault" )
    self.group.allow_replanning( True )

    target_pose = Pose()
    if(direction == "side"):
      self.group.set_end_effector_link("r_eef_grasp_link")
      quat = tf.transformations.quaternion_from_euler(0,0,0)
    elif(direction == "top"): 
      self.group.set_end_effector_link("r_eef_pick_link")
      quat = tf.transformations.quaternion_from_euler(-1.57,0.79,0)

    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]

    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(vel)
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'
      

  def work(self):
    box1 = [0.6,-0.3,0.36]
    box2 = [0.6,0.3,0.76]
    box3 = [0.6,0,1.16]
    self.set_grasp_position(box1[0], box1[1], box1[2], direction="top")
    self.grasp(True)
    time.sleep(2)
    self.set_grasp_position(box1[0], -0.4, box2[2], direction="top")
    self.grasp(False)
    self.set_grasp_position(box2[0], box2[1], box2[2])
    self.grasp(True)
    time.sleep(2)
    self.set_grasp_position(box2[0], -0.2, box2[2])
    self.grasp(False)
    time.sleep(2)
    
    self.init_pose(top=False)

  def run(self):
    rospy.loginfo('motion start')
    self.init_pose()
    self.grasp(True)

    #腕上げる
    self.upper_body.set_joint_value_target('r_elbow_joint',-1.5)
    self.upper_body.set_joint_value_target('r_shoulder_p_joint',0)
    self.upper_body.set_joint_value_target('r_shoulder_r_joint',-1.57)
    self.upper_body.set_joint_value_target('r_shoulder_y_joint',0)

    self.upper_body.set_joint_value_target('l_elbow_joint',-1.5)
    self.upper_body.set_joint_value_target('l_shoulder_p_joint',0)
    self.upper_body.set_joint_value_target('l_shoulder_r_joint',1.57)
    self.upper_body.set_joint_value_target('l_shoulder_y_joint',0)
    self.upper_body.go(wait=True)

    #腕回す
    self.upper_body.set_joint_value_target('r_shoulder_y_joint',-1.57)
    self.upper_body.set_joint_value_target('l_shoulder_y_joint',1.57)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('r_shoulder_y_joint',1.57)
    self.upper_body.set_joint_value_target('l_shoulder_y_joint',-1.57)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('r_shoulder_y_joint',0)
    self.upper_body.set_joint_value_target('l_shoulder_y_joint',0)
    self.upper_body.go(wait=True)

    #腕おろす
    self.upper_body.set_joint_value_target('r_elbow_joint',-1.5)
    self.upper_body.set_joint_value_target('r_shoulder_p_joint',0)
    self.upper_body.set_joint_value_target('r_shoulder_r_joint',0)
    self.upper_body.set_joint_value_target('r_shoulder_y_joint',0)

    self.upper_body.set_joint_value_target('l_elbow_joint',-1.5)
    self.upper_body.set_joint_value_target('l_shoulder_p_joint',0)
    self.upper_body.set_joint_value_target('l_shoulder_r_joint',0)
    self.upper_body.set_joint_value_target('l_shoulder_y_joint',0)
    self.upper_body.go(wait=True)

    #手首回す
    self.upper_body.set_joint_value_target('r_wrist_y_joint',-1.57)
    self.upper_body.set_joint_value_target('l_wrist_y_joint',1.57)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('r_wrist_y_joint',1.57)
    self.upper_body.set_joint_value_target('l_wrist_y_joint',-1.57)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('r_wrist_y_joint',0)
    self.upper_body.set_joint_value_target('l_wrist_y_joint',0)
    self.upper_body.go(wait=True)

    #手首曲げる
    self.upper_body.set_joint_value_target('r_wrist_r_joint',0.8)
    self.upper_body.set_joint_value_target('l_wrist_r_joint',-0.8)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('r_wrist_r_joint',-0.34)
    self.upper_body.set_joint_value_target('l_wrist_r_joint',0.34)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('r_wrist_r_joint',0)
    self.upper_body.set_joint_value_target('l_wrist_r_joint',0)
    self.upper_body.go(wait=True)

    #腰曲げる
    self.upper_body.set_joint_value_target('waist_p_joint',0.69)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('waist_p_joint',-0.17)
    self.upper_body.go(wait=True)
    self.upper_body.set_joint_value_target('waist_p_joint',0)
    self.upper_body.go(wait=True)

    # 手先位置移動  
    self.rarm_with_torso.set_end_effector_link("r_eef_grasp_link")
    self.rarm_with_torso.set_pose_reference_frame("base_link")
    pose_goal = Pose()
    pose_goal.orientation.x = -0.707
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0.707
    pose_goal.position.x = 0.8                              
    pose_goal.position.y = 0
    pose_goal.position.z = 0.5

    self.rarm_with_torso.set_pose_target(pose_goal)

    plan = self.rarm_with_torso.plan()
    if type(plan) is tuple: # for noetic
      plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.rarm_with_torso.clear_pose_targets()
    else: 
      self.rarm_with_torso.execute(plan)

    self.init_pose(lifter=False)
    self.grasp(False)

    #リフター曲げる
    self.lifter.set_joint_value_target('ankle_joint',1.57)
    self.lifter.set_joint_value_target('knee_joint',-1.57)
    self.lifter.go(wait=True)

    rospy.loginfo('motion end')


if __name__ == '__main__':
  rospy.init_node('motion_node')

  motion = Motion()

  try:
    motion.init_pose(top=False)
    motion.hello()
    motion.work()
  except rospy.ROSInterruptException:
    pass
