#!/usr/bin/env python
#-*- coding: utf-8 -*-
import time
import rospy
import actionlib
import tf
from geometry_msgs.msg import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Mover():
  def __init__(self):
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not self.ac.wait_for_server(rospy.Duration(5)):
      rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up")

    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

  def set_goal(self,pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    self.ac.send_goal(goal_pose)
    succeeded = self.ac.wait_for_result(rospy.Duration(60))
    state = self.ac.get_state()

    if succeeded:
      rospy.loginfo("Succeed")
    else:
      rospy.loginfo("Failed")

  def set_velocity(self,x,y,theta,sec):
    vel = Twist()
    vel.linear.x = x
    vel.linear.y = y
    vel.angular.z = theta

    end_time = time.time() + sec
    while(time.time() < end_time  and not rospy.is_shutdown()):
      self.vel_pub.publish(vel)

    vel.linear.x = 0
    vel.linear.y = 0
    vel.angular.z = 0
    self.vel_pub.publish(vel)

if __name__ == '__main__':
  rospy.init_node('mover_node')

  mv = Mover()
  try:
    mv.set_velocity(-0.1,0,0,2)
    mv.set_goal([(1.0,0.0,0.0),(0.0,0.0,0.0,1.0)])
  except rospy.ROSInterruptException:
    pass
