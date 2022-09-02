#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
##-- for navigation
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import yaml
import tf
import math
##-- for find pkg
import rospkg

##-- for dynamic reconfigure
import dynamic_reconfigure.client

##--- follow joint trajectory conntroller
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
##---- joint state 
from sensor_msgs.msg import JointState

###########################################
## @brief ナビゲーション関連のクラス
class NaviAction:
  ## @brief コンストラクタ。waypointsの読込とmove_baseのアクションクライアントの定義
  def __init__(self):
    rospack = rospkg.RosPack()
    rospack.list() 
    self.path = rospack.get_path('task_programmer')
    ## @brief 読み込まれたwaypointsのデータ
    self.wp_client = dynamic_reconfigure.client.Client("/waypoints_editor", timeout=30)
    # self.config = yaml.load(file(path + "/config/waypoints.yaml"))
    rospy.on_shutdown(self.shutdown)
    ## @brief /move_baseアクションクライアント
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not self.ac.wait_for_server(rospy.Duration(5)):
      rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up")
    ## @brief MoveBaseGoal型のゴール
    self.goal = MoveBaseGoal()

    self.vel_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.vel_ = Twist()

  ## @brief ゴールポジションの設定と移動の開始
  # @param _number waypointsの番号(0以上の数値）
  # @return ゴールに到着したか否か（succeeded or aborted）
  def set_goal(self,_number):
    rospy.on_shutdown(self.shutdown)

    dir_name = self.wp_client.get_configuration(timeout=5)['dir_name']
    file_name = self.wp_client.get_configuration(timeout=5)['file_name']
    with open(self.path + dir_name + '/' + file_name + '.yaml') as f:
        config = yaml.safe_load(f) or {}

    if(_number >= len(config)):
      rospy.logerr("wp number is not exist.")
      return 'aborted'

    rev = dict(config[_number]) #List to Dictionary

    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = rev['pose']['position']['x']
    self.goal.target_pose.pose.position.y = rev['pose']['position']['y']
    self.goal.target_pose.pose.position.z = rev['pose']['position']['z']
    self.goal.target_pose.pose.orientation.x = rev['pose']['orientation']['x']
    self.goal.target_pose.pose.orientation.y = rev['pose']['orientation']['y']
    self.goal.target_pose.pose.orientation.z = rev['pose']['orientation']['z']
    self.goal.target_pose.pose.orientation.w = rev['pose']['orientation']['w']

    rospy.loginfo('Sending goal')
    self.ac.send_goal(self.goal)
    succeeded = self.ac.wait_for_result(rospy.Duration(60))
    state = self.ac.get_state()
    if succeeded:
      rospy.loginfo("Succeed")
      return 'succeeded'
    else:
      rospy.loginfo("Failed")
      return 'aborted'

  ## @brief 経由地点の設定と移動の開始
  # @param _number waypointsの番号(0以上の数値）
  # @return ゴール付近に到着したか否か（succeeded or aborted）
  def set_via_point(self,_number):
    rospy.on_shutdown(self.shutdown)

    dir_name = self.wp_client.get_configuration(timeout=5)['dir_name']
    file_name = self.wp_client.get_configuration(timeout=5)['file_name']
    with open(self.path + dir_name + '/' + file_name + '.yaml') as f:
        config = yaml.safe_load(f) or {}

    if(_number >= len(config)):
      rospy.logerr("wp number is not exist.")
      return 'aborted'
    rev = dict(config[_number]) #List to Dictionary

    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = rev['pose']['position']['x']
    self.goal.target_pose.pose.position.y = rev['pose']['position']['y']
    self.goal.target_pose.pose.position.z = rev['pose']['position']['z']
    self.goal.target_pose.pose.orientation.x = rev['pose']['orientation']['x']
    self.goal.target_pose.pose.orientation.y = rev['pose']['orientation']['y']
    self.goal.target_pose.pose.orientation.z = rev['pose']['orientation']['z']
    self.goal.target_pose.pose.orientation.w = rev['pose']['orientation']['w']

    rospy.loginfo('Sending goal')
    self.ac.send_goal(self.goal)
    listener = tf.TransformListener()

    timeout = time.time() + 10 #[sec]

    while True:
      try:
        (position, quaternion) = listener.lookupTransform('map', 'base_link', rospy.Time(0) )
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      #ゴールから0.5[m]以内なら、succeededを返す
      if(math.sqrt((position[0]-self.goal.target_pose.pose.position.x)**2 
        + (position[1]-self.goal.target_pose.pose.position.y)**2 ) <= 0.5):
        
        rospy.loginfo("Succeed")
        return 'succeeded'

      elif (time.time() > timeout):
        rospy.loginfo("Timeout")
        return 'aborted' 

  if(len(sys.argv) != 2): waypoint = 0
  else:
        rospy.sleep(0.5)

  def set_velocity(self,_x,_y,_theta,_time):
    self.vel_.linear.x = _x
    self.vel_.linear.y = _y
    self.vel_.angular.z = _theta

    end_time = time.time() + _time #[sec]

    while(time.time() < end_time):
      self.vel_pub_.publish(self.vel_)

    self.vel_.linear.x = 0
    self.vel_.linear.y = 0
    self.vel_.angular.z = 0

    self.vel_pub_.publish(self.vel_)

    return 'succeeded'

  def cancel(self):
    self.ac.cancel_all_goals()
  
  ## @brief move_baseの終了
  def shutdown(self):
    self.ac.cancel_goal()

###########################################
class TrajectoryAction:
  def __init__(self):
    self.pub = rospy.Publisher('lifter_controller/command', JointTrajectory, queue_size=10)
    self.msg = JointTrajectory()
    self.msg.joint_names = ["ankle_joint","knee_joint"]
    self.msg.points = [JointTrajectoryPoint() for i in range(1)]
    rospy.sleep(0.5)

  def set_lifter_goal(self, _pos):
    lifter_position_str = (_pos).split(',')
    lifter_position = [float(s) for s in lifter_position_str]

    self.msg.header.stamp = rospy.Time.now()
    self.msg.points[0].positions = [math.radians(lifter_position[0]), -math.radians(lifter_position[1])]
    self.msg.points[0].time_from_start = rospy.Time(lifter_position[2]/1000)

    self.pub.publish(self.msg)

#==================================
#==================================
if __name__ == '__main__':
  rospy.init_node('got_to_waypoint_server')

  na = NaviAction()
  ta = TrajectoryAction()

  if(len(sys.argv) < 2):
    na.cancel()
  elif(len(sys.argv) < 3):
    waypoint = int(sys.argv[1])
    na.set_goal(int(waypoint))
  else:
    if(int(sys.argv[1]) != -1):
      waypoint = int(sys.argv[1])
      na.set_goal(int(waypoint))
    else:
      rospy.loginfo("lifter move only")
    ta.set_lifter_goal(sys.argv[2])
