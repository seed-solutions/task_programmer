#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
import copy
import subprocess
##-- for smach
from smach import State,StateMachine
import smach_ros
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
##-- for moveit
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
##-- for service call
from task_programmer.srv import *
from seed_r7_ros_controller.srv import *
from std_srvs.srv import*
##-- for dynamic reconfigure
import dynamic_reconfigure.client

###########################################
## @brief ナビゲーション関連のクラス
class NaviAction:
  ## @brief コンストラクタ。waypointsの読込とmove_baseのアクションクライアントの定義
  def __init__(self):
    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('task_programmer')
    ## @brief 読み込まれたwaypointsのデータ
    self.config = yaml.load(file(path + "/config/waypoints.yaml"))
    rospy.on_shutdown(self.shutdown)
    ## @brief /move_baseアクションクライアント
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not self.ac.wait_for_server(rospy.Duration(5)):
      rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up");
    ## @brief MoveBaseGoal型のゴール
    self.goal = MoveBaseGoal()

    self.vel_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.vel_ = Twist()

    self.tf_listener_ = tf.TransformListener()

  ## @brief ゴールポジションの設定と移動の開始
  # @param _number waypointsの番号(0以上の数値）
  # @return ゴールに到着したか否か（succeeded or aborted）
  def set_goal(self,_number):
    rospy.on_shutdown(self.shutdown)

    rev = dict(self.config[_number]) #List to Dictionary

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
    succeeded = self.ac.wait_for_result(rospy.Duration(60));
    state = self.ac.get_state();
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

    rev = dict(self.config[_number]) #List to Dictionary

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

    timeout = time.time() + 60 #[sec]

    while True:
      try:
        (position, quaternion) = self.tf_listener_.lookupTransform('map', 'base_link', rospy.Time(0) )
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      distance = math.sqrt((position[0]-self.goal.target_pose.pose.position.x)**2 \
        + (position[1]-self.goal.target_pose.pose.position.y)**2 )

      #ゴールから0.5[m]以内なら、succeededを返す
      if(distance <= 0.5):
        rospy.loginfo("Succeed")
        return 'succeeded'

      elif (time.time() > timeout):
        rospy.loginfo("Timeout")
        return 'aborted' 

      else:
        pass
        
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

  def set_pose(self,_number):
    rev = dict(self.config[_number]) #List to Dictionary

    self.x_ = rev['pose']['position']['x']
    self.y_ = rev['pose']['position']['y']
    (self.roll_,self.pitch_,self.yaw_) = tf.transformations.euler_from_quaternion(
      (rev['pose']['orientation']['x'], rev['pose']['orientation']['y'], 
       rev['pose']['orientation']['z'], rev['pose']['orientation']['w']))
    
    try:
      rospy.loginfo("set initialpose at %s,%s,%s" % (self.x_,self.y_,self.yaw_))
      set_initialpose = rospy.ServiceProxy('set_initialpose', SetInitialPose)
      res = set_initialpose(self.x_, self.y_, self.yaw_)
      rospy.sleep(1)
      return 'succeeded'

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return 'aborted'
        
  ## @brief move_baseの終了
  def shutdown(self):
    #ta.set_led(10,2)
    #ta.set_led(11,2)
    #rospy.loginfo("The robot was terminated")
    self.ac.cancel_goal()
#--------------------------------
## @brief ”ムーバー移動”ステート
# @param State smachのステートクラスを継承
class GO_TO_PLACE(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _place  waypointsの番号
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief waypointsの番号
    self.place_ = _place

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return ゴールに到着したか否か（succeeded or aborted）
  def execute(self, userdata):
    #ta.set_led(10,5)
    #ta.set_led(11,5)
    print 'Going to Place'+str(self.place_)
    if(na.set_goal(self.place_) == 'succeeded'):return 'succeeded'
    else: return 'aborted' 

## @brief ”経由点移動”ステート
# @param State smachのステートクラスを継承
class GO_TO_VIA_POINT(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _place  waypointsの番号
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief waypointsの番号
    place_string = _place.split(',')
    self.place_ = [int(s) for s in place_string]

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return ゴールに付近に到着したか否か（succeeded or aborted）
  def execute(self, userdata):
    #ta.set_led(10,5)
    #ta.set_led(11,5)
    print 'Going to Place'+str(self.place_)
    place_counter = self.place_[0]
    while(place_counter < self.place_[1]):
      na.set_via_point(place_counter)
      place_counter += 1

    if(na.set_via_point(self.place_[1]) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

class SET_POSE(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _place  waypointsの番号
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief waypointsの番号
    self.place_ = _place

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return initailposeを設定できたか否か（succeeded or aborted）
  def execute(self, userdata):
    print 'set initial pose at'+str(self.place_)
    if(na.set_pose(self.place_) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

class VELOCITY_MOVE(State):
  def __init__(self,_velocity):
    State.__init__(self, outcomes=['succeeded','aborted'])
    velocity_string = _velocity.split(',')
    self.vel_ = [float(s) for s in velocity_string]

  def execute(self, userdata):
    #ta.set_led(10,6)
    #ta.set_led(11,6)
    print 'velocity move ' + str(self.vel_[0]) +',' + str(self.vel_[1]) + ',' +\
      str(self.vel_[2])  + ') in time ' + str(self.vel_[3])

    if(na.set_velocity(self.vel_[0],self.vel_[1],self.vel_[2],self.vel_[3]) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------

###########################################
## @brief Moveit Commander用のクラス
# moveit_commanderのライブラリを読みだす
class MoveitCommand:
  ## @brief コンストラクタ。モデルの読み出し、グループの定義
  def __init__(self):
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    print "set_group"
    self.group = moveit_commander.MoveGroupCommander("torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_end_effector_link("body_link")

    self.robot_model = rospy.get_param("/seed_r7_ros_controller/robot_model_plugin")

  ## @brief lifterのIK算出と動作
  # @param _x リフター上部のx座標[m]
  # @param _z リフター上部のz座標[m]
  # @param _vel 速度のスケーリング : 0〜１
  # @return IKが算出できて、移動が完了したか否か（succeeded or aborted）
  def set_lifter_position(self, _x, _z, _vel=1.0):

    if("typef" in self.robot_model or "TypeF" in self.robot_model):
      distance_body_to_lifter_top = 1.065 - 0.92
    elif("typeg" in self.robot_model or "TypeG" in self.robot_model):
      distance_body_to_lifter_top = 0.994 - 0.857

    target_pose = Pose()

    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.orientation.w = 1

    target_pose.position.x = _x
    target_pose.position.y = 0
    target_pose.position.z = _z + distance_body_to_lifter_top

    #self.group.set_start_state_to_current_state()
    print "set pose"
    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(_vel)

    print "plan stat"
    plan = self.group.plan()

    print "check plan"
    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("can't be solved lifter ik")
      self.group.clear_pose_targets()
      return 'aborted'
    else:
      print "execute plan"
      self.group.execute(plan)
      self.group.clear_pose_targets()
      return 'succeeded'

#---------------------------------
## @brief ”リフター移動”ステート 
# @param State smachのステートクラスを継承
class LIFTER(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _position リフターの絶対位置(x,y)[m]と速度スケーリング(0~1)
  def __init__(self,_position):
    State.__init__(self, outcomes=['succeeded','aborted'])
    position_string = _position.split(',')
    ## @brief リフターの絶対位置(x,y)[m]と速度スケーリング(0~1)
    self.position_ = [float(s) for s in position_string]

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return サービスの呼び出し結果（succeeded or aborted）
  def execute(self, userdata):
    #ta.set_led(10,3)
    #ta.set_led(11,3)
    print 'Move Lifter at (' + str(self.position_[0]) +',' + \
      str(self.position_[1]) +') in scale velocity ' + str(self.position_[2])
    if(mc.set_lifter_position(self.position_[0],self.position_[1],self.position_[2]) == 'succeeded'):return 'succeeded'
    else: return 'aborted'


    
###########################################
## @brief ナビゲーション以外のタスク実行クラス
# task_controllerサーバー( TaskController.hh )とサービスコールで通信を行う
class TaskAction:
  ## @brief コンストラクタ。task_controllerサーバーの起動を待つ
  def __init__(self):
    rospy.loginfo('waiting task_controller service')
    rospy.wait_for_service('task_controller')
    rospy.loginfo('waiting led_control service')
    rospy.wait_for_service('led_control')    

  ## @brief タスクの設定と実行を行う。task_controllerクライアントを作成し、サーバーへサービスを呼び出す。@n
  # 型の定義は TaskController.srv を参照のこと
  # @warning タスク名以外の引数は不要であっても何か（0など）を入力すること
  # @param _task タスク名(mover, lifter, wait)
  # @param _place waypoints番号
  # @param _lifter_position リフターの位置と移動時間
  # @param _time 待ち時間
  # @return サービスの結果
  def set_action(self,_task,_marker):
    try:
      service = rospy.ServiceProxy('task_controller', TaskController)
      response = service(_task,_marker)
      return response.result
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def set_led(self, _send_number, _script_number):
    try:
      service = rospy.ServiceProxy('led_control', LedControl)
      response = service(_send_number,_script_number)
      return response.result
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

#---------------------------------
class GO_TO_MARKER(State):
  def __init__(self,_marker):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.marker_ = str(_marker)

  def execute(self, userdata):
    #ta.set_led(10,4)
    #ta.set_led(11,4)
    print 'Go to at (' + self.marker_ +')'
    if(ta.set_action("marker",self.marker_) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

class TURN_ON_LED(State):
  def __init__(self,_led):
    State.__init__(self, outcomes=['succeeded','aborted'])
    led_string = _led.split(',')
    self.led_ = [int(s) for s in led_string]

  def execute(self, userdata):
    print 'Turn on ' + str(self.led_[0])  + ' script at ' + str(self.led_[1])
    if(ta.set_led(self.led_[0],self.led_[1]) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

##########################################
## @brief ”待ち”ステート 
# @param State smachのステートクラスを継承
class WAIT(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _time 待ち時間[msec]
  def __init__(self,_time):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief 待ち時間[msec]
    self.time_ = int(_time)

  ## @brief 遷移実行
  # @warning 待ち時間が-1の時はrosparam'/task_programmer/wait_task'がTrueになるまで無限待機する
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return succeededのみ
  def execute(self, userdata):
    #ta.set_led(10,2)
    #ta.set_led(11,2)
    if(self.time_ >= 0):
      print 'wait ' + str(self.time_) + 'msec'
      rospy.sleep(self.time_ * 0.001)
      rospy.set_param('/task_programmer/jump', 1)
    else :
      print 'wait for /task_programmer/wait_task is False'
      rospy.set_param('/task_programmer/wait_task',True)
      while(rospy.get_param('/task_programmer/wait_task') == True): pass

    rospy.set_param('/task_programmer/wait_task',False)

    if (rospy.get_param('/task_programmer/jump') == -1):
      rospy.logwarn("run previous task")
      return 'aborted'
    else : return 'succeeded'

##########################################
## @brief ”ループ”ステート 
# @param State smachのステートクラスを継承
class LOOP(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _count ループ回数。負の値だと、無限ループ
  def __init__(self,_count):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief ループ回数。負の値だと、無限ループ
    sn.loop_count_array[_count[0]] = int(_count[1])
    self.count_array_ = copy.copy(sn.loop_count_array)

    self.count_  = _count

    print "init count is %s \t" % sn.loop_count_array

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return ループがある場合はsucceeded、無い場合はaborted
  def execute(self, userdata):
    count = self.count_array_[self.count_[0]]
    #print "count is %s \t" % count
    #print "count_array_ is %s \t" % self.count_array_
    #print "loop_count_array_ is %s \t" % sn.loop_count_array
    count -= 1

    if(count > 0):
      print 'count ' + str(count) + ' times'
      self.count_array_[self.count_[0]] = count
      return 'succeeded'
    elif(count < -1):
      print 'infinity loop'
      return 'succeeded'
    else:
      print 'reset count'
      self.count_array_[self.count_[0]] = sn.loop_count_array[self.count_[0]]
      #print "count_array_ is %s \t" % self.count_array_
      #print "loop_count_array_ is %s \t" % sn.loop_count_array
      return 'aborted'

##########################################
## @brief ”終了”ステート 
# @param State smachのステートクラスを継承
class FINISH(State):
  ## コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return succeededのみ
  def execute(self, userdata):
    print 'FINISHED'
    return 'succeeded'

##########################################
## @brief ”何もしない”ステート 実装されていないタスクが実行された場合のみ使用
# @param State smachのステートクラスを継承
class NONE(State):
  ## @briefコンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return succeededのみ
  def execute(self, userdata):
    return 'succeeded'

class ROS_CLEAN(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  def execute(self, userdata):
    pwd = 'seed'
    cmd = 'find /var/log/ -type f -name \* -exec cp -f /dev/null {} \;'
    subprocess.call('echo {} | sudo -S {}'.format(pwd,cmd), shell=True)
    #subprocess.call("rosclean purge -y", shell=True)
    return 'succeeded'

class LOAD_MAP(State):
  def __init__(self,_name):
    State.__init__(self, outcomes=['succeeded','aborted'])
    #self.name_ = str(_name)
    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('task_programmer')
    ## @brief 読み込まれたシナリオのデータ
    self.map_path = path + "/config/maps/" + str(_name) + ".yaml"

  def execute(self, userdata):
    rospy.loginfo("load map")

    cmd = 'export DISPLAY=:0.0; \
      gnome-terminal --zoom=0.2 --geometry=+0+480 --tab -e \'bash -c \"rosrun map_server map_server {} __name:=map_localization_server \"\' \
      --tab -e \'bash -c \"rosrun map_server map_server {} __name:=map_planning_server map:=map_keepout \"\' \
      ;export DISPLAY=:10.0'
    subprocess.call(cmd.format(self.map_path,self.map_path), shell=True)
    rospy.sleep(1)

    try:
      rospy.loginfo("set initialpose")
      set_initialpose = rospy.ServiceProxy('set_initialpose', SetInitialPose)
      res = set_initialpose(0, 0, 0)
      rospy.sleep(1)
      return 'succeeded'

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return 'aborted'

class SET_INFLATION(State):
  def __init__(self,_value):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.value_ = float(_value)
    self.client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation", timeout=30)

  def execute(self, userdata):
    print "set inflation"
    self.client.update_configuration({"inflation_radius":self.value_})
    
    return 'succeeded'

class SET_MAX_VELOCITY(State):
  def __init__(self, _velocity):
    State.__init__(self, outcomes=['succeeded','aborted'])
    velocity_string = _velocity.split(',')
    self.velocity_ = [float(s) for s in velocity_string]
    self.client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=30)

  def execute(self, userdata):
    print "set max velocity"
    self.client.update_configuration({"max_vel_x":self.velocity_[0], "max_vel_y":self.velocity_[1], "max_vel_theta":self.velocity_[2]})
    
    return 'succeeded'

class SET_INITIAL_POSE(State):
  def __init__(self, _pose):
    State.__init__(self, outcomes=['succeeded','aborted'])
    pose_string = _pose.split(',')
    self.pose_ = [float(s) for s in pose_string]

  def execute(self, userdata):
    try:
      rospy.loginfo("set initialpose")
      set_initialpose = rospy.ServiceProxy('set_initialpose', SetInitialPose)
      res = set_initialpose(self.pose_[0], self.pose_[1], self.pose_[2])
      rospy.sleep(1)
      return 'succeeded'

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return 'aborted'
  
############################################
## @brief シナリオの読込クラス
class Scenario:
  ## @brief コンストラクタ。引数のシナリオファイル名を読み込む（デフォルトはscenario.yaml)
  def __init__(self):

    if(len(sys.argv) != 2): file_name = "scenario.yaml"
    else: file_name = str(sys.argv[1])

    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('task_programmer')
    ## @brief 読み込まれたシナリオのデータ
    self.scenario = yaml.load(file(path + "/config/" + file_name))
    ## @brief タスクの数
    self.scenario_size = len(self.scenario)
    self.loop_count_array = self.scenario_size * [0]

    rospy.set_param('/task_programmer/wait_task',False)
    rospy.set_param('/task_programmer/jump',1)

  ## @brief タスクの読込
  # @param _number scenario.yamlのデータ行
  # @return タスク名(move,lifter,time,endなど)
  def read_task(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['task']

  ## @brief waypointsの読込
  # @param _number scenario.yamlのデータ行
  # @return waypoints名(0,1,2,3など)
  def read_place(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  def read_via_place(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  def read_velocity(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']
  
  ## @brief リフターの姿勢読込
  # @param _number scenario.yamlのデータ行
  # @return リフターの絶対位置(x,z[m])と移動時間[msec]
  def read_position(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  ## @brief 待ち時間の読込
  # @param _number scenario.yamlのデータ行
  # @return 待ち時間[msec]
  def read_time(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  ## @brief ループ先の読込
  # @param _number scenario.yamlのデータ行
  # @return ジャンプ先
  def read_jump(self, _number):
    rev = dict(self.scenario[_number])
    jump_number = int(rev['action']['arg'].split(',')[0])
    if jump_number > 0:
      jump_number = jump_number - 1
    else :
      jump_number = 0
    return jump_number

  ## @brief ループ回数
  # @param _number scenario.yamlのデータ行
  # @return ループ回数
  def read_count(self, _number):
    rev = dict(self.scenario[_number])
    return [_number,rev['action']['arg'].split(',')[1]]

  def read_marker(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  def read_led(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  def read_map(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']
  
  def read_inflation(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  def read_max_velocity(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']

  def read_initial_pose(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['arg']
  
#==================================
#==================================
if __name__ == '__main__':
  rospy.init_node('aero_scenario_node')

  mc = MoveitCommand()
  na = NaviAction()
  ta = TaskAction()
  sn = Scenario()

  # scneario_playというステートマシンのインスタンスを作成
  scenario_play = StateMachine(outcomes=['succeeded','aborted'])
  # scneario_playにステートを追加
  with scenario_play:

    StateMachine.add('ACTION -1', NONE(), \
      transitions={'succeeded':'ACTION 0','aborted':'ACTION 0'})

    for i in range(sn.scenario_size):
      if sn.read_task(i) == 'init':
        StateMachine.add('ACTION ' + str(i), INITIALIZE(), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'move':
        StateMachine.add('ACTION ' + str(i), GO_TO_PLACE(sn.read_place(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'via':
        StateMachine.add('ACTION ' + str(i), GO_TO_VIA_POINT(sn.read_via_place(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'set_pose':
        StateMachine.add('ACTION ' + str(i), SET_POSE(sn.read_place(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'vel_move':
        StateMachine.add('ACTION ' + str(i), VELOCITY_MOVE(sn.read_velocity(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'lifter':
        StateMachine.add('ACTION ' + str(i), LIFTER(sn.read_position(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'wait':
        StateMachine.add('ACTION ' + str(i), WAIT(sn.read_time(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i-1)})
      elif sn.read_task(i) == 'loop':
        StateMachine.add('ACTION ' + str(i), LOOP(sn.read_count(i)), \
          transitions={'succeeded':'ACTION '+ str(sn.read_jump(i)),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'marker':
        StateMachine.add('ACTION ' + str(i), GO_TO_MARKER(sn.read_marker(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'led':
        StateMachine.add('ACTION ' + str(i), TURN_ON_LED(sn.read_led(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'map':
        StateMachine.add('ACTION ' + str(i), LOAD_MAP(sn.read_map(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'set_inflation':
        StateMachine.add('ACTION ' + str(i), SET_INFLATION(sn.read_inflation(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'set_max_vel':
        StateMachine.add('ACTION ' + str(i), SET_MAX_VELOCITY(sn.read_max_velocity(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'set_initial_pose':
        StateMachine.add('ACTION ' + str(i), SET_INITIAL_POSE(sn.read_initial_pose(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'ros_clean':
        StateMachine.add('ACTION ' + str(i), ROS_CLEAN(), \
          transitions={'succeeded':'succeeded','aborted':'aborted'})
      elif sn.read_task(i) == 'end':
        StateMachine.add('ACTION ' + str(i), FINISH(), \
          transitions={'succeeded':'succeeded','aborted':'aborted'})
      else :
        StateMachine.add('ACTION ' + str(i), NONE(), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})

  ## @brief デバッグ出力(smach_viewerで見れるようにする) @sa http://wiki.ros.org/smach_viewer
  sis = smach_ros.IntrospectionServer('server_name',scenario_play,'/SEED-Noid-Mover Scenario Play')
  sis.start()
  # ステートマシンの実行
  scenario_play.execute()
  sis.stop()

