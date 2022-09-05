#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
import subprocess
import rosnode
from sensor_msgs.msg import Joy

class GamepadFunction():
  def __init__(self):
    self.pre_joy = Joy()
    self.pre_joy.axes = 6 * [0]
    self.pre_joy.buttons = 13 * [0]

    self.bringup = 'export DISPLAY=:0.0; gnome-terminal --zoom=0.5 \
      --tab -- bash -c "roslaunch --wait task_programmer static_map.launch can_del_wp:=false;exit\" '
    self.scenario_run = 'export DISPLAY=:0.0; gnome-terminal --zoom=0.5 --tab \
      -- bash -c "rosrun task_programmer start.py {}.yaml;exit\" '
    self.kill = "rosnode kill aero_scenario_node;killall mplayer;killall eog;"

  def node_alive(self,name):
    return rosnode.rosnode_ping(name,max_count=1,verbose=False)

  def callback(self,data):
    scenario_name = "scenario"

    if(data.buttons[1] == 0 and self.pre_joy.buttons[1] == 1):    #triangle button
      rospy.loginfo("on triangle")
      rospy.set_param('/task_programmer/wait_task',False)
    elif(data.buttons[3] == 0 and self.pre_joy.buttons[3] == 1):  #circle button
      rospy.loginfo("on circle")
      rospy.set_param('/task_programmer/wait_task',False)
    elif(data.buttons[2] == 0 and self.pre_joy.buttons[2] == 1):  #cross button
      rospy.loginfo("on cross")
      rospy.set_param('/task_programmer/wait_task',False)
    elif(data.buttons[0] == 0 and self.pre_joy.buttons[0] == 1):  #square button
      rospy.loginfo("on square")
      rospy.set_param('/task_programmer/wait_task',False)

    if(len(data.buttons)>12 and len(self.pre_joy.buttons)>12):
      if(data.buttons[12] == 0 and self.pre_joy.buttons[12] == 1):  # GuidButton
        # move_baseが起動していなかったら、起動してからシナリオ実行
        if(not self.node_alive("/move_base")):
          subprocess.call(self.bringup, shell=True)
          while(not self.node_alive("/move_base")):
            rospy.sleep(1)
          rospy.sleep(1)
          subprocess.call(self.scenario_run.format(scenario_name), shell=True)
        # シナリオが起動していなかったらシナリオ実行
        elif(not self.node_alive("/aero_scenario_node")) : 
          subprocess.call(self.scenario_run.format(scenario_name), shell=True)
        # シナリオが起動していたら、シナリオ終了
        else :
          subprocess.call(self.kill, shell=True)

    self.pre_joy = copy.copy(data)


  def subscriber(self):
    rospy.Subscriber("joy", Joy, self.callback)
    rospy.spin()

if __name__ == '__main__':
  rospy.init_node('gamepad_function', anonymous=True)

  js = GamepadFunction()

  try:
    js.subscriber()
  except rospy.ROSInterruptException: pass
