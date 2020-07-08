#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
from sensor_msgs.msg import Joy

class GamepadFunction():
  def __init__(self):
    self.pre_joy = Joy()
    self.pre_joy.axes = 29 * [0]
    self.pre_joy.buttons = 17 * [0]


  def callback(self,data):

    if(data.buttons[12] == 0 and self.pre_joy.buttons[12] == 1):    #triangle button
      rospy.loginfo("on triangle")
      rospy.set_param('/task_programmer/wait_task',False)
    elif(data.buttons[13] == 0 and self.pre_joy.buttons[13] == 1):  #circle button
      rospy.loginfo("on circle")
      rospy.set_param('/task_programmer/wait_task',False)
    elif(data.buttons[14] == 0 and self.pre_joy.buttons[14] == 1):  #cross button
      rospy.loginfo("on cross")
      rospy.set_param('/task_programmer/wait_task',False)
    elif(data.buttons[15] == 0 and self.pre_joy.buttons[15] == 1):  #square button
      rospy.loginfo("on square")
      rospy.set_param('/task_programmer/wait_task',False)

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
