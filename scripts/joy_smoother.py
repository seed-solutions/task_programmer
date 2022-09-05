#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
from sensor_msgs.msg import Joy

class JoySmoother():
  def __init__(self):
    self.K = 0.9

    self.joy_publisher = rospy.Publisher('joy_out', Joy, queue_size=1)
    self.joy = Joy()
    self.joy.axes = 6 * [0]
    self.joy.buttons = 13 * [0]
    self.pre_joy = Joy()
    self.pre_joy.axes = 6 * [0]
    self.pre_joy.buttons = 13 * [0]


  def callback(self,data):
    for i in range(0,4):
      self.joy.axes[i] = self.K * self.pre_joy.axes[i] + (1 - self.K) * data.axes[i]
      if(self.joy.axes[i] > 0.99 ): self.joy.axes[i] = 1
      elif(self.joy.axes[i] < 0.01 and self.joy.axes[i] > -0.01 ): self.joy.axes[i] = 0
      elif(self.joy.axes[i] < -0.99 ): self.joy.axes[i] = -1

    self.joy.buttons = copy.copy(data.buttons)

    self.joy_publisher.publish(self.joy)
    self.pre_joy = self.joy

  def subscriber(self):
    rospy.Subscriber("joy_in", Joy, self.callback)
    rospy.spin()

if __name__ == '__main__':
  rospy.init_node('joy_smoother', anonymous=True)

  js = JoySmoother()

  try:
    js.subscriber()
  except rospy.ROSInterruptException: pass
