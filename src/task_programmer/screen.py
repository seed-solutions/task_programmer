#!/usr/bin/env python
# -*- coding: utf8 -*-
import sys
import rospy
import rospkg
import glob
import subprocess

class Screen:
  def __init__(self):
    rospack = rospkg.RosPack()
    rospack.list() 
    self.path = rospack.get_path('task_programmer')

  def video(self,_file):
    if sys.version_info.major != 2:
      file_name = _file.decode('utf-8')
    else:
      file_name = _file

    if(file_name == "kill"):
      cmd = 'killall mplayer;'
      subprocess.call(cmd, shell=True)
      return True

    file_path = self.path + "/config/screen/videos/" + "\'" + file_name + "\'"
    if not glob.glob(file_path.replace('\'','') +".*"):
      rospy.logerr(file_path + ".* does not exist")
      return False

    cmd = 'export DISPLAY=:0.0; killall mplayer; \
          mplayer -fs -zoom {} > /dev/null 2>&1 & export DISPLAY=:10.0' 
    subprocess.call(cmd.format(file_path + ".*"), shell=True)

    return True

  def image(self,_file):
    if sys.version_info.major != 2:
      file_name = _file.decode('utf-8')
    else:
      file_name = _file

    if(file_name == "kill"):
      cmd = 'killall eog;'
      subprocess.call(cmd, shell=True)
      return True

    file_path = self.path + "/config/screen/images/" + "\'" + file_name + "\'"
    if not glob.glob(file_path.replace('\'','') +".*"):
      rospy.logerr(file_path + ".* does not exist")
      return False

    cmd = 'export DISPLAY=:0.0; eog --fullscreen {} & export DISPLAY=:10.0' 
    subprocess.call(cmd.format(file_path + ".*"), shell=True)

    return True

  def kill(self):
    devnull = open('/dev/null', 'w')
    subprocess.call("killall mplayer;killall eog;", shell=True, stdout=devnull, stderr=devnull)
    
