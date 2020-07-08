#!/bin/bash
#ros_ws=/home/seed/ros/kinetic
ros_ws=/home/seed/ros/melodic
source ${ros_ws}/devel/setup.bash

gnome-terminal --zoom=0.5 --geometry=+0+0 --tab -e 'bash -c "roslaunch task_programmer robot_bringup.launch "'
