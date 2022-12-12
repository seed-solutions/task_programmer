#!/bin/bash
ros_ws=$1
robot_ip=$2
user_name=$3
password=$4
local_ip=$5
command=$6

##----- version check ----------
#version=`cat /etc/lsb-release | grep DISTRIB_RELEASE`
#if [[ ${version} =~ "16" ]]; then
#  ros_ws=/home/seed/ros/kinetic
#elif [[ ${version} =~ "18" ]]; then
#  ros_ws=/home/seed/ros/melodic
#elif [[ ${version} =~ "20" ]]; then
#  ros_ws=/home/seed/ros/noetic
#fi
##-------------------------------
source ${ros_ws}/devel/setup.bash
roscd task_programmer/scripts

if [ ${command} = "controller" ]; then 
  :
elif [ ${command} = "make_map" ]; then
  rosnode kill --all & killall -9 roscore & killall -9 rosmaster;
  killall gnome-terminal-server;
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"roslaunch task_programmer robot_bringup.launch;exit\" "'
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"sleep 1;export ROS_IP='${robot_ip}'\" \"roslaunch --wait task_programmer make_map.launch;exit\" "'

elif [ ${command} = "save_map" ]; then
  map_name=$7
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"roslaunch task_programmer map_saver.launch dir_name:='${map_name}' ;exit\" "'

elif [ ${command} = "static_map" ]; then 
  map_name=$7
  rosnode kill --all & killall -9 roscore & killall -9 rosmaster;
  killall gnome-terminal-server;
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"roslaunch task_programmer robot_bringup.launch;exit\" "'
  gnome-terminal --zoom=0.5 \
    --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"sleep 1;export ROS_IP='${robot_ip}'\" \"roslaunch --wait task_programmer static_map.launch dir_name:='${map_name}' can_del_wp:=true; exit\" "'
#    --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"sleep 1;export ROS_IP='${robot_ip}'\" \"roslaunch task_programmer --wait realsense_laser.launch;exit\" "'


elif [ ${command} = "scenario_start" ]; then 
  scenario_name=$7
  gnome-terminal --zoom=0.5 --geometry=+0+0 --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"export DISPLAY=:0.0;rosrun task_programmer start.py '${scenario_name}';exit\" "'

elif [ ${command} = "scenario_stop" ]; then
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    rosnode kill /aero_scenario_node;

elif [ ${command} = "scenario_restart" ]; then 
  gnome-terminal --zoom=0.5 --geometry=+0+0 --tab -e 'bash -c "export ROS_MASTER_URI=http://'${robot_ip}':11311;export ROS_IP='${local_ip}';rosparam set /task_programmer/wait_task False ;exit"'

elif [ ${command} = "rviz" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "export ROS_MASTER_URI=http://'${robot_ip}':11311;export ROS_IP='${local_ip}';roslaunch --wait task_programmer view.launch;exit"'

elif [ ${command} = "operation" ]; then 
  action=$7
  scenario_name=$8
  map_name=$9
  way_point=${10}
  lifter_position=${11}

  if [ ${action} = "start" ]; then 
    gnome-terminal --zoom=0.5 \
      --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"rosnode kill --all;killall -9 roscore;killall -9 rosmaster;exit\" "';
    gnome-terminal --zoom=0.5 \
      --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"roslaunch task_programmer robot_bringup.launch;exit\" "' \
      --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"sleep 1;export ROS_IP='${robot_ip}'\" \"roslaunch --wait  task_programmer static_map.launch dir_name:='${map_name}';exit\" "' \
      --tab -e 'bash -c "export ROS_MASTER_URI=http://'${robot_ip}':11311;export ROS_IP='${local_ip}';sleep 5;roslaunch --wait task_programmer view.launch;exit"' \
      --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"sleep 5;rosrun task_programmer start.py '${scenario_name}';exit\" "'
#      --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"sleep 1;export ROS_IP='${robot_ip}'\" \"roslaunch task_programmer --wait realsense_laser.launch;exit\" "' 

  elif [ ${action} = "previous" ]; then 
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    rosparam set /task_programmer/jump -1;
    rosparam set /task_programmer/wait_task False ;
  elif [ ${action} = "next" ]; then 
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    rosparam set /task_programmer/jump 1;
    rosparam set /task_programmer/wait_task False ;
  elif [ ${action} = "pause" ]; then 
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
#    rosrun task_programmer go.py
    gnome-terminal --zoom=0.5 \
      --geometry=+0+0 --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"export DISPLAY=:0.0;rosrun task_programmer go.py ;exit\" "'
  elif [ ${action} = "go_waypoint" ]; then 
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
#    rosrun task_programmer go.py ${way_point} ${lifter_position}
    gnome-terminal --zoom=0.5 \
      --geometry=+0+0 --tab -e 'bash -c "expect ssh.exp '${user_name}'@'${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"export DISPLAY=:0.0;rosrun task_programmer go.py '${way_point}' '${lifter_position}';exit\" "'
  elif [ ${action} = "stop" ]; then 
    killall gnome-terminal-server&
    expect ssh.exp ${user_name}@${robot_ip} ${password} "export DISPLAY=:0.0;killall gnome-terminal-server; bash '${ros_ws}'/src/task_programmer/scripts/teleop_mover.sh"
  fi

elif [ ${command} = "kill" ]; then
  killall gnome-terminal-server

elif [ ${command} = "shutdown" ]; then 
  rosclean purge -y;
  sleep 1;
  expect ssh.exp ${user_name}@${robot_ip} ${password} "expect -c \" spawn env LANG=C sudo shutdown -h now; expect password; send ${password}\n; interact \" "

elif [ ${command} = "reboot" ]; then 
  expect ssh.exp ${user_name}@${robot_ip} ${password} "expect -c \" spawn env LANG=C sudo reboot; expect password; send ${password}\n; interact \" "

elif [ ${command} = "save_scenario" ]; then
  file_name=$7
  directory_name=$8
  remote_directory_name=$9
  expect scp.exp ${user_name}@${robot_ip} ${password} send ${file_name} ${directory_name} ${remote_directory_name}

elif [ ${command} = "load_scenario" ]; then 
  file_name=$7
  directory_name=$8
  remote_directory_name=$9
  expect scp.exp ${user_name}@${robot_ip} ${password} load ${file_name} ${directory_name} ${remote_directory_name}

elif [ ${command} = "load_waypoint" ]; then 
  file_name=$7
  directory_name=$8
  remote_directory_name=$9
  expect scp.exp ${user_name}@${robot_ip} ${password} load ${file_name} ${directory_name} ${remote_directory_name}

elif [ ${command} = "check_wifi" ]; then 
  while true
  do
    ping -q -c5 $robot_ip > /dev/null
    if [ $? -eq 0 ]; then
      echo "Connection Succeeded"
      break;
    fi
  done

elif [ ${command} = "check_wp_dir" ]; then 
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    rosparam get /waypoints_editor/dir_name;
fi
