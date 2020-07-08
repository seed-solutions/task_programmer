#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "task_programmer/TaskController.h"

//////////////////////////////
class TaskController{
public:
  TaskController(ros::NodeHandle _nh);
  bool taskControllerCallback(task_programmer::TaskController::Request &_req, task_programmer::TaskController::Response &_res);
  bool moveMarker(std::string _marker);

  tf::StampedTransform GetTransformObject(std::string _base, std::string _object);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer task_controller_server_;

  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist cmd_vel_;

  tf::TransformListener listener_;

};

#endif
