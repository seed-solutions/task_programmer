#include "TaskController.hh"

TaskController::TaskController(const ros::NodeHandle _nh) :
  nh_(_nh)
{
  task_controller_server_ = nh_.advertiseService("task_controller", &TaskController::taskControllerCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
}

bool TaskController::moveMarker(std::string _marker){
  tf::StampedTransform marker_coord;
  tf::StampedTransform camera_coord;
  tf::StampedTransform object_coord;
  tf::StampedTransform cd_tf;
  tf::Quaternion rotate;

  double dx,dy,dth;

  bool reached_x,reached_y,reached_th;
  reached_x = reached_y = reached_th = false;
  rotate.setRPY(0,1.57,0);

  double goal_x = 0.12;
  double goal_y = 0.01;
  double goal_z = 0.02; //3[deg]

  double gain_x = 0.1;
  double gain_y = 0.3;
  double gain_z = 0.3;

  while(1){
    //marker_coord = GetTransformObject("ar_marker_0");
    //marker_coord = GetTransformObject(_marker);
    //camera_coord = GetTransformObject("front_camera_link");
    object_coord = GetTransformObject("front_camera_link",_marker);

    //dx = marker_coord.getOrigin().x() - camera_coord.getOrigin().x();
    //dy = marker_coord.getOrigin().y() - camera_coord.getOrigin().y();
    //dth = tf::getYaw(marker_coord.getRotation() * rotate);

    dx = object_coord.getOrigin().x();
    dy = object_coord.getOrigin().y();
    dth = tf::getYaw(object_coord.getRotation() * rotate);

    //std::cout << dx << "\t" << dy << "\t" << fabs(dx) << "\t" << fabs(dy) << std::endl;
    //std::cout << "\t" << dth << std::endl;

    if(fabs(dx) < goal_x) reached_x = true;
    else reached_x = false;
    if(fabs(dy) < goal_y) reached_y = true;
    else reached_y = false;
    if(fabs(dth) < goal_z) reached_th = true;
    else reached_th = false;


    if((marker_coord.getOrigin() == tf::Vector3(0.0,0.0,0.0)) || (reached_x == true && reached_y == true && reached_th == true)) break;
    else{
      if(reached_x == false) cmd_vel_.linear.x = dx * gain_x;
      else cmd_vel_.linear.x = 0;

      if(reached_y == false) cmd_vel_.linear.y = dy * gain_y;
      else cmd_vel_.linear.y = 0;

      if(dth < 0.52 && reached_th == false) cmd_vel_.angular.z= dth * gain_z;
      else cmd_vel_.angular.z= 0;

      //std::cout << cmd_vel_.linear.x << "," << cmd_vel_.linear.y << "," << cmd_vel_.angular.z << std::endl;
      
      cmd_vel_pub_.publish(cmd_vel_);
      //ros::Duration(0.1).sleep();
    }
  }
  cmd_vel_.linear.x = 0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.angular.z= 0;

  cmd_vel_pub_.publish(cmd_vel_);

  return true;
}

tf::StampedTransform TaskController::GetTransformObject(std::string _base,std::string _object){
  tf::StampedTransform transform;

  try{
    ros::Time now = ros::Time(0);
    //listener_.waitForTransform(_base,_object,now, ros::Duration(2.0));
    listener_.lookupTransform(_base,_object,now, transform);
  }
  catch (tf::TransformException ex){
    ROS_INFO("No getting point");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
    transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  }

  return transform;
}


bool TaskController::taskControllerCallback(task_programmer::TaskController::Request &_req, task_programmer::TaskController::Response &_res)
{
  ROS_INFO("call back start");

  if(_req.task == "marker"){
    if(moveMarker(_req.marker) == true) _res.result = "succeeded";
    else _res.result = "aborted";
  }
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"task_controller_node");
  ros::NodeHandle nh;

  TaskController tc(nh);

  ros::spin();

  return 0;
}
