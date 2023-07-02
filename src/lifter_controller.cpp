#include "lifter_controller.h"

LifterController::LifterController(const ros::NodeHandle _nh) :
  nh_(_nh),lifter_ratio_(0.01),on_protective_stop(false),
  update_joints_once(true),update_joints(true)
{

  controller_rate_ = 50;
  controller_cycle_ = (1.0/controller_rate_);
  move_time_ = 0.5;//controller_cycle_;

  joint_state_sub_ = nh_.subscribe("/joint_states",2, &LifterController::jointStateCallback,this);
  joy_sub_ = nh_.subscribe("/joy",2, &LifterController::getJoy,this);
  diag_sub_ = nh_.subscribe("/diagnostics",1, &LifterController::diagnosticsCallback,this);

  init_follow_joint_trajectory();

  

}

void LifterController::init_follow_joint_trajectory()
{
  lifter_client_ = new TrajectoryClient("lifter_controller/follow_joint_trajectory", true);

  while(!lifter_client_->waitForServer(ros::Duration(1)) )
  {
      ROS_INFO("Waiting for the lifter joint_trajectory_action server");
  }

  lifter_goal_.trajectory.joint_names.resize(2);
  lifter_goal_.trajectory.joint_names[0] = "ankle_joint";
  lifter_goal_.trajectory.joint_names[1] = "knee_joint";
  lifter_goal_.trajectory.points.resize(1);
  lifter_goal_.trajectory.points[0].positions.resize(lifter_goal_.trajectory.joint_names.size());
}

void LifterController::jointStateCallback(const sensor_msgs::JointState& _joint_data)
{
  if(update_joints_once)
  {
    std::vector<std::string> joint_names = _joint_data.name;
    std::vector<std::string>::iterator itr;

    itr = std::find(joint_names.begin(),joint_names.end(),std::string("ankle_joint"));
    ankle_jnumber_ = std::distance(joint_names.begin(),itr);

    itr = std::find(joint_names.begin(),joint_names.end(),std::string("knee_joint"));
    knee_jnumber_ = std::distance(joint_names.begin(),itr);

    update_joints_once = false;
  }
  else if(update_joints)
  {
    joint_angles_["ankle"] = _joint_data.position[ankle_jnumber_];
    joint_angles_["knee"] = _joint_data.position[knee_jnumber_];
  }

}
void LifterController::sendJointAngles()
{
  lifter_goal_.trajectory.points[0].positions[0] = joint_angles_["ankle"];
  lifter_goal_.trajectory.points[0].positions[1] = joint_angles_["knee"];
  lifter_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  lifter_goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(controller_cycle_);

  lifter_client_->sendGoal(lifter_goal_);

}

void LifterController::getJoy(const sensor_msgs::JoyPtr& _data)
{
  if((_data->buttons[4] == 1 || _data->buttons[6] == 1) && _data->axes[2] != 0
      && !on_protective_stop)
  {
    update_joints = false;
    joint_angles_["ankle"] -= (_data->axes[2] * lifter_ratio_);
    joint_angles_["knee"] += (_data->axes[2] * lifter_ratio_);
    if(joint_angles_["ankle"] > ankle_upper_limt) joint_angles_["ankle"] = ankle_upper_limt;
    else if(joint_angles_["ankle"] < ankle_lower_limt) joint_angles_["ankle"] = ankle_lower_limt;
    if(joint_angles_["knee"] > knee_upper_limt) joint_angles_["knee"] = knee_upper_limt;
    else if(joint_angles_["knee"] < knee_lower_limt) joint_angles_["knee"] = knee_lower_limt;

    sendJointAngles();
  }
  else
  {
    update_joints = true;
  }

}

void LifterController::diagnosticsCallback(const diagnostic_msgs::DiagnosticArrayPtr& _msg)
{
  std::string hardware_id = _msg->status[0].hardware_id;
  std::map<std::string,std::string> status;

  if(hardware_id.find("SEED-Noid-Mover") != std::string::npos){
    for (int i=0; i < _msg->status[0].values.size(); ++i){
      status[_msg->status[0].values[i].key] = _msg->status[0].values[i].value;
    }
  }

  // send goal 0 when E-STOP pressed
  if(status["Emergency Stopped"] == "True"){
    joint_angles_["ankle"] = 0;
    joint_angles_["knee"] = 0;
    sendJointAngles();
  }
  else if(status["Protective Stopped"] == "True"){
    on_protective_stop = true;
  }
  else if(status["Protective Stopped"] == "False"){
    on_protective_stop = false;
  }

}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"lower_controller_node");
  ros::NodeHandle nh;

  LifterController lc(nh);

  //ros::spin();

  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok()){

  }
  spinner.stop();

  return 0;
}
