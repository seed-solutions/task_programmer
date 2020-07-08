#include <ros/ros.h>
#include <fstream>
//#include <string>
//to get tf--
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//---
//to get ps3 data---
#include <sensor_msgs/Joy.h>
//read yaml---
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <ros/package.h>

//////////////////////////////
/** 
* @brief waypointsの作成/読み出し用。
* @details DUALSHOCKのボタンに応じて関数が呼び出される。
*/
class Waypoint_Edit{
public:
  /**
  * @brief コンストラクタ
  * @param _nh ROSのノードハンドル
  */
  Waypoint_Edit(ros::NodeHandle _nh);

  /**
  * @brief DUALSHOCKデータをサブスクライブした際のコールバック定義
  * @param _ps3 DUALSHOCKのデータ
  */
  void getPS3(const sensor_msgs::JoyPtr& _ps3);

private:
//to get tf--
  tf::TransformListener listener_; ///< @brief TFの取得 @sa http://wiki.ros.org/tf

//to get ps3 data--
  ros::Subscriber ps3_sub_; ///< @brief DUALSHOKデータ取得用サブスクライバーの定義
  int32_t ps3_start_; ///< @brief DUALSHOCKのスタートボタンデータの格納
  int32_t ps3_pre_start_; ///< @brief DUALSHOCKの過去のスタートボタンデータの格納

  int32_t ps3_select_;  ///< @brief DUALSHOCKのセレクトボタンデータの格納
//----

  ros::NodeHandle nh_;  ///< @brief ROSのノードハンドル
  uint16_t counter_;  ///< @brief waypoints番号のカウント

  std::string pkg_path_;  ///< @brief task_programmerの絶対位置パス格納
};

Waypoint_Edit::Waypoint_Edit(ros::NodeHandle _nh)
  : ps3_start_(0)
  , ps3_pre_start_(0)
  , ps3_select_(0)
  , counter_(0)
{ 	
//to get ps3 data---
  nh_ = _nh;
  ps3_sub_ = nh_.subscribe("/joy",1, &Waypoint_Edit::getPS3,this);
//--
  pkg_path_ = ros::package::getPath("task_programmer");

}

void Waypoint_Edit::getPS3(const sensor_msgs::JoyPtr& _ps3){
  ps3_start_ = _ps3->buttons[3];
  ps3_select_ = _ps3->buttons[0];

  static tf::TransformBroadcaster br_now;
  static tf::TransformBroadcaster br_all;
  tf::StampedTransform transform_now;
  tf::StampedTransform transform_all;
  tf::Quaternion q;

  move_base_msgs::MoveBaseGoal goal;

  //スタートボタンが押されたら現在値の座標（TF）をwaypointsとして登録する
  if (ps3_start_ == 0 && ps3_pre_start_ == 1){
    try{
      listener_.lookupTransform("/map", "/base_link",ros::Time(0), transform_now);

      //save postion------------
      std::ofstream ofs(pkg_path_ + "/config/waypoints.yaml", std::ios_base::app);   
      ofs << "- pose:"              << std::endl;
      ofs << "    position:"      << std::endl;
      ofs << "        x: "        << transform_now.getOrigin().x() << std::endl;
      ofs << "        y: "        << transform_now.getOrigin().y() << std::endl;
      ofs << "        z: "        << transform_now.getOrigin().z() << std::endl;
      ofs << "    orientation:"   << std::endl;
      ofs << "        x: "        << transform_now.getRotation().x() << std::endl;
      ofs << "        y: "        << transform_now.getRotation().y() << std::endl;
      ofs << "        z: "        << transform_now.getRotation().z() << std::endl;
      ofs << "        w: "        << transform_now.getRotation().w() << std::endl; 
      //------------------------------

      br_now.sendTransform(tf::StampedTransform(transform_now, ros::Time::now(), "map", "s" + std::to_string(counter_) ) );

      counter_ += 1;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  //セレクトボタンが押されたら登録されているwaypointsをTFとして発行する
  else if (ps3_select_ == 1){  //readm yaml file
    YAML::Node config = YAML::LoadFile(pkg_path_ + "/config/waypoints.yaml");
    const YAML::Node &wp_node_tmp = config;
    const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    if(wp_node != NULL){
      for(int i=0; i < wp_node->size(); i++){
        goal.target_pose.pose.position.x = (*wp_node)[i]["pose"]["position"]["x"].as<float>() ;
        goal.target_pose.pose.position.y = (*wp_node)[i]["pose"]["position"]["y"].as<float>() ;
        goal.target_pose.pose.position.z = (*wp_node)[i]["pose"]["position"]["z"].as<float>() ;
        goal.target_pose.pose.orientation.x = (*wp_node)[i]["pose"]["orientation"]["x"].as<float>() ;
        goal.target_pose.pose.orientation.y = (*wp_node)[i]["pose"]["orientation"]["y"].as<float>() ;
        goal.target_pose.pose.orientation.z = (*wp_node)[i]["pose"]["orientation"]["z"].as<float>() ;
        goal.target_pose.pose.orientation.w = (*wp_node)[i]["pose"]["orientation"]["w"].as<float>() ;

        transform_all.setOrigin( tf::Vector3(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z) );
        q.setRPY(0,0,tf::getYaw(goal.target_pose.pose.orientation));
        transform_all.setRotation(q);

        br_all.sendTransform(tf::StampedTransform(transform_all, ros::Time::now(), "map", "p" + std::to_string(i)));

      }
    }
  }
  //L1,L2,R1,R2ボタンが同時押しされたら、waypointsをyamlファイルごと削除する
  else if (_ps3->buttons[8] == 1 && _ps3->buttons[9] == 1 && _ps3->buttons[10] == 1 && _ps3->buttons[11] == 1){
    //L1,L2,R1,R2 == 1
    std::ofstream ofs(pkg_path_ + "/config/waypoints.yaml", std::ios_base::trunc);
  }

  ps3_pre_start_ = ps3_start_;

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"waypoints_editor");
	ros::NodeHandle nh;

  Waypoint_Edit we(nh);

	ros::spin();

	return 0;
}
