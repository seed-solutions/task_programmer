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
#include <sys/stat.h>

#include <ros/package.h>

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <task_programmer/WaypointsConfig.h>
//---- GUI parameters  ---
std::string DIR_NAME = "/config/maps";
std::string FILE_NAME = "waypoints";
bool ENABLE_DEL = false;

//////////////////////////////
/** 
* @brief waypointsの作成/読み出し用。
* @details ゲームパッドのボタンに応じて関数が呼び出される。
*/
class Waypoint_Edit{
public:
  /**
  * @brief コンストラクタ
  * @param _nh ROSのノードハンドル
  */
  Waypoint_Edit(ros::NodeHandle _nh);

  /**
  * @brief ゲームパッドデータをサブスクライブした際のコールバック定義
  * @param ゲームパッドのデータ
  */
  void getJoy(const sensor_msgs::JoyPtr& _data);

private:
//to get tf--
  tf::TransformListener listener_; ///< @brief TFの取得 @sa http://wiki.ros.org/tf

//to get joy data--
  ros::Subscriber joy_sub_; ///< @brief DUALSHOKデータ取得用サブスクライバーの定義
  int32_t joy_start_; ///< @brief ゲームパッドのスタートボタンデータの格納
  int32_t joy_pre_start_; ///< @brief ゲームパッドの過去のスタートボタンデータの格納

  int32_t joy_select_;  ///< @brief ゲームパッドのセレクトボタンデータの格納
//----

  ros::NodeHandle nh_;  ///< @brief ROSのノードハンドル
  uint16_t counter_;  ///< @brief waypoints番号のカウント

  std::string pkg_path_;  ///< @brief task_programmerの絶対位置パス格納
  
};

Waypoint_Edit::Waypoint_Edit(ros::NodeHandle _nh)
  : joy_start_(0)
  , joy_pre_start_(0)
  , joy_select_(0)
  , counter_(0)
{ 	
//to get joy data---
  nh_ = _nh;
  joy_sub_ = nh_.subscribe("/joy",1, &Waypoint_Edit::getJoy,this);
//--
  pkg_path_ = ros::package::getPath("task_programmer");

}

void Waypoint_Edit::getJoy(const sensor_msgs::JoyPtr& _data){
  joy_start_ = _data->buttons[11];
  joy_select_ = _data->buttons[10];

  static tf::TransformBroadcaster br_now;
  static tf::TransformBroadcaster br_all;
  tf::StampedTransform transform_now;
  tf::StampedTransform transform_all;
  tf::Quaternion q;

  move_base_msgs::MoveBaseGoal goal;

  std::string dir_path = pkg_path_ + DIR_NAME;
  std::string file_path = pkg_path_ + DIR_NAME + "/" + FILE_NAME + ".yaml";

  struct stat st; //ディレクトリの有無確認用

  //スタートボタンが押されたら現在値の座標（TF）をwaypointsとして登録する
  if (joy_start_ == 0 && joy_pre_start_ == 1){
    if(stat(dir_path.c_str(), &st) != 0){ //ディレクトリが存在しない
      ROS_ERROR("%s is not exist", dir_path.c_str());
      joy_pre_start_ = joy_start_;
      return;
    }
    try{
      listener_.lookupTransform("/map", "/base_link",ros::Time(0), transform_now);

      //save postion------------
      std::ofstream ofs(file_path, std::ios_base::app);
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
  else if (joy_select_ == 1){  //readm yaml file
    if(stat(dir_path.c_str(), &st) != 0){ //ディレクトリが存在しない
      ROS_ERROR("%s is not exist", dir_path.c_str());
      joy_pre_start_ = joy_start_;
      return;
    }
    std::ifstream file(file_path);
    if(!file)  std::ofstream ofs(file_path);

    YAML::Node config = YAML::LoadFile(file_path);

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
  else if (ENABLE_DEL && _data->buttons[4] == 1 && _data->buttons[5] == 1 && _data->buttons[6] == 1 && _data->buttons[7] == 1){
    if(stat(dir_path.c_str(), &st) != 0){ //ディレクトリが存在しない
      ROS_ERROR("%s is not exist", dir_path.c_str());
      joy_pre_start_ = joy_start_;
      return;
    }
    //L1,L2,R1,R2 == 1
    std::ofstream ofs(file_path, std::ios_base::trunc);
  }

  joy_pre_start_ = joy_start_;

}

void callback(task_programmer::WaypointsConfig& config, uint32_t level)
{

  ROS_INFO("Reconfigure Request: %s, %s, %d", config.dir_name.c_str(), config.file_name.c_str(), config.enable_del);

  DIR_NAME = config.dir_name.c_str();
  FILE_NAME = config.file_name.c_str();
  ENABLE_DEL = config.enable_del;

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"waypoints_editor");
	ros::NodeHandle nh;

  Waypoint_Edit we(nh);

  dynamic_reconfigure::Server<task_programmer::WaypointsConfig> server;
  dynamic_reconfigure::Server<task_programmer::WaypointsConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

	ros::spin();

	return 0;
}
