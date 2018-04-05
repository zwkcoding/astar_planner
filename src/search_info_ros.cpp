#include "astar_planner/search_info_ros.h"
#include <chrono>


SearchInfo::SearchInfo()
  : allow_time_delay_(500),
    map_set_(false)
  , start_set_(false)
  , goal_set_(false)
  , goal_update_flag_(false)
{
    ros::NodeHandlePtr pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    pnode_->param<std::string>("global_map_frame_name", global_map_frame_name_, "/odom");

}

SearchInfo::~SearchInfo()
{
}

void SearchInfo::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{

  last_receive_map_timestamp_ = ros::WallTime::now();  // tick time

  map_ = *msg;

  planner_map_frame_name_ = map_.header.frame_id;
  ROS_WARN_STREAM_THROTTLE(15, "planner map frame is :" << planner_map_frame_name_);
  // first get position in global frame , then transform  into speicial frame
  std::string map_frame = global_map_frame_name_;
  std::string ogm_frame = msg->header.frame_id;
  // Set transform
  tf::StampedTransform map2ogm_frame;
  try
  {
    tf_listener_.lookupTransform(map_frame, ogm_frame, ros::Time(0), map2ogm_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }


  tf::Transform map2ogm;
  geometry_msgs::Pose ogm_in_map = astar::transformPose(map_.info.origin, map2ogm_frame);
  tf::poseMsgToTF(ogm_in_map, map2ogm);
  ogm2map_ = map2ogm.inverse();

  map_set_ = true;
}


void SearchInfo::startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO_THROTTLE(5, "Subcscribed current pose!");

  std::string global_frame  = global_map_frame_name_;
  std::string goal_frame  = msg->header.frame_id;

  // Get transform (map to world in Autoware)
  tf::StampedTransform world2map;
  try
  {
    tf_listener_.lookupTransform(global_frame, goal_frame, ros::Time(0), world2map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  // Set pose in Global frame
  geometry_msgs::Pose msg_pose = msg->pose.pose;
  start_pose_global_.header = msg->header;
  start_pose_global_.pose   = astar::transformPose(msg_pose, world2map);
  start_pose_local_.pose    = astar::transformPose(start_pose_global_.pose, ogm2map_);

  start_pose_local_.header.stamp = ros::Time::now();
  start_pose_local_.header.frame_id = planner_map_frame_name_;
  
  start_set_ = true;
}


void SearchInfo::currentPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
    last_receive_position_timestamp_ = ros::WallTime::now();  // tick time

    if (!map_set_)
    return;

//  ROS_INFO("Subcscribed current pose!");
//    auto start = std::chrono::system_clock::now();

  std::string global_frame  = global_map_frame_name_;
  std::string goal_frame  = msg->header.frame_id;

   // Get transform (map to world in Autoware)
  tf::StampedTransform world2map;
  try
  {
      tf_listener_.lookupTransform(global_frame, goal_frame, ros::Time(0), world2map);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s", ex.what());
      return;
  }
  geometry_msgs::Pose msg_pose = msg->pose.pose;
  start_pose_global_.header = msg->header;
  start_pose_global_.pose   = astar::transformPose(msg_pose, world2map);
  start_pose_local_.pose = astar::transformPose(start_pose_global_.pose, ogm2map_);
    double yaw = tf::getYaw(start_pose_local_.pose.orientation);
    ROS_INFO_THROTTLE(0.5, "search start cell[in planner_map coord] : [%f[m], %f[m], %f[degree]]", start_pose_local_.pose.position.x,
              start_pose_local_.pose.position.y, yaw * 180 / M_PI);
    start_pose_local_.header.frame_id = planner_map_frame_name_;
    start_pose_local_.header.stamp =  ros::Time::now();

    if(!last_goal_pose_local_.header.frame_id.empty()) {
    if (sqrt(std::pow(last_goal_pose_local_.pose.position.x - start_pose_local_.pose.position.x, 2)
         + std::pow(last_goal_pose_local_.pose.position.y - start_pose_local_.pose.position.y, 2)) < 1.5) {
      goal_update_flag_ = true;
    }
  }
  start_set_ = true;

//    auto end = std::chrono::system_clock::now();
//    auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
//    ROS_INFO_THROTTLE(1, "start pose tf calc msec: %lf", msec);
}


void SearchInfo::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    last_receive_goal_timestamp_ = ros::WallTime::now();  // tick time
  if (!map_set_)
    return;

  ROS_INFO_THROTTLE(5, "Subcscribed goal pose!");

  // TODO: what frame do we use?
  std::string global_frame  = global_map_frame_name_;
  std::string goal_frame  = msg->header.frame_id;

  // Get transform (map to world in Autoware)
  tf::StampedTransform world2map;
  try
  {
    tf_listener_.lookupTransform(global_frame, goal_frame, ros::Time(0), world2map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set pose in Global frame
  geometry_msgs::Pose msg_pose = msg->pose;
  goal_pose_global_.pose   = astar::transformPose(msg_pose, world2map);
  goal_pose_global_.header = msg->header;

    geometry_msgs::Pose tmp = astar::transformPose(goal_pose_global_.pose, ogm2map_);
  double yaw = tf::getYaw(tmp.orientation);
    ROS_INFO_THROTTLE(5, "search goal cell [in planner_map coord]: [%f[m], %f[m], %f[degree]]", tmp.position.x, tmp.position.y, yaw * 180 / M_PI);
#ifndef PLAN_IN_LOCAL_MAP
  // first time receive goal command or arrived last goal
  if(last_goal_pose_local_.header.frame_id.empty() || goal_update_flag_ == true) {
    ROS_INFO("last goal is reached or last goal is aborted!");
    goal_pose_local_.pose = astar::transformPose(goal_pose_global_.pose, ogm2map_);
    goal_pose_local_.header = goal_pose_global_.header;
    last_goal_pose_local_ = goal_pose_local_;
    goal_update_flag_ = false;
  } else{
    // keep last goal
    goal_pose_local_ = last_goal_pose_local_;
  }
#else
    if(last_goal_pose_local_.header.frame_id.empty() || goal_update_flag_ == true) {
        ROS_INFO("last goal is reached or last goal is aborted!");
        goal_pose_local_.pose = astar::transformPose(goal_pose_global_.pose, ogm2map_);
        goal_pose_local_.header = goal_pose_global_.header;
        last_goal_pose_local_ = goal_pose_local_;
        goal_update_flag_ = false;
    }
    goal_pose_local_.pose = astar::transformPose(goal_pose_global_.pose, ogm2map_);
    goal_pose_local_.header.stamp = ros::Time::now();
    goal_pose_local_.header.frame_id =  planner_map_frame_name_;

#endif

  goal_set_ = true;
}

void SearchInfo::reset()
{
    ros::WallTime end = ros::WallTime::now();
    double map_elapse_time = (end - last_receive_map_timestamp_).toSec() * 1000;
    double position_elapse_time = (end - last_receive_position_timestamp_).toSec() * 1000;
    double goal_elapse_time = (end - last_receive_goal_timestamp_).toSec() * 1000;
    if(map_elapse_time < allow_time_delay_) {
    map_set_   = true;
    } else {
        map_set_   = false;
        ROS_ERROR("Receive planner map delay exceed time!");
    }
    if(position_elapse_time < allow_time_delay_) {
        start_set_ = true;
    }else {
        start_set_   = false;
        ROS_ERROR("Receive vehicle position delay exceed time!");
    }
    if(goal_elapse_time < allow_time_delay_) {
        goal_set_ = true;
    }else {
#ifdef PLAN_IN_LOCAL_MAP
        goal_set_  = false;
        ROS_ERROR("Receive goal position delay exceed time!");
#endif
    }


}