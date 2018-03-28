#include "astar_planner/search_info_ros.h"

SearchInfo::SearchInfo()
  : map_set_(false)
  , start_set_(false)
  , goal_set_(false)
  , goal_update_flag_(false)
{
}

SearchInfo::~SearchInfo()
{
}

void SearchInfo::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  map_ = *msg;

  // TODO: what frame do we use?
  std::string map_frame = "/odom";
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

//  ogm2map_ = map2ogm_frame.inverse();

  map_set_ = true;
}


void SearchInfo::startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO("Subcscribed current pose!");

  std::string global_frame  = "/odom";
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
  start_pose_local_.header  = start_pose_global_.header;
  
  start_pose_global_.header.frame_id = "/odom";
  start_pose_global_.header.stamp = ros::Time::now();
  start_pose_local_.header = start_pose_global_.header;
  
  start_set_ = true;
}


void SearchInfo::currentPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO("Subcscribed current pose!");

  std::string global_frame  = "/odom";
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
    ROS_INFO_THROTTLE(0.5, "search start cell[in global_map coord] : [%f, %f, %f]", start_pose_local_.pose.position.x,
              start_pose_local_.pose.position.y, yaw * 180 / M_PI);
    start_pose_local_.header = start_pose_local_.header;
  if(!last_goal_pose_local_.header.frame_id.empty()) {
    if (sqrt(std::pow(last_goal_pose_local_.pose.position.x - start_pose_local_.pose.position.x, 2)
         + std::pow(last_goal_pose_local_.pose.position.y - start_pose_local_.pose.position.y, 2)) < 3) {
      goal_update_flag_ = true;
    }
  }
  start_set_ = true;
}


void SearchInfo::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!map_set_)
    return;

//  ROS_INFO("Subcscribed goal pose!");

  // TODO: what frame do we use?
  std::string global_frame  = "/odom";
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
    ROS_INFO("search goal cell [in global_map coord]: [%f, %f, %f]", tmp.position.x, tmp.position.y, yaw * 180 / M_PI);
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

  goal_set_ = true;
}

void SearchInfo::reset()
{
  map_set_   = false;
  start_set_ = false;
//  goal_set_  = false;
}
