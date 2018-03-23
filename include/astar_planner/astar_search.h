#ifndef ASTAR_NAVI_NODE_H
#define ASTAR_NAVI_NODE_H

#define DEBUG 1

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <chrono>

#include "astar_util.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <internal_grid_map/internal_grid_map.hpp>
#include <car_model/car_geometry.hpp>


namespace astar_planner {

class AstarSearch
{
 public:
  AstarSearch();
  ~AstarSearch();

  //-- FOR DEBUG -------------------------
  void publishPoseArray(const ros::Publisher &pub, const std::string &frame);
  void publishFootPrint(const ros::Publisher &pub, const std::string &frame);
  geometry_msgs::PoseArray debug_pose_array_;
  //--------------------------------------

  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, const nav_msgs::OccupancyGrid &map);
  void reset();
  void broadcastPathTF();
  nav_msgs::Path getPath() {return path_;}
  nav_msgs::Path getDensePath() {return dense_path_;}
  void samplePathByStepLength(double step = 0.1);

 private:
  bool search();
  void resizeNode(int width, int height, int angle_size);
  void createStateUpdateTable(int angle_size);
  void createStateUpdateTableLocal(int angle_size); //
  void poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta);
  bool getOdomCoordinates(double &x, double &y, unsigned int ind_x, unsigned int ind_y);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode &goal);

  void setMap(const nav_msgs::OccupancyGrid &map);
  bool setStartNode();
  bool setGoalNode();
  bool isGoal(double x, double y, double theta);
  bool detectCollision(const SimpleNode &sn);
  bool calcWaveFrontHeuristic(const SimpleNode &sn);
  bool detectCollisionWaveFront(const WaveFrontNode &sn);
  // use car geometry and gridmap pkg
  void InitCarGeometry(hmpl::CarGeometry &car);

  bool isSingleStateCollisionFree(const SimpleNode &current_state);
  bool isSingleStateCollisionFree(const hmpl::State &current);
  bool isSingleStateCollisionFreeImproved(const SimpleNode &current_state);
  bool isSinglePathCollisionFreeImproved(std::vector<SimpleNode> *curve);

  bool findValidClosePose(const grid_map::GridMap& grid_map,
                          const std::string dis_layer_name,
                          const grid_map::Index& start_index,
                          grid_map::Index& adjusted_index,
                          const float required_final_distance,
                          const float desired_final_distance);
  void touchDistanceField(const grid_map::Matrix& dist_trans_map,
                          const grid_map::Index& current_point,
                          const int idx_x,
                          const int idx_y,
                          float& highest_val,
                          grid_map::Index& highest_index)
  {
//    //If no valid expl transform data, return
//    if (dist_trans_map(idx_x, idx_y) == std::numeric_limits<float>::max())
//      return;

    float this_delta = dist_trans_map(idx_x, idx_y) - dist_trans_map(current_point(0),current_point(1));

    if ( (this_delta > 0.0f) && (this_delta > highest_val) ){
      highest_val = this_delta;
      highest_index = grid_map::Index(idx_x, idx_y);
    }
  }
  //
  void clearArea( int xCenter,  int yCenter);
  // ROS param
  std::string path_frame_;        // publishing path frame
  int angle_size_;                // descritized angle size
  double minimum_turning_radius_; // varying by vehicles
  int obstacle_threshold_;        // more than this value is regarded as obstacles
  double goal_radius_;            // meter
  double goal_angle_;             // degree
  bool use_back_;                 // use backward driving
  double robot_length_;
  double robot_width_;
  double robot_wheelbase_;
  double base2back_;
  double curve_weight_;
  double reverse_weight_;
  bool use_wavefront_heuristic_;

  bool node_initialized_;
  std::vector<std::vector<NodeUpdate>> state_update_table_;
  nav_msgs::MapMetaData map_info_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // Pose in global(/map) frame
  geometry_msgs::Pose start_pose_;
  geometry_msgs::Pose goal_pose_;

  // Pose in OccupancyGrid frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;

  // Transform which converts OccupancyGrid frame to global frame
  tf::Transform map2ogm_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Searched path
  nav_msgs::Path path_;
  // Dense path
  nav_msgs::Path dense_path_;

  // gridmap
  hmpl::InternalGridMap igm_;

  // car model
  hmpl::CarGeometry car_;




};

}
#endif // ASTAR_NAVI_NODE_H
