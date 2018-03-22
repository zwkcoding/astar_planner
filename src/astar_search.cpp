#include "astar_planner/astar_search.h"
#include <unistd.h>  // usleep
namespace astar_planner {

AstarSearch::AstarSearch()
  : node_initialized_(false)
{

  ros::NodeHandle private_nh_("~");
  private_nh_.param<std::string>("path_frame", path_frame_, "/odom");
  private_nh_.param<int>("angle_size", angle_size_, 48);
  private_nh_.param<double>("minimum_turning_radius", minimum_turning_radius_, 5);
  private_nh_.param<int>("obstacle_threshold", obstacle_threshold_, 70);
  private_nh_.param<double>("goal_radius", goal_radius_, 2);
  private_nh_.param<double>("goal_angle", goal_angle_, 10.0); // unit: degree
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<double>("robot_length", robot_length_, 4.9);
  private_nh_.param<double>("robot_width", robot_width_, 2.8);
  private_nh_.param<double>("base2back", base2back_, 1.09);
  private_nh_.param<double>("curve_weight", curve_weight_, 1.05);
  private_nh_.param<double>("reverse_weight", reverse_weight_, 2.00);
  private_nh_.param<bool>("use_wavefront_heuristic", use_wavefront_heuristic_, true);

  createStateUpdateTable(angle_size_);

}

AstarSearch::~AstarSearch()
{
}

void AstarSearch::resizeNode(int width, int height, int angle_size)
{
  nodes_.resize(height);

  for (int i = 0; i < height; i++)
    nodes_[i].resize(width);

  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++)
      nodes_[i][j].resize(angle_size);
}

void AstarSearch::poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta)
{
  *index_x = pose.position.x / map_info_.resolution;
  *index_y = pose.position.y / map_info_.resolution;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.orientation, quat);
  double yaw = tf::getYaw(quat);
  if (yaw < 0)
    yaw += 2 * M_PI;

  // Descretize angle
  static double one_angle_range = 2 * M_PI / angle_size_;
  *index_theta = yaw / one_angle_range;
  *index_theta %= angle_size_;
}


void AstarSearch::createStateUpdateTable(int angle_size)
{
  // Vehicle moving for each angle
  state_update_table_.resize(angle_size);

  // 6 is the number of steering actions
  // max-left, no-turn, and max-right of forward and backward
  if (use_back_) {
    for (int i = 0; i < angle_size; i++)
      state_update_table_[i].resize(6);
  } else {
    for (int i = 0; i < angle_size; i++)
      state_update_table_[i].resize(3);
  }

  // Minimum moving distance with one state update
  //     arc  = r                       * theta
  double step = minimum_turning_radius_ * (2.0 * M_PI / angle_size_);

  for (int i = 0; i < angle_size; i++) {
    double descretized_angle = 2.0 * M_PI / angle_size;
    double robot_angle = descretized_angle * i;

    // Calculate right and left circle
    // Robot moves along these circles
    double right_circle_center_x = minimum_turning_radius_ * std::sin(robot_angle);
    double right_circle_center_y = minimum_turning_radius_ * std::cos(robot_angle) * -1.0;
    double left_circle_center_x  = right_circle_center_x * -1.0;
    double left_circle_center_y  = right_circle_center_y * -1.0;

    // Calculate x and y shift to next state
    NodeUpdate nu;
    // forward
    nu.shift_x     = step * std::cos(robot_angle);
    nu.shift_y     = step * std::sin(robot_angle);
    nu.rotation    = 0;
    nu.index_theta = 0;
    nu.step        = step;
    nu.curve       = false;
    nu.back        = false;
    state_update_table_[i][0] = nu;

    // forward right
    nu.shift_x     = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle - descretized_angle);
    nu.shift_y     = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle - descretized_angle);
    nu.rotation    = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i][1] = nu;

    // forward left
    nu.shift_x     = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.shift_y     = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.rotation    = descretized_angle;
    nu.index_theta = 1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i][2] = nu;

    // We don't use back move
    if (!use_back_)
      continue;

    // backward
    nu.shift_x     = step * std::cos(robot_angle) * -1.0;
    nu.shift_y     = step * std::sin(robot_angle) * -1.0;
    nu.rotation    = 0;
    nu.index_theta = 0;
    nu.step        = step;
    nu.curve       = false;
    nu.back        = true;
    state_update_table_[i][3] = nu;

    // backward right
    nu.shift_x     = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle + descretized_angle);
    nu.shift_y     = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle + descretized_angle);
    nu.rotation    = descretized_angle;
    nu.index_theta = 1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = true;
    state_update_table_[i][4] = nu;

    // backward left
    nu.shift_x     = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle - descretized_angle);
    nu.shift_y     = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle - descretized_angle);
    nu.rotation    = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = true;
    state_update_table_[i][5] = nu;
  }

}

bool AstarSearch::isOutOfRange(int index_x, int index_y)
{
  if (index_x < 0 || index_x >= static_cast<int>(map_info_.width) || index_y < 0 || index_y >= static_cast<int>(map_info_.height))
    return true;

  return false;
}

void AstarSearch::setPath(const SimpleNode &goal)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = path_frame_;
  path_.header = header;

  // From the goal node to the start node
  AstarNode *node = &nodes_[goal.index_y][goal.index_x][goal.index_theta];

  while (node != NULL) {
    // Set tf pose
    tf::Vector3 origin(node->x, node->y, 0);
    tf::Pose tf_pose;
    tf_pose.setOrigin(origin);
    tf_pose.setRotation(tf::createQuaternionFromYaw(node->theta));

    // Transform path to global frame
    tf_pose = map2ogm_ * tf_pose;

    // Set path as ros message
    geometry_msgs::PoseStamped ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose.pose);
    ros_pose.header = header;
    // use pose.position.z to represent back or forward
    // -1 --> back; 1 --> forward
    if(true == node->back) {
      ros_pose.pose.position.z = -1;
    } else {
      ros_pose.pose.position.z = 1;
    }

    path_.poses.push_back(ros_pose);

    // To the next node
    node = node->parent;
  }

  // Reverse the vector to be start to goal order
  std::reverse(path_.poses.begin(), path_.poses.end());

}

void AstarSearch::samplePathByStepLength(double step) {
     if (this->path_.poses.empty()) {
         return;
     }
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = path_frame_;
    this->dense_path_.header = header;
    this->dense_path_.poses.clear();
    geometry_msgs::PoseStamped ros_pose;
    ros_pose.header = header;
    for (std::size_t i = 0; i < path_.poses.size() - 1 ; i++) {
        double x1, y1, x2, y2;
        x1 = path_.poses.at(i).pose.position.x;
        y1 = path_.poses.at(i).pose.position.y;
        x2 = path_.poses.at(i + 1).pose.position.x;
        y2 = path_.poses.at(i + 1).pose.position.y;
        double t1 = astar::modifyTheta(tf::getYaw(path_.poses.at(i).pose.orientation));
        double t2 = astar::modifyTheta(tf::getYaw(path_.poses.at(i + 1).pose.orientation));
        double delta_abs_t = astar::calcDiffOfRadian(t2, t1);
        double delta_t = t2 - t1;
        double delta_s = std::hypot(x2 - x1, y2 - y1);
        // transfer z(back or forward dir)
        ros_pose.pose.position.z = path_.poses.at(i).pose.position.z;
        // straight line case
        if (delta_t == 0) {
            // calculate how many points need to be inserted by step
            double num = delta_s / step;
            for (int j = 0; j < static_cast<int>(num); j++) {
                ros_pose.pose.position.x = x1 + cos(t1) * step * j;
                ros_pose.pose.position.y = y1 + sin(t2) * step * j;
                ros_pose.pose.orientation = path_.poses.at(i).pose.orientation;
                dense_path_.poses.push_back(ros_pose);
            }
        } else if (delta_t > 0) {
            // left turn
            // arc length
            double R = delta_s / (sqrt(2*(1 - cos(delta_abs_t))));  // no need to cal, R is a constant
            double l = R * delta_abs_t;
            double center_x = x1 + cos(t1 + M_PI / 2.0)*R;
            double center_y = y1 + sin(t1 + M_PI / 2.0)*R;
            // delta arc length now becomes l
            double num = l / step;
            int size = static_cast<int>(num);
            for (int j = 0; j < size; j++) {
                double heading = t1 + j*step / l * delta_abs_t;
                ros_pose.pose.position.x = center_x + cos(heading - M_PI / 2.0)*R;
                ros_pose.pose.position.y = center_y + sin(heading - M_PI / 2.0)*R;
                ros_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
                dense_path_.poses.push_back(ros_pose);
            }
           
        } else {
            // right turn
            // arc length
            double R = delta_s / (sqrt(2*(1 - cos(delta_abs_t))));
            double l = R * delta_abs_t;
            double center_x = x1 + cos(t1 - M_PI / 2.0)*R;
            double center_y = y1 + sin(t1 - M_PI / 2.0)*R;
            // delta arc length now becomes l
            double num = l / step;
            
            int size = static_cast<int>(num);
            for (int j = 0; j < size; j++) {
                double heading = t1 - j*step / l * delta_abs_t;
                ros_pose.pose.position.x = center_x + cos(heading + M_PI / 2.0)*R;
                ros_pose.pose.position.y = center_y + sin(heading + M_PI / 2.0)*R;
                ros_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
                dense_path_.poses.push_back(ros_pose);
            }
        }
    }
}

// theta is 0 ~ 2pi
bool AstarSearch::isGoal(double x, double y, double theta)
{
  // To reduce computation time, we use square value
  static const double goal_radius = goal_radius_ * goal_radius_; // meter
  static const double goal_angle  = M_PI * goal_angle_ / 180.0; // degrees -> radian

  // Check the pose of goal
  if (std::pow(goal_pose_local_.pose.position.x - x, 2) + std::pow(goal_pose_local_.pose.position.y - y, 2) < goal_radius) {
    // Get goal yaw in 0 ~ 2pi
    double goal_yaw = astar::modifyTheta(tf::getYaw(goal_pose_local_.pose.orientation));

    // Check the orientation of goal
    if (astar::calcDiffOfRadian(goal_yaw, theta) < goal_angle)
      return true;
  }

  return false;
}

bool AstarSearch::detectCollision(const SimpleNode &sn)
{
  // Define the robot as rectangle
  static double left   = -1.0 * base2back_;
  static double right  = robot_length_ - base2back_;
  static double top    = robot_width_ / 2.0;
  static double bottom = -1.0 * robot_width_ / 2.0;
  static double resolution = map_info_.resolution;

  // Coordinate of base_link in OccupancyGrid frame
  static double one_angle_range = 2.0 * M_PI / angle_size_;
  double base_x     = sn.index_x * resolution;
  double base_y     = sn.index_y * resolution;
  double base_theta = sn.index_theta * one_angle_range;

  // Calculate cos and sin in advance
  double cos_theta = std::cos(base_theta);
  double sin_theta = std::sin(base_theta);

  // todo zwk
  // Convert each point to index and check if the node is Obstacle
  for (double x = left; x < right; x += resolution) {
    for (double y = top; y > bottom; y -= resolution) {
      // 2D point rotation
      int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
      int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

      if (isOutOfRange(index_x, index_y))
        return true;
      if (nodes_[index_y][index_x][0].status == STATUS::OBS)
        return true;
    }
  }

  return false;
}

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode &sn)
{
  // Set start point for wavefront search
  // This is goal for Astar search
  nodes_[sn.index_y][sn.index_x][0].hc = 0;
  WaveFrontNode wf_node(sn.index_x, sn.index_y, 1e-10);
  std::queue<WaveFrontNode> qu;
  qu.push(wf_node);

  // State update table for wavefront search
  // Nodes are expanded for each neighborhood cells (moore neighborhood)
  double resolution = map_info_.resolution;
  static std::vector<WaveFrontNode> updates = {
    astar::getWaveFrontNode( 0,  1, resolution),
    astar::getWaveFrontNode(-1,  0, resolution),
    astar::getWaveFrontNode( 1,  0, resolution),
    astar::getWaveFrontNode( 0, -1, resolution),
    astar::getWaveFrontNode(-1,  1, std::hypot(resolution, resolution)),
    astar::getWaveFrontNode( 1,  1, std::hypot(resolution, resolution)),
    astar::getWaveFrontNode(-1, -1, std::hypot(resolution, resolution)),
    astar::getWaveFrontNode( 1, -1, std::hypot(resolution, resolution)),
  };

  // Get start index
  int start_index_x;
  int start_index_y;
  int start_index_theta;
  poseToIndex(start_pose_local_.pose, &start_index_x, &start_index_y, &start_index_theta);

  // Whether the robot can reach goal
  bool reachable = false;

  // Start wavefront search
  while (!qu.empty()) {
    WaveFrontNode ref = qu.front();
    qu.pop();

    WaveFrontNode next;
    for (const auto &u : updates) {
      next.index_x = ref.index_x + u.index_x;
      next.index_y = ref.index_y + u.index_y;

      // out of range OR already visited OR obstacle node
      if (isOutOfRange(next.index_x, next.index_y) ||
          nodes_[next.index_y][next.index_x][0].hc > 0 ||
          nodes_[next.index_y][next.index_x][0].status == STATUS::OBS)
        continue;

      // Take the size of robot into account
      // time costy when large free space
      if (detectCollisionWaveFront(next))
        continue;

      // Check if we can reach from start to goal
      if (next.index_x == start_index_x && next.index_y == start_index_y)
        reachable = true;

      // Set wavefront heuristic cost
      next.hc = ref.hc + u.hc;
      nodes_[next.index_y][next.index_x][0].hc = next.hc;

      qu.push(next);
    }
  }

  // End of search
  return reachable;
}

// Simple collidion detection for wavefront search
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode &ref)
{
  // Define the robot as square
  static double half = robot_width_ / 2;
  double robot_x = ref.index_x * map_info_.resolution;
  double robot_y = ref.index_y * map_info_.resolution;

  for (double y = half; y > -1.0 * half; y -= map_info_.resolution) {
    for (double x = -1.0 * half; x < half; x += map_info_.resolution) {
      int index_x = (robot_x + x) / map_info_.resolution;
      int index_y = (robot_y + y) / map_info_.resolution;

      if (isOutOfRange(index_x, index_y)) {
//        ROS_INFO("out of range");
        return true;
      }

      if (nodes_[index_y][index_x][0].status == STATUS::OBS) {
//        ROS_INFO("state is obs");
        return true;
      }
    }
  }

  return false;
}

bool AstarSearch::findValidClosePose(const grid_map::GridMap& grid_map,
                                     const std::string dis_layer_name,
                                     const grid_map::Index& start_index,
                                     grid_map::Index& adjusted_index,
                                     const float required_final_distance,
                                     const float desired_final_distance) {
  grid_map::Index current_index;
  grid_map::Index next_index;
  current_index = start_index;
  std::vector <grid_map::Index> path_indices;
  path_indices.push_back(current_index);
  const grid_map::Matrix& dist_data = grid_map[dis_layer_name];

  float dist_from_goal = 0.0f;
  float dist_from_obstacle = dist_data(current_index(0), current_index(1));

  if (dist_from_obstacle >= desired_final_distance){
    ROS_INFO("Pose already farther than desired distance from next obstacle, not modifying");
    adjusted_index = start_index;
    return true;
  }

  bool abort = false;
  while(!abort)
  {

    // We guarantee in construction of expl. transform that we're not
    // at the border.
    //if (point(0) < 1 || point(0) >= size_x_lim ||
    //    point(1) < 1 || point(1) >= size_y_lim){
    //    continue;
    //}

    //std::cout << "\nStartloop curr_index:\n" << current_index << "\nval: " << expl_data(current_index(0), current_index(1)) << "\n";



    float highest_cost = 0.0f;


    touchDistanceField(dist_data,
                       current_index,
                       current_index(0)-1,
                       current_index(1),
                       highest_cost,
                       next_index);

    touchDistanceField(dist_data,
                       current_index,
                       current_index(0),
                       current_index(1)-1,
                       highest_cost,
                       next_index);

    touchDistanceField(dist_data,
                       current_index,
                       current_index(0),
                       current_index(1)+1,
                       highest_cost,
                       next_index);

    touchDistanceField(dist_data,
                       current_index,
                       current_index(0)+1,
                       current_index(1),
                       highest_cost,
                       next_index);

    touchDistanceField(dist_data,
                       current_index,
                       current_index(0)-1,
                       current_index(1)-1,
                       highest_cost,
                       next_index);

    touchDistanceField(dist_data,
                       current_index,
                       current_index(0)-1,
                       current_index(1)+1,
                       highest_cost,
                       next_index);

    touchDistanceField(dist_data,
                       current_index,
                       current_index(0)+1,
                       current_index(1)-1,
                       highest_cost,
                       next_index);

    touchDistanceField(dist_data,
                       current_index,
                       current_index(0)+1,
                       current_index(1)+1,
                       highest_cost,
                       next_index);


    dist_from_obstacle = dist_data(current_index(0), current_index(1));

    //std::cout << "curr_index: " << current_index << "\ndist_from_obstacle: " << dist_from_obstacle << "dist_from_start: " << dist_from_start << " highest cost: " << highest_cost << "\n";

    // If gradient could not be followed..
    if (highest_cost == 0.0f){

      //std::cout << "curr_index: " << current_index << "\nval: " << dist_data(current_index(0), current_index(1)) << "dist_from_start: " << dist_from_start << " highest cost: " << highest_cost << "\n";

      //ROS_INFO_STREAM("Reached end of distance transform with dist_from_start)

      // Could not reach enough clearance
      if (dist_from_obstacle < required_final_distance){
        ROS_WARN("Could not find gradient of distance transform leading to free area, returning original pose");
        adjusted_index = start_index;
        return false;

        // Could not reach desired distance from obstacles, but enough clearance
      }else if (dist_from_obstacle < desired_final_distance){
        ROS_WARN("Could not find gradient of distance transform reaching desired final distance");
        abort = true;

      }else{
        ROS_INFO("Reached final distance");
        abort = true;
      }

      // Gradient following worked
    }else{

      //dist_from_start += highest_cost;

      // If desired distance exceeded, stop here
      if (dist_from_obstacle > desired_final_distance){
        abort = true;

        // Otherwise continue gradient following
      }else{

        current_index = next_index;
        dist_from_goal += highest_cost;
        path_indices.push_back(current_index);
      }
    }
  }

  adjusted_index = current_index;


  return true;


}

void AstarSearch::reset()
{
  // Clear path
  path_.poses.clear();
  // clear dense path
  dense_path_.poses.clear();

  // Clear queue
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> empty;
  std::swap(openlist_, empty);
}

void AstarSearch::clearArea( int xCenter,  int yCenter) {

  int radius = ceil(robot_width_ / map_info_.resolution);

  int vx[2], vy[2];
  for(int x = xCenter - radius; x <= xCenter; ++x)
    for(int y = yCenter - radius; y <= yCenter; ++y)
      if(1/*(x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter) <= radius*radius*/)
      {
        vx[0] = x;
        vy[0] = y;
        vx[1] = xCenter - (x - xCenter);
        vy[1] = yCenter - (y - yCenter);

        for(int i=0; i<2; ++i)
          for(int j=0; j<2; ++j)
          {
            int m = vx[i], n = vy[j];
            for (int k = 0; k < angle_size_; k++) {
              //nodes_[i][j][k].gc     = 0;
              nodes_[n][m][k].hc     = 0;
              nodes_[n][m][k].status = STATUS::NONE;
              nodes_[n][m][k].parent = NULL;
            }
          }
      }

}
void AstarSearch::setMap(const nav_msgs::OccupancyGrid &map)
{
  map_info_ = map.info;

  // TODO: what frame do we use?
  std::string map_frame = path_frame_;
  std::string ogm_frame = map.header.frame_id;
  // Set transform
  // no need, cuz map_frame is static
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
  geometry_msgs::Pose ogm_in_map = astar::transformPose(map_info_.origin, map2ogm_frame);
  tf::poseMsgToTF(ogm_in_map, map2ogm_);

  // Initialize node according to map size
  if (!node_initialized_) {
    resizeNode(map.info.width, map.info.height, angle_size_);
    node_initialized_ = true;
    const double lengthX = map.info.resolution * map.info.height;
    const double lengthY = map.info.resolution * map.info.width;
    grid_map::Length length(lengthX, lengthY);
    igm_.maps.setGeometry(length, map.info.resolution, grid_map::Position::Zero());
    igm_.maps.setFrameId(map.header.frame_id);
  }
  ros::WallTime begin = ros::WallTime::now();

  // from occupancy to gridmap
  bool success = grid_map::GridMapRosConverter::fromOccupancyGrid(map, igm_.obs, igm_.maps);
  if(!success) {
    ROS_ERROR("fail to create gridmap");
  }
//  igm_.updateDistanceLayerCV();
//  ros::WallTime end = ros::WallTime::now();
//  std::cout << "gridmap process time: " << (end - begin).toSec() * 1000 << "[ms]" << std::endl;

  for (size_t i = 0; i < map.info.height; i++) {
    for (size_t j = 0; j < map.info.width; j++) {
      // Index of subscribing OccupancyGrid message
      size_t og_index = i * map.info.width + j;
      int cost = map.data[og_index];

      // more than threshold or unknown area
      if (cost > obstacle_threshold_ || cost < 0 ) {
        nodes_[i][j][0].status = STATUS::OBS;
      }
      else
      {
        for (int k = 0; k < angle_size_; k++) {
          //nodes_[i][j][k].gc     = 0;
          nodes_[i][j][k].hc     = 0;
          nodes_[i][j][k].status = STATUS::NONE;
          nodes_[i][j][k].parent = NULL;
        }
      }

    }
  }

}

bool AstarSearch::setStartNode()
{
  // Get index of start pose
  int index_x;
  int index_y;
  int index_theta;
  poseToIndex(start_pose_local_.pose, &index_x, &index_y, &index_theta);
  SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);

  // Check if start is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(start_sn))
    return false;

  // Set start node
  AstarNode &start_node = nodes_[index_y][index_x][index_theta];
  start_node.x      = start_pose_local_.pose.position.x;
  start_node.y      = start_pose_local_.pose.position.y;
  start_node.theta  = 2.0 * M_PI / angle_size_ * index_theta;
  start_node.gc     = 0;
  start_node.back   = false;
  start_node.status = STATUS::OPEN;
  if (!use_wavefront_heuristic_)
    start_node.hc = astar::calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y);

  // Push start node to openlist
  start_sn.cost = start_node.gc + start_node.hc;
  openlist_.push(start_sn);
  return true;
}

bool AstarSearch::setGoalNode()
{
  // Get index of goal pose
  int index_x;
  int index_y;
  int index_theta;
  poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);
  auto start = std::chrono::system_clock::now();
  clearArea(index_x, index_y);
  auto end = std::chrono::system_clock::now();
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
//  std::cout << "clear goal neighber cost: " << usec / 1000.0 <<  "[ms]" <<  '\n';
  SimpleNode goal_sn(index_x, index_y, index_theta, 0, 0);

//  grid_map::Index current_index;

//  if (!igm_.maps.getIndex(grid_map::Position(goal_pose_local_.pose.position.x + map_info_.origin.position.x,
//                                             goal_pose_local_.pose.position.y + map_info_.origin.position.y),current_index)){
//    ROS_WARN("goal index not in map");
//    return false;
//  }

  // Check if goal is valid
  if (isOutOfRange(index_x, index_y) /*|| detectCollision(goal_sn)*/)
    return false;
//  if(detectCollision(goal_sn)) {
//    ROS_INFO("goal Pose invalid, modifying");
//    grid_map::Index adjusted_index;
//    findValidClosePose(igm_.maps, igm_.dis, current_index, adjusted_index, 3, 6);
//    grid_map::Position adjusted_position;
//
//    igm_.maps.getPosition(adjusted_index, adjusted_position);
//
//    goal_pose_local_.pose.position.x = adjusted_position[0] - map_info_.origin.position.x;
//    goal_pose_local_.pose.position.y = adjusted_position[1] - map_info_.origin.position.y;
//    poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);
//    goal_sn.index_x = index_x;
//    goal_sn.index_y = index_y;
//    ROS_INFO("Adjusted goal pose :%f, %f ",goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y);
//
//  } else {
//    ROS_INFO("goal Pose valid, not modifying");
//  }

  // Calculate wavefront heuristic cost
  if (use_wavefront_heuristic_) {
    auto start = std::chrono::system_clock::now();
    bool wavefront_result = calcWaveFrontHeuristic(goal_sn);
    auto end = std::chrono::system_clock::now();
    auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
//    std::cout << "wavefront cost: " << usec / 1000.0 <<  "[ms]" <<  '\n';

    if (!wavefront_result) {
      ROS_WARN("Goal is not reachable...");
      return false;
    }
  }

  return true;
}

bool AstarSearch::search()
{
  // -- Start Astar search ----------
  // If the openlist is empty, search failed
  ros::WallTime begin = ros::WallTime::now();
  while (!openlist_.empty()) {

    // Terminate the search if the count reaches a certain value
    ros::WallTime end = ros::WallTime::now();
    float elapse_time = (end - begin).toSec() * 1000;
    if(elapse_time > 500) {
      ROS_WARN("Exceed time limit");
      std::cout << "Exceed time limit: " << (end - begin).toSec() * 1000 << "[ms]" << '\n';
      return false;
    }

    // Pop minimum cost node from openlist
    SimpleNode sn;
    sn = openlist_.top();
    openlist_.pop();
    nodes_[sn.index_y][sn.index_x][sn.index_theta].status = STATUS::CLOSED;

    // Expand include from this node
    AstarNode *current_node = &nodes_[sn.index_y][sn.index_x][sn.index_theta];

    // for each update
    for (const auto &state : state_update_table_[sn.index_theta]) {
      // Next state
      double next_x     = current_node->x + state.shift_x;
      double next_y     = current_node->y + state.shift_y;
      double next_theta = astar::modifyTheta(current_node->theta + state.rotation);
      double move_cost  = state.step;

#if DEBUG
      // Display search process
      geometry_msgs::Pose p;
      p.position.x = next_x;
      p.position.y = next_y;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(next_theta), p.orientation);
      p = astar::transformPose(p, map2ogm_);
      debug_pose_array_.poses.push_back(p);
#endif

      // Increase curve cost
      if (state.curve)
        move_cost *= curve_weight_;

      // Increase reverse cost
      if (current_node->back != state.back)
        move_cost *= reverse_weight_;

      // Calculate index of the next state
      SimpleNode next;
      next.index_x     = next_x / map_info_.resolution;
      next.index_y     = next_y / map_info_.resolution;
      next.index_theta = sn.index_theta + state.index_theta;
      // Avoid invalid index
      next.index_theta = (next.index_theta + angle_size_) % angle_size_;

      // Check if the index is valid
      if (isOutOfRange(next.index_x, next.index_y) || detectCollision(next))
        continue;

      AstarNode *next_node = &nodes_[next.index_y][next.index_x][next.index_theta];
      double next_hc       =  nodes_[next.index_y][next.index_x][0].hc;

      // Calculate euclid distance heuristic cost
      if (!use_wavefront_heuristic_)
        next_hc = astar::calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y);

      // GOAL CHECK
      if (isGoal(next_x, next_y, next_theta)) {
        next_node->status = STATUS::OPEN;
        next_node->x      = next_x;
        next_node->y      = next_y;
        next_node->theta  = next_theta;
        next_node->gc     = current_node->gc + move_cost;
        next_node->hc     = next_hc;
        next_node->back   = state.back;
        next_node->parent = current_node;

        setPath(next);
        return true;
      }

      // NONE
      if (next_node->status == STATUS::NONE) {
        next_node->status = STATUS::OPEN;
        next_node->x      = next_x;
        next_node->y      = next_y;
        next_node->theta  = next_theta;
        next_node->gc     = current_node->gc + move_cost;
        next_node->hc     = next_hc;
        next_node->back   = state.back;
        next_node->parent = current_node;

        next.cost = next_node->gc + next_node->hc;
        openlist_.push(next);
        continue;
      }

      // OPEN or CLOSED
      if (next_node->status == STATUS::OPEN || next_node->status == STATUS::CLOSED) {
        if (current_node->gc + move_cost + next_hc < next_node->gc + next_hc) {
          next_node->status = STATUS::OPEN;
          next_node->x      = next_x;
          next_node->y      = next_y;
          next_node->theta  = next_theta;
          next_node->gc     = current_node->gc + move_cost;
          next_node->hc     = next_hc;
          next_node->back   = state.back;
          next_node->parent = current_node;

          next.cost = next_node->gc + next_node->hc;
          openlist_.push(next);
          continue;
        }
      }

    } // state update

  }

  // Failed to find path
  ROS_WARN("Openlist is Empty!");
  return false;
}

bool AstarSearch::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, const nav_msgs::OccupancyGrid &map)
{
  start_pose_local_.pose = start_pose;
  goal_pose_local_.pose  = goal_pose;
  ros::WallTime begin = ros::WallTime::now();
  setMap(map);
  ros::WallTime end = ros::WallTime::now();
//  std::cout << "set map time: " << (end - begin).toSec() * 1000 << "[ms]" << '\n';
  if (!setStartNode()) {
    ROS_WARN("Invalid start pose!");
    return false;
  }

  if (!setGoalNode()) {
    ROS_WARN("Invalid goal pose!");
    return false;
  }
  auto start = std::chrono::system_clock::now();
  bool result = search();
  auto end0 = std::chrono::system_clock::now();
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end0 - start).count();
//  std::cout << "astar search process cost: " << usec / 1000.0 <<  "[ms]" <<  '\n';

  return result;
}

// for debug
void AstarSearch::publishPoseArray(const ros::Publisher &pub, const std::string &frame)
{
  // display astar node expanding process
  debug_pose_array_.header.frame_id = frame;
  debug_pose_array_.header.stamp = ros::Time();;
  debug_pose_array_.poses.push_back(start_pose_.pose);
  debug_pose_array_.poses.push_back(goal_pose_.pose);

  pub.publish(debug_pose_array_);

  debug_pose_array_.poses.clear();
}

void AstarSearch::publishFootPrint(const ros::Publisher &pub, const std::string &frame) {
  // displayFootprint
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.ns = "footprint";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = robot_length_;
  marker.scale.y = robot_width_;
  marker.scale.z = 2.0;
  marker.color.a = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.frame_locked = true;

  visualization_msgs::MarkerArray marker_array;
  for (const auto &p : path_.poses)
  {
    marker.pose = p.pose;
    marker_array.markers.push_back(marker);
    marker.id += 1;
  }
  pub.publish(marker_array);
}

// for demo
void AstarSearch::broadcastPathTF()
{
  tf::Transform transform;

  // Broadcast from start pose to goal pose
  for (unsigned int i = 0; i < path_.poses.size(); i++) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(path_.poses[i].pose.orientation, quat);
    transform.setOrigin(tf::Vector3(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y, path_.poses[i].pose.position.z));
    transform.setRotation(quat);

    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), path_frame_, "/astar_path"));

    // sleep 0.01 [sec]
    usleep(1000000);
  }

}

}
