//
// Created by kevin on 3/24/18.
//

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opt_utils/opt_utils.hpp>
#include <iv_explore_msgs/GetAstarPlan.h>
#include <control_msgs/Traj_Node.h>
#include <control_msgs/Trajectory.h>

#include "astar_planner/astar_search.h"
#include "astar_planner/search_info_ros.h"

using namespace ros;

boost::shared_ptr<astar_planner::AstarSearch> as_planner_ptr;
boost::shared_ptr<SearchInfo> search_info_ptr;
boost::shared_ptr<tf::TransformListener> tf_listener_ptr;

ros::Publisher path_pub, debug_pose_pub, footprint_pub;
std::string receive_planner_map_topic_name, global_map_frame_name, local_map_frame_name, abso_global_map_frame_name;
tf::Transform ogm2map_;

void convertIntoPlannerOgm(geometry_msgs::PoseStamped &msg, geometry_msgs::Pose &local_pose) {
    // get tf from odom To ogm
    ogm2map_ = search_info_ptr->getTfFromOdomToOgm();
    std::string global_frame  = global_map_frame_name;
    std::string goal_frame  = msg.header.frame_id;

    // First, transform into global_map("/odom")
    tf::StampedTransform world2map;
    try
    {
        tf_listener_ptr->lookupTransform(global_frame, goal_frame, ros::Time(0), world2map);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    // Set pose in Global frame
    geometry_msgs::Pose goal_pose_global = astar::transformPose(msg.pose, world2map);
    // Set pose in planner_map frame
    local_pose = astar::transformPose(goal_pose_global, ogm2map_);
}

bool localPlanCallback(iv_explore_msgs::GetAstarPlan::Request &req, iv_explore_msgs::GetAstarPlan::Response &res) {
    geometry_msgs::Pose local_start_pose, local_goal_pose;
    geometry_msgs::PoseStamped global_start_pose, global_goal_pose;
    // default : goal and start is in odom frame
    global_start_pose.header.frame_id = global_goal_pose.header.frame_id = global_map_frame_name;
    global_goal_pose.pose = req.goal;
    global_start_pose.pose = req.start;
    // convert into planner map
    convertIntoPlannerOgm(global_start_pose, local_start_pose);
    convertIntoPlannerOgm(global_goal_pose, local_goal_pose);

  //  ROS_INFO_THROTTLE(5,"request start pose in odom frame: x=%f, y=%f, theta=%f", req.start.position.x, req.start.position.y, tf::getYaw(req.start.orientation) * 180 / M_PI);
  //  ROS_INFO_THROTTLE(5,"request start pose in planner ogm map: x=%f, y=%f, theta=%f", local_start_pose.position.x, local_start_pose.position.y, tf::getYaw(local_start_pose.orientation) * 180 / M_PI);

  //  ROS_INFO_THROTTLE(5, "request goal pose in odom frame: x=%f, y=%f, theta=%f", req.goal.position.x, req.goal.position.y, tf::getYaw(req.goal.orientation) * 180 / M_PI);
    ROS_INFO_THROTTLE(1, "request goal pose in planner ogm map: x=%f, y=%f, theta=%f", local_goal_pose.position.x, local_goal_pose.position.y, tf::getYaw(local_goal_pose.orientation) * 180 / M_PI);

    if(search_info_ptr->getMapSet()) {
        // reset receive_map flag considering time delay
        search_info_ptr->resetReceiveMapFlag();

        // Execute astar search
        auto start = std::chrono::system_clock::now();
        bool search_result = as_planner_ptr->makePlan(local_start_pose, local_goal_pose, search_info_ptr->getMap());
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        ROS_INFO_THROTTLE(3, "astar msec: %lf", msec);

        if (search_result) {
            ROS_INFO_THROTTLE(5, "Found GOAL!");
            // sample path by constant length
//            as_planner.samplePathByStepLength();

            // transform path into "control" path
            nav_msgs::Path tmp = as_planner_ptr->getPath();  // use path, not sampled arc
            path_pub.publish(tmp);
            control_msgs::Traj_Node path_node;
            control_msgs::Trajectory trajectory;
#ifndef CONTROL_LOCAL_PATH
            // Get transform (map to world in Autoware)
                tf::StampedTransform world2map;
                try
                {
                    tf_listener_ptr->lookupTransform(abso_global_map_frame_name, global_map_frame_name, ros::Time(0), world2map);
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s", ex.what());
//                    return;
                }

                trajectory.header.frame_id = global_map_frame_name;
                trajectory.header.stamp = ros::Time::now();
                for (int i = 0; i < tmp.poses.size(); i++) {
                      // decide forward before world2map
                    if (tmp.poses[i].pose.position.z == -1) {
                        path_node.forward = 0;  // 0 --> back
                    } else {
                        path_node.forward = 1; // 1 --> forward
                    }
                    tmp.poses[i].pose = astar::transformPose(tmp.poses[i].pose, world2map);
                    path_node.position.x = tmp.poses[i].pose.position.x;
                    path_node.position.y = tmp.poses[i].pose.position.y;
                    path_node.velocity.linear.x = 3;

                    if(i > tmp.poses.size() - 5 && i < tmp.poses.size() - 2)
                        path_node.velocity.linear.x = 1;
                    else if(i > tmp.poses.size() - 2)
                        path_node.velocity.linear.x = 0;

                    trajectory.points.push_back(path_node);
                }

#else
            // Get transform (map to world in Autoware)
            tf::StampedTransform world2map;
            try
            {
                tf_listener_ptr->lookupTransform(local_map_frame_name, global_map_frame_name, ros::Time(0), world2map);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
//                    return;
            }

            trajectory.header.frame_id = local_map_frame_name;
            trajectory.header.stamp = ros::Time::now();
            for (int i = 0; i < tmp.poses.size(); i++) {
                // decide forward before world2map
                if (tmp.poses[i].pose.position.z == -1) {
                    path_node.forward = 0;  // 0 --> back
                } else {
                    path_node.forward = 1; // 1 --> forward
                }
                tmp.poses[i].pose = astar::transformPose(tmp.poses[i].pose, world2map);
                path_node.position.x = tmp.poses[i].pose.position.x;
                path_node.position.y = tmp.poses[i].pose.position.y;
                path_node.velocity.linear.x = 1.5;
                trajectory.points.push_back(path_node);
            }

#endif
            ROS_INFO_THROTTLE(3, "Path node size is %d", trajectory.points.size());
            res.path = trajectory;
#ifdef DEBUG
            as_planner_ptr->publishPoseArray(debug_pose_pub, global_map_frame_name);
            as_planner_ptr->publishFootPrint(footprint_pub, global_map_frame_name);
#endif
            res.status_code = 0;
        } else {
            ROS_INFO("Fail to find GOAL!");
            // 0 --> success; 1 --> invaid start pose; 2 --> invaid goal pose; 3 --> time exceed
            res.status_code = as_planner_ptr->getStatusCode();
            ROS_INFO("Planner Fail status code : %d", res.status_code);

        }
        // clear astar_planner search cache path in any condition: even fail
        as_planner_ptr->reset();
        return true;
    } else {
        ROS_WARN("No planning maps received!");
        return false;
    }

}


int main(int argc, char **argv) {
    init(argc, argv, "get_plan_server");
    ros::NodeHandle private_nh_("~");
    ros::NodeHandle nh_;

//    pListener = new (tf::TransformListener);
    as_planner_ptr.reset(new astar_planner::AstarSearch());
    search_info_ptr.reset(new SearchInfo);
    tf_listener_ptr.reset(new tf::TransformListener);
    private_nh_.param<std::string>("receive_planner_map_topic_name", receive_planner_map_topic_name, "/local_map");
    private_nh_.param<std::string>("global_map_frame_name", global_map_frame_name, "/odom");
    private_nh_.param<std::string>("local_map_frame_name", local_map_frame_name, "base_link");
    private_nh_.param<std::string>("abso_global_map_frame_name", abso_global_map_frame_name, "/abso_odom");

    // ROS subscribers
    ros::Subscriber map_sub = nh_.subscribe(receive_planner_map_topic_name, 1, &SearchInfo::mapCallback, search_info_ptr);
    ServiceServer local_map_srv = nh_.advertiseService("get_plan", localPlanCallback);

    path_pub = nh_.advertise<nav_msgs::Path>(nh_.getNamespace() + "global_path_show", 1, false);
    debug_pose_pub = nh_.advertise<geometry_msgs::PoseArray>(nh_.getNamespace() + "debug_pose_array", 1, false);
    footprint_pub = nh_.advertise<visualization_msgs::MarkerArray>(nh_.getNamespace() + "astar_footprint", 1, false);

    spin();
    return 0;

}
