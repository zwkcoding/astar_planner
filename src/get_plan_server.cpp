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

astar_planner::AstarSearch as_planner;
SearchInfo search_info;

bool search_result = false;

bool localPlanCallback(iv_explore_msgs::GetAstarPlan::Request &req, iv_explore_msgs::GetAstarPlan::Response &res) {
    geometry_msgs::Pose local_start_pose, local_goal_pose;
    local_start_pose = req.start.pose;
    local_goal_pose = req.goal.pose;

    ROS_INFO("request start pose: x=%f, y=%f", req.start.pose.position.x, req.start.pose.position.x);
    ROS_INFO("request goal pose: x=%f, y=%f", req.goal.pose.position.x, req.goal.pose.position.x);

    if(search_info.getMapSet()) {

        search_info.resetReceiveMapFlag();

        // Execute astar search
        auto start = std::chrono::system_clock::now();
        search_result = as_planner.makePlan(local_start_pose, local_goal_pose, search_info.getMap());
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        ROS_INFO("astar msec: %lf", msec);

        if (search_result) {
            ROS_INFO("Found GOAL!");
            // sample path by constant length
            as_planner.samplePathByStepLength();
            nav_msgs::Path tmp = as_planner.getDensePath();
            control_msgs::Traj_Node path_node;
            control_msgs::Trajectory trajectory;
            trajectory.header = tmp.header;
            for (int i = 0; i < tmp.poses.size(); i++) {
                path_node.position.x = tmp.poses[i].pose.position.x;
                path_node.position.y = tmp.poses[i].pose.position.y;
                if (tmp.poses[i].pose.position.z == -1) {
                    path_node.forward = 0;  // 0 --> back
                } else {
                    path_node.forward = 1; // 1 --> forward
                }
                path_node.velocity.linear.x = 1.5;
                trajectory.points.push_back(path_node);
            }
            as_planner.reset();
            // 0 --> success; 1 --> invaid start pose; 2 --> invaid goal pose; 3 --> time exceed
            res.status_code = 0;
            res.plan = trajectory;
        } else {
            res.status_code = as_planner.getStatusCode();
        }
        return true;

    } else {
        ROS_WARN("No planning maps received!");
        return false;
    }

    ROS_INFO("Planner status: %d", res.status_code);

}


int main(int argc, char **argv) {
    init(argc, argv, "get_plan_server");
    ros::NodeHandle private_nh_("~");

    std::string map_topic;
    private_nh_.param<std::string>("map_topic", map_topic, "/local_map");

    // ROS subscribers
    ros::Subscriber map_sub = private_nh_.subscribe(map_topic, 1, &SearchInfo::mapCallback, &search_info);
    ServiceServer local_map_srv = private_nh_.advertiseService("get_plan", localPlanCallback);

    spin();
    return 0;

}
