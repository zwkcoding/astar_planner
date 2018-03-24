//
// Created by kevin on 3/24/18.
//

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <iv_explore_msgs/PlannerControlAction.h>
#include "astar_planner/astar_search.h"
#include "astar_planner/search_info_ros.h"
#include <control_msgs/Traj_Node.h>
#include <control_msgs/Trajectory.h>

using namespace astar_planner;

class AstarAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<iv_explore_msgs::PlannerControlAction> as_;
    std::string action_name_;
    iv_explore_msgs::PlannerControlActionFeedback feedback_;
    iv_explore_msgs::PlannerControlActionResult result_;

public:
    AstarAction(std::string name) :
            as_(nh_, name,  false),
            action_name_(name) {

        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&AstarAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&AstarAction::preemptCB, this));

        // ROS param
        std::string map_topic;
        nh_.param<std::string>("map_topic", map_topic, "/local_map");

        // ROS subscribers
        map_sub_ = nh_.subscribe(map_topic, 1, &SearchInfo::mapCallback, &search_info_);
        vehicle_pose_sub_ = nh_.subscribe("/vehicle_global_pose_topic", 1, &SearchInfo::currentPoseCallback, &search_info_);
        // ROS publisher
        nav_path_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 1, false);
        control_trajectory_pub_ = nh_.advertise<control_msgs::Trajectory>("/global_path", 1, false);
        debug_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("debug_pose_array", 1, false);
        footprint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("astar_footprint", 1, false);


    }

    ~AstarAction(void) {

    }

    void executeCB(const iv_explore_msgs::PlannerControlActionGoalConstPtr &goal) {


    }

private:
    AstarSearch astar_;
    SearchInfo search_info_;

    ros::Subscriber map_sub_;
    ros::Subscriber vehicle_pose_sub_;

    ros::Publisher nav_path_pub_, control_trajectory_pub_, debug_pose_pub_, footprint_pub_;



};




int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_planner_server_node");

    AstarAction astar_server("astar_server");
    ros::spin();

    return 0;
}
