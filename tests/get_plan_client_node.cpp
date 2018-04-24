//
// Created by kevin on 4/5/18.
//

#include <ros/ros.h>
#include <iv_explore_msgs/GetAstarPlan.h>
#include "astar_planner/astar_search.h"
#include "astar_planner/search_info_ros.h"

geometry_msgs::Pose global_goal_pose, global_start_pose;
bool start_flag = false,  goal_flag = false;

void goalPoseCallback (const geometry_msgs::PoseStampedConstPtr &msg) {

//    ROS_INFO("Subcscribed goal pose!");
    double yaw = tf::getYaw(msg->pose.orientation);
//    for(int i = 0; i < 20; i++) {
//        ROS_INFO_STREAM("goal frame is in :" <<  msg->header.frame_id);
//        ROS_INFO("goal cell [in global_map coord]: [%f[m], %f[m], %f[degree]]", msg->pose.position.x,
//                 msg->pose.position.y, yaw * 180 / M_PI);
//    }

    global_goal_pose = msg->pose;
    goal_flag = true;
}

void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    ROS_INFO("Subcscribed start pose!");

    global_start_pose = msg->pose.pose;
    start_flag = true;
}

void currentPoseCallback(const nav_msgs::OdometryConstPtr &msg) {
    global_start_pose = msg->pose.pose;
    // for vrep, convert pose into odom frame
    static int count = 0;
    static double base_x, base_y;
    if(0 == count) {
        base_x = global_start_pose.position.x;
        base_y = global_start_pose.position.y;
        global_start_pose.position.x -= base_x;
        global_start_pose.position.y -= base_y;
        count = 1;
    }
    global_start_pose.position.x -= base_x;
    global_start_pose.position.y -= base_y;

    start_flag = true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "get_plan_client");
    ros::NodeHandle n;

    SearchInfo search_info;
    control_msgs::Trajectory result_path;

    // ROS subscribers
//    ros::Subscriber start_sub = n.subscribe("/initialpose", 1, startCallback);
    ros::Subscriber start_sub = n.subscribe("/odom", 1, currentPoseCallback);
    ros::Subscriber goal_sub  = n.subscribe("/move_base_simple/goal", 1, goalPoseCallback);
    // true:  a persistent connection
    ros::ServiceClient plan_client = n.serviceClient<iv_explore_msgs::GetAstarPlan>(std::string("get_plan"));
    ros::Publisher trajectory_pub = n.advertise<control_msgs::Trajectory>("/global_path", 1, false);

    ros::Rate rate(100);
    while(n.ok()) {
        ros::spinOnce();
        if(!start_flag || !goal_flag) {
            rate.sleep();
            continue;
        }
        start_flag = goal_flag = true;

        iv_explore_msgs::GetAstarPlan srv;
        srv.request.start = global_start_pose;
        srv.request.goal = global_goal_pose;
        if (!plan_client.isValid()) {
            ROS_WARN("persistent astar planner service has dropped !");
            continue;
        }
        if (plan_client.call(srv)) {
            if (0 == srv.response.status_code) {
                result_path = srv.response.path;
                trajectory_pub.publish(result_path);
                ROS_INFO("FIND GOAL!, Path size is %d", result_path.points.size());
            }
            ROS_INFO("Planner status: %d",srv.response.status_code);
        } else {
            ROS_WARN("astar planner service callback fail : Could not get a planner map.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}