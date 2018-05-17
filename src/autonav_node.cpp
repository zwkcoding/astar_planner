#include "astar_planner/astar_search.h"
#include "astar_planner/search_info_ros.h"
#include <opt_utils/opt_utils.hpp>
#include <vector>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <control_msgs/Traj_Node.h>
#include <control_msgs/Trajectory.h>

//#define Time_Profile
#include <ros/package.h>

using namespace astar_planner;

void saveStatePath(const nav_msgs::Path &path) {
    hmpl::State st;
    std::vector<hmpl::State> state_path;
    for (const auto &pt: path.poses) {
        st.x = pt.pose.position.x;
        st.y = pt.pose.position.y;
        st.z = tf::getYaw(pt.pose.orientation);
        state_path.push_back(st);
    }

    // let's calculate the s
    for (std::size_t i = 1; i < state_path.size(); i++) {
        double delta_s = std::hypot(state_path.at(i).x - state_path.at(i - 1).x,
                                    state_path.at(i).y - state_path.at(i - 1).y);
        state_path.at(i).s = state_path.at(i - 1).s + delta_s;
    }

    // curvature
    for (std::size_t i = 1; i < state_path.size() - 1; i++) {
        hmpl::Vector2D<double> pt1(state_path.at(i - 1).x, state_path.at(i - 1).y);
        hmpl::Vector2D<double> pt2(state_path.at(i).x, state_path.at(i).y);
        hmpl::Vector2D<double> pt3(state_path.at(i + 1).x, state_path.at(i + 1).y);
        state_path.at(i).k = hmpl::getCurvature(pt1, pt2, pt3);
    }
    state_path.back().k = 0;

    // this
    // fill the dkappa  forward diff
    for (std::size_t j = 1; j < state_path.size() - 1; j++) {
        double delta_f = state_path.at(j).k - state_path.at(j - 1).k;
        double delta_s = state_path.at(j).s - state_path.at(j - 1).s;
        if (delta_s != 0) {
            state_path.at(j).dk = delta_f / delta_s;
        } else {
            state_path.at(j).dk = 0;
        }
    }
    state_path.back().dk = 0;

    hmpl::CSVFile astar_path("astar_path.csv");
    astar_path << "s" << "x" << "y" << "z" << "k" << "dk" << hmpl::endrow;
    for (const auto &pt_itr: state_path) {
        astar_path << pt_itr.s << pt_itr.x << pt_itr.y << pt_itr.z << pt_itr.k << pt_itr.dk << hmpl::endrow;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_navi");
    ros::NodeHandle n;
    ros::NodeHandle private_nh_("~");

    std::string node_name = ros::this_node::getName();
#ifdef Time_Profile
    hmpl::CSVFile astar_runtime("astar_runtime.csv");
    astar_runtime << "index" << "t" << hmpl::endrow;
    int runtime_counter = 0;
#endif
    std::string image_name;
    private_nh_.param<std::string>("image_name", image_name, "large_U_obs.png");
    std::string package_dir = ros::package::getPath("astar_planner");
    std::string img_dir = "/images/" + image_name;
    cv::Mat img_src = cv::imread(package_dir + img_dir, CV_8UC1);
    double resolution = 0.2;  // in meter
    hmpl::InternalGridMap in_gm;
    // set the 2d position of the center point of grid map in the grid map frame
    in_gm.initializeFromImage(img_src, resolution, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(img_src, 0.5);
    in_gm.updateDistanceLayer();
    in_gm.maps.setFrameId("/odom");
    ROS_INFO("Created map with size %f x %f m (%i x %i cells), map resolution is %f",
             in_gm.maps.getLength().x(), in_gm.maps.getLength().y(),
             in_gm.maps.getSize()(0), in_gm.maps.getSize()(1), in_gm.maps.getResolution());



    std::string map_topic, start_pose_topic, goal_pose_topic ;
    std::string se_result_global_path_send_topic;
    std::string global_map_frame_name, local_map_frame_name, abso_global_map_frame_name;
    private_nh_.param<std::string>("global_map_frame_name", global_map_frame_name, "/odom");
    private_nh_.param<std::string>("local_map_frame_name", local_map_frame_name, "base_link");
    private_nh_.param<std::string>("abso_global_map_frame_name", abso_global_map_frame_name, "/abso_odom");

    private_nh_.param<std::string>("receive_start_pose_topic_name", start_pose_topic, "/odom");
    private_nh_.param<std::string>("send_result_global_path_send_topic", se_result_global_path_send_topic, "/global_path");

#ifndef PLAN_IN_LOCAL_MAP
    private_nh_.param<std::string>("re_planner_map_topic_name", map_topic, "/global_map");
    private_nh_.param<std::string>("re_goal_pose_topic_name", goal_pose_topic, "/move_base_simple/goal");

#else
    private_nh_.param<std::string>("re_planner_map_topic_name", map_topic, "/explore_entry_map");
    private_nh_.param<std::string>("re_goal_pose_topic_name", goal_pose_topic, "/local_search_goal");

#endif

    AstarSearch astar;
    SearchInfo search_info;

    // ROS subscribers
    ros::Subscriber map_sub = n.subscribe(map_topic, 1, &SearchInfo::mapCallback, &search_info);
    ros::Subscriber start_sub = n.subscribe("/initialpose", 1, &SearchInfo::startCallback, &search_info);
    //ros::Subscriber start_sub = n.subscribe(start_pose_topic, 1, &SearchInfo::currentPoseCallback, &search_info);
//    ros::Subscriber goal_sub  = n.subscribe(goal_pose_topic, 1, &SearchInfo::goalCallback, &search_info);
    ros::Subscriber goal_sub  = n.subscribe("/move_base_simple/goal", 1, &SearchInfo::goalCallback, &search_info);
    // ROS publishers
    ros::Publisher path_pub = private_nh_.advertise<nav_msgs::Path>("global_path_show", 1, false);
    ros::Publisher trajectory_pub = n.advertise<control_msgs::Trajectory>(se_result_global_path_send_topic, 1, false);
    ros::Publisher debug_pose_pub = n.advertise<geometry_msgs::PoseArray>(node_name + "debug_pose_array", 1, false);
    ros::Publisher footprint_pub_ = n.advertise<visualization_msgs::MarkerArray>(node_name + "astar_footprint", 1, false);
    // create map publisher
    ros::Publisher map_publisher =
            n.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);
    tf::TransformListener tf_listener_;
    auto last_timestamp = std::chrono::system_clock::now();

    geometry_msgs::Pose start_pose,goal_pose;
    double yaw0, yaw1;
    private_nh_.param<double>("s_x", start_pose.position.x, 15);
    private_nh_.param<double>("s_y", start_pose.position.y, 70);
    private_nh_.param<double>("s_z", yaw0, -86);
    start_pose.orientation = tf::createQuaternionMsgFromYaw(yaw0 * M_PI / 180.0);
    private_nh_.param<double>("g_x", goal_pose.position.x, 15);
    private_nh_.param<double>("g_y", goal_pose.position.y, 23);
    private_nh_.param<double>("g_z", yaw1, 90);
    goal_pose.orientation = tf::createQuaternionMsgFromYaw(yaw1 * M_PI / 180.0);

    ros::Rate loop_rate(100);  // hz
    while (ros::ok()) {
        ros::spinOnce();

        // Add data to grid map.
        ros::Time time = ros::Time::now();
        // publish the grid_map
        in_gm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(
                in_gm.maps, in_gm.obs, in_gm.FREE, in_gm.OCCUPY, message);
        map_publisher.publish(message);

        // warn: path send hz depends on map/position msg hz
        if (!search_info.getMapSet() ) {
            loop_rate.sleep();
            continue;
        }

        // Reset flag : update current start pose and map env
//        search_info.reset();

        // new next goal received and not reached
        if(false == search_info.goal_update_flag_) {
            auto start = std::chrono::system_clock::now();
            // Execute astar search
            astar.init(search_info.getMap());
            if(search_info.getStartSet())
                start_pose = search_info.getStartPose().pose;
            if(search_info.getGoalSet())
                goal_pose = search_info.getGoalPose().pose;
            bool result = astar.makePlan(start_pose, goal_pose, search_info.getMap());

            auto end = std::chrono::system_clock::now();
            auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
            ROS_INFO_THROTTLE(1, "astar msec: %lf", msec);

            if (result) {

                ROS_INFO_THROTTLE(1, "Found GOAL!");
                // sample path by constant length
//                astar.samplePathByStepLength();

                // convert msg : from path to traj
                path_pub.publish(astar.getPath());
                nav_msgs::Path tmp = astar.getPath();  // use path, not sampled arc
                saveStatePath(astar.getPath());
      /*          control_msgs::Traj_Node path_node;
                control_msgs::Trajectory trajectory;
#ifndef CONTROL_LOCAL_PATH

                // Get transform (map to world in Autoware)
                tf::StampedTransform world2map;
                try
                {
                    tf_listener_.lookupTransform(abso_global_map_frame_name, global_map_frame_name, ros::Time(0), world2map);
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
                    path_node.velocity.linear.x = 1.5;
                    trajectory.points.push_back(path_node);
                }

#else
                // Get transform (map to world in Autoware)
                tf::StampedTransform world2map;
                try
                {
                    tf_listener_.lookupTransform(local_map_frame_name, global_map_frame_name, ros::Time(0), world2map);
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

                trajectory_pub.publish(trajectory);*/
#if DEBUG
                auto time = std::chrono::system_clock::now();
                auto msec = std::chrono::duration_cast<std::chrono::microseconds>(time - last_timestamp).count() / 1000.0;
//                ROS_INFO_THROTTLE(1, "control_msg send period: %lf[msec]", msec);
                last_timestamp = time;
#endif

#ifdef Time_Profile
                if (runtime_counter < 101) {
                    astar_runtime << runtime_counter << msec << hmpl::endrow;
                    runtime_counter++;
                }
                saveStatePath(astar.getPath()/*getDensePath()*/);
#endif

#if DEBUG
                astar.publishPoseArray(debug_pose_pub, global_map_frame_name);
                astar.publishFootPrint(footprint_pub_, global_map_frame_name);
//            astar.broadcastPathTF();
#endif

            } else {
                for(int i = 0; i < 10; i++) {
                    ROS_INFO("can't find path!");
                }
                search_info.goal_update_flag_ = true;
                search_info.resetGoalFlag(); // enable receive new goal

                astar.publishPoseArray(debug_pose_pub, global_map_frame_name);

            }
        } else {

            ROS_INFO("goal is unaccessible or goal is reached! waiting for new goal!");

        }

        astar.reset();
        loop_rate.sleep();
    }

    return 0;
}