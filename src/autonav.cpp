#include "astar_planner/astar_search.h"
#include "astar_planner/search_info_ros.h"
#include <opt_utils/opt_utils.hpp>
#include <vector>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <internal_grid_map/internal_grid_map.hpp>

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

    hmpl::CSVFile astar_runtime("astar_runtime.csv");
    astar_runtime << "index" << "t" << hmpl::endrow;
    int runtime_counter = 0;
    std::string map_topic;
    private_nh_.param<std::string>("map_topic", map_topic, "grid_map");

    std::string package_dir = ros::package::getPath("astar_planner");
    std::string img_dir = "/obstacles.png";
    cv::Mat img_src = cv::imread(package_dir + img_dir, CV_8UC1);
    double resolution = 0.2;  // in meter
    hmpl::InternalGridMap in_gm;
    // set the 2d position of the center point of grid map in the grid map frame
    in_gm.initializeFromImage(img_src, resolution, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(img_src, 0.5);
    in_gm.updateDistanceLayer();
    in_gm.maps.setFrameId("odom");
    ROS_INFO("Created map with size %f x %f m (%i x %i cells), map resolution is %f",
             in_gm.maps.getLength().x(), in_gm.maps.getLength().y(),
             in_gm.maps.getSize()(0), in_gm.maps.getSize()(1), in_gm.maps.getResolution());
    // create map publisher
    ros::Publisher map_publisher =
            n.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);

    AstarSearch astar;
    SearchInfo search_info;

    // ROS subscribers
    ros::Subscriber map_sub = n.subscribe(map_topic, 1, &SearchInfo::mapCallback, &search_info);
    ros::Subscriber start_sub = n.subscribe("/initialpose", 1, &SearchInfo::startCallback, &search_info);
    ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1, &SearchInfo::goalCallback, &search_info);

    // ROS publishers
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("astar_path", 1, true);
    ros::Publisher debug_pose_pub = n.advertise<geometry_msgs::PoseArray>("debug_pose_array", 1, true);
    ros::Publisher footprint_pub_ = n.advertise<visualization_msgs::MarkerArray>("astar_footprint", 1, true);

    ros::Rate loop_rate(10);
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
//        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published, origin position: (%f,%f)",
//                          message.header.stamp.toSec(), message.info.origin.position.x,
//                          message.info.origin.position.y);

        if (!search_info.getMapSet() || !search_info.getStartSet() || !search_info.getGoalSet()) {
            loop_rate.sleep();
            continue;
        }

        // Reset flag
        search_info.reset();
        std::cout << "search times:" << runtime_counter << std::endl;
        auto start = std::chrono::system_clock::now();
        // Execute astar search
        bool result = astar.makePlan(search_info.getStartPose().pose, search_info.getGoalPose().pose,
                                     search_info.getMap());

        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;



        //std::cout << "astar msec: " << usec / 1000.0 << std::endl;
        ROS_INFO("astar msec: %lf", msec);

        if (result) {
            if (runtime_counter < 101) {
                astar_runtime << runtime_counter << msec << hmpl::endrow;
                runtime_counter++;
            }
            ROS_INFO("Found GOAL!");
            //publishPathAsWaypoints(waypoints_pub, astar.getPath(), waypoint_velocity_kmph);
            astar.samplePathByStepLength();
            path_pub.publish(astar.getDensePath());
            saveStatePath(astar.getDensePath());


#if DEBUG
            astar.publishPoseArray(debug_pose_pub, "odom");
            astar.publishFootPrint(footprint_pub_, "odom");
            astar.broadcastPathTF();
#endif

        } else {
            ROS_INFO("can't find goal...");

#if DEBUG
            astar.publishPoseArray(debug_pose_pub, "odom"); // debug
            path_pub.publish(astar.getPath());
#endif

        }

        astar.reset();

        loop_rate.sleep();
    }

    return 0;
}