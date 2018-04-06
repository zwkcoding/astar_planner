//
// Created by kevin on 4/5/18.
//
#include <iv_explore_msgs/PlannerControlAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_datatypes.h>

using namespace iv_explore_msgs;

geometry_msgs::Pose global_goal_pose;
bool goal_flag = false, complete_flag = false;
void goalPoseCallback (const geometry_msgs::PoseStampedConstPtr &msg) {

    ROS_INFO("Subcscribed goal pose!");
    double yaw = tf::getYaw(msg->pose.orientation);
//    for(int i = 0; i < 20; i++) {
//        ROS_INFO_STREAM("goal frame is in :" <<  msg->header.frame_id);
//        ROS_INFO("goal cell [in global_map coord]: [%f[m], %f[m], %f[degree]]", msg->pose.position.x,
//                 msg->pose.position.y, yaw * 180 / M_PI);
//    }

    global_goal_pose = msg->pose;
    goal_flag = true;
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const PlannerControlResultConstPtr& result)
{
    ROS_INFO("Astar actionclient:Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Astar actionclient Answer: ");
    std::cout << std::boolalpha << result->success_flag << '\n';
    complete_flag = true;
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Explore actionclient: Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const PlannerControlFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback from astar action server, current goal: %f, %f", feedback->current_goal.position.x,
    feedback->current_goal.position.y, tf::getYaw(feedback->current_goal.orientation) * 180 / M_PI);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_action_client");
    ros::NodeHandle n;

    iv_explore_msgs::PlannerControlGoal goal;
    iv_explore_msgs::PlannerControlResultConstPtr result_ptr;
    char ch;

    actionlib::SimpleActionClient<iv_explore_msgs::PlannerControlAction> ac("astar_move_base", true);
    ros::Subscriber goal_sub  = n.subscribe("/move_base_simple/goal", 1, goalPoseCallback);

    ROS_INFO("Waiting for astar action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, preparing sending goal.");

    ros::Rate rate(10);
    while(n.ok() && ch!='q') {
        ros::spinOnce();
        if (!goal_flag) {
            rate.sleep();
            continue;
        }
        goal_flag = false;

        goal.goals.header.frame_id = "/odom";
        goal.goals.header.stamp = ros::Time::now();
        goal.goals.poses.clear();
        goal.goals.poses.push_back(global_goal_pose);
        if(1) {
            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
            while (!complete_flag) {
                ros::spinOnce();
            }
            complete_flag = false;
        }
        if(0) {
            ac.sendGoal(goal);
            bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
            if (finished_before_timeout) {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Action finished: %s", state.toString().c_str());
            } else
                ROS_INFO("Action did not finish before the time out.");
        }
        // Get the Result of the current goal.
        result_ptr = ac.getResult();
       /* std::cin >> ch;
        if(ch=='c') {
            ac.cancelGoal();
            break;
        }*/
    }

    return 0;
}


