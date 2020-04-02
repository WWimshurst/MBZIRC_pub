//
// Created by wesley on 08/02/2020.
//
#include <ros/ros.h>
#include "gps_navigation_msgs/ExecuteWaypointsAction.h"
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Point.h>
#include <cstdlib>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ExecuteWaypointsAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<gps_navigation_msgs::ExecuteWaypointsAction> as_;
    std::string action_name_;
    gps_navigation_msgs::ExecuteWaypointsFeedback feedback_;
    gps_navigation_msgs::ExecuteWaypointsResult result_;
public:
    ExecuteWaypointsAction(std::string name)  :
            as_(nh_,name,boost::bind(&ExecuteWaypointsAction::executeCB, this, _1),false),
            action_name_(name)
    {
        as_.start();
    }

    ~ExecuteWaypointsAction(void)
    {

    }
    void executeCB(const gps_navigation_msgs::ExecuteWaypointsGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        bool success = true;
        ROS_INFO("I am executing the callback for you now boss");

        MoveBaseClient move_base_client("move_base", true);
        //wait for the action server to come up
        while(!move_base_client.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up boss");
        }
        feedback_.waypointReached = 0;
        as_.publishFeedback(feedback_);
        int num_waypoints = (goal->waypoints).size();
        for(int i = 0; i < num_waypoints; i++)
        {
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }

            float x = (goal->waypoints)[i].x;
            float y = (goal->waypoints)[i].y;
            ROS_INFO("waypoint 1 x is: %f", x);
            ROS_INFO("waypoint 1 y is: %f", y);

            move_base_msgs::MoveBaseGoal goal;

            //we'll send a goal to the robot to move 1 meter forward
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.x = y;
            goal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Sending goal");
            move_base_client.sendGoal(goal);

            move_base_client.waitForResult();

            if(move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray, the base moved 1 meter forward");
            else
            {
                success = false;
                ROS_INFO("failed to reach point");
                break;
            }

            feedback_.waypointReached = i+1;
            as_.publishFeedback(feedback_);
            r.sleep();
        }

        if(success)
        {
            ROS_INFO("Total success, you have have arrived");
            result_.result = 1;
            as_.setSucceeded(result_);
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoints");
    ExecuteWaypointsAction move("waypoints");
    ros::spin();

    return 0;
}