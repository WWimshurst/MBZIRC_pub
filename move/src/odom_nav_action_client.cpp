//
// Created by wesley on 08/03/2020.
//
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gps_navigation_msgs/ExecuteWaypointsAction.h>// Note: "Action" is appended
#include <geometry_msgs/Point.h>
#include <vector>
#include <cstdlib>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "execute_waypoints_client");
    actionlib::SimpleActionClient<gps_navigation_msgs::ExecuteWaypointsAction> ac("waypoints", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    gps_navigation_msgs::ExecuteWaypointsGoal goal;
    // Fill in goal here
    geometry_msgs::Point point;
    point.x = 4;
    point.y = 4;
    geometry_msgs::Point point2;
    point2.x = 3;
    point2.y = 3;
    goal.waypoints.push_back(point);
    goal.waypoints.push_back(point2);

    ac.sendGoal(goal);
    bool finished_early = ac.waitForResult(ros::Duration(200.0));
    if (finished_early)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());

    }
    else
    {
        ROS_INFO("could not finish before time out");
    }
    return 0;
}