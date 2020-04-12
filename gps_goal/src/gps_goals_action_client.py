#! /usr/bin/env python

import rospy
import actionlib
import gps_goal_msgs.msg
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import time
from numpy import loadtxt

def get_waypoints():
  lines = loadtxt("catkin_ws/src/waypoint_test/waypoints/points_outdoor.txt", delimiter=" ")
  return lines

class gps_goal_client():
  def __init__(self):
    self.pub = rospy.Publisher('/waypointPosition', Point,  queue_size=10)
    self.pub2 = rospy.Publisher('/waypointTime',Float64, queue_size=10 )

  def send_waypoints(self):
    client = actionlib.SimpleActionClient('gps_goals', gps_goal_msgs.msg.ExecuteWaypointsGPSAction)
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()
    goals = []
    for line in get_waypoints():
      goal= NavSatFix()
      goal.header.frame_id= "/navsat/fix"
      goal.header.stamp = rospy.Time.now()
      goal.latitude = line[0]
      goal.longitude = line[1]
      goals.append(goal)
    goals_message = gps_goal_msgs.msg.ExecuteWaypointsGPSGoal(goals)
    self.starting_time = time.time()
    client.send_goal(goals_message,
                     active_cb=self.callback_active,
                     feedback_cb=self.callback_feedback,
                     done_cb=self.callback_done)

    rospy.loginfo("Goal has been sent to the action server.")
    rospy.spin()

  def callback_active(self):
    rospy.loginfo("Action server is processing the goal")

  def callback_done(self, state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

  def callback_feedback(self, feedback):
    rospy.loginfo("Feedback: I have just reached waypoint: something")
    rospy.loginfo("Feedback: I have just reached waypoint:  %d",feedback.waypointReached)
    rospy.wait_for_service('/gazebo/get_model_state')




if __name__ == '__main__':
  try:
    rospy.init_node('gps_goals_client')
    gps_goal_client = gps_goal_client()
    gps_goal_client.send_waypoints()
  except rospy.ROSInterruptException:
    rospy.loginfo("program interrupted before completion")
