#! /usr/bin/env python
#HALF FINSISHED
import rospy
import actionlib
import gps_goal_msgs.msg
import math
import tf
from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

def get_origin_lat_long():
  # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
  rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
  origin_pose = rospy.wait_for_message('local_xy_origin', PoseStamped)
  origin_lat = origin_pose.pose.position.y
  origin_long = origin_pose.pose.position.x
  rospy.loginfo('Received origin: lat %s, long %s.' % (origin_lat, origin_long))
  return origin_lat, origin_long

def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points
  hypotenuse = distance = g['s12'] # access distance
  rospy.loginfo("The distance from the origin to the goal is {:.3f} m.".format(distance))
  azimuth = g['azi1']
  rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse
  rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

  return x, y

class ExecuteWaypointsGPSAction(object):
  # create messages that are used to publish feedback/result
  _feedback = gps_goal_msgs.msg.ExecuteWaypointsGPSFeedback()
  _result = gps_goal_msgs.msg.ExecuteWaypointsGPSResult()

  def __init__(self, name):
    self.wall_found = False
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, gps_goal_msgs.msg.ExecuteWaypointsGPSAction, execute_cb=self.execute_cb, auto_start = False)
    self.wallCorner = rospy.Subscriber('corner_location', Point, self.stop_moving)
    self._as.start()
    rospy.loginfo("Connecting to move_base...")
    self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.move_base.wait_for_server()
    rospy.loginfo("Connected.")
    self.origin_lat, self.origin_long = get_origin_lat_long()

  def stop_moving(self, _):
    self.move_base.cancel_goal()
    self.wall_found = True
    self.wallCorner.unregister()

  def execute_cb(self, goals):
    # helper variables
    r = rospy.Rate(1)
    success = True
    waypointreached = 1
    rospy.loginfo('the number of waypoints: %f', len(goals.waypoints))
    # start executing the action
    for i in goals.waypoints:
      # publish info to the console for the user
      rospy.loginfo(' Executing waypoints starting with, %f, %f', i.latitude, i.longitude)

      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
      self.do_gps_goal(i.latitude, i.longitude)
      self._feedback.waypointReached = waypointreached
      # publish the feedback
      self._as.publish_feedback(self._feedback)
      if self.wall_found == True:
        break;

      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes

      waypointreached = waypointreached +1
      r.sleep()

    if success:
      self._result.result = self._feedback.waypointReached
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)

  def do_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
    # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location
    x, y = calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
    # Create move_base goal
    self.publish_goal(x=x, y=y, z=z, yaw=yaw, roll=roll, pitch=pitch)


  def publish_goal(self, x=0, y=0, z=0, yaw=0, roll=0, pitch=0):
    # Create move_base goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = rospy.get_param('~frame_id','map')
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z =  z
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    rospy.loginfo('Executing move_base goal to position (x,y) %s, %s, with %s degrees yaw.' %
                  (x, y, yaw))
    rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

    # Send goal
    self.move_base.send_goal(goal)
    rospy.loginfo('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
    status = self.move_base.get_goal_status_text()
    if status:
      rospy.loginfo(status)

    # Wait for goal result
    self.move_base.wait_for_result()
    rospy.loginfo('Final goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
    status = self.move_base.get_goal_status_text()
    if status:
      rospy.loginfo(status)


if __name__ == '__main__':
  rospy.init_node('gps_goals')
  server = ExecuteWaypointsGPSAction(rospy.get_name())
  rospy.spin()