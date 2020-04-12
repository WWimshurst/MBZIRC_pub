#!/usr/bin/python
"""
l
"""
import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from move_msgs.srv import AlignWall
from std_msgs.msg import Float64

PROPORTIONAL_CONSTANT = 1


brick_number_to_distance = {
    1:0.4,
    2:0.8,
    3:1.2,
    4:1.8,
}

def saturate(value, min, max):
    #positive angle
    if value > 0:
        value = (90/value)-1
    else:
        value = (90/value)+1
    return value*PROPORTIONAL_CONSTANT


class WallAligner:
    def __init__(self):
        self._time_detected = 0.0
        s = rospy.Service("align_wall", AlignWall, self.handle_align_wall)
        s2 = rospy.Service("move_brick_length", AlignWall, self.handle_move_length)

        self.sub_center = rospy.Subscriber("wall_angle", Float64, self.update_angle)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        self._message = Twist()
        self.wall_angle = 100
	self.aligned_count = 0

        rospy.spin()

    def handle_align_wall(self, req):
        # --- Set the control rate
        rate = rospy.Rate(20)
        aligned_count = 0
        while not rospy.is_shutdown():

            #if 3 > self.wall_angle > -3:
            #    break
            if self.time_undetected > 10:
                self._message.angular.z = 0.2;
                self.pub_twist.publish(self._message)
            else:
                steer_action = self.get_control_action()
                rospy.loginfo("Steering = %3.1f" % steer_action)
                # -- update the message
                self._message.angular.z = -steer_action
                # -- publish it
                self.pub_twist.publish(self._message)
                if steer_action == 0.0:
		    self.aligned_count = self.aligned_count + 1
		    if self.aligned_count > 4:
                    	break;
		else:
		    self.aligned_count = 0

        rate.sleep()
        return 1

    def handle_move_length(self, req):
        # --- Set the control rate
        distance =  brick_number_to_distance[req.req];
        speed = 0.2;
        time_to_move = distance/speed
        time_begins = time.time()
        rate = rospy.Rate(20)
        aligned_count = 0
        while not rospy.is_shutdown() and (time.time() - time_begins) < time_to_move:
            steer_action = self.get_control_action()
            rospy.loginfo("Steering = %3.1f" % steer_action)
            # -- update the message
            self._message.angular.z = -steer_action
            self._message.linear.x = -speed
            # -- publish it
            self.pub_twist.publish(self._message)

            if(self._message == 0):
                break;

        rate.sleep()
        return 1

    @property
    def time_undetected(self):
        return time.time() - self._time_detected

    @property
    def is_detected(self):
        return time.time() - self._time_detected < 1.0

    def update_angle(self, message):
        self.wall_angle = message.data
        self._time_detected = time.time()
        #rospy.loginfo("wall angle detected: %f", self.wall_angle)

    def get_control_action(self):
        steer_action = 0.0
        rospy.loginfo("I get control angle")
        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            steer_action = saturate(self.wall_angle, -0.4, 0.4)
            rospy.loginfo("Steering command %f" % steer_action)
        return steer_action


if __name__ == "__main__":
    rospy.init_node('align_wall')
    wall_align = WallAligner()
