#!/usr/bin/python
"""

"""
import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from move_msgs.srv import FindWall
from std_msgs.msg import Bool

Proportional_constant = 1.0
Target_offset_constant = 0.4
MAXMIMUM_LINEAR_SPEED = 1.0

def saturate(value, min, max):
    if value <= min:
        return min
    elif value >= max:
        return max
    else:
        return value


def get_throttle(wall_y):
    # top half of the screen
    if wall_y < 0:
        return MAXMIMUM_LINEAR_SPEED
    else:
        speed =  1/(wall_y*10)
        if speed > MAXMIMUM_LINEAR_SPEED:
            return MAXMIMUM_LINEAR_SPEED
        return speed


class WallChaser:
    def __init__(self):
        self._time_detected = 0.0
        s = rospy.Service("chase_wall", FindWall, self.handle_find_wall)
        self.sub_center = rospy.Subscriber("corner_location", Point, self.update_wall)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")

        self._message = Twist()
        self._time_steer = 0
        self._steer_sign_prev = 0
        rospy.spin()


    # this is the service call back, will only run when the service is called
    def handle_find_wall(self, req):
        # --- Set the control rate
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # if the wall is out of sight spin and find
            if self.time_undetected > 10:
                self._message.linear.x = 0;
                self._message.angular.z = 0.2;
                self.pub_twist.publish(self._message)
            # if the wall is in sight drive towards
            else:
                steer_action, throttle_action = self.get_control_action()
                rospy.loginfo("Steering = %3.1f" % (steer_action))
                # -- update the message
                self._message.linear.x = throttle_action
                self._message.angular.z = -steer_action
                print(steer_action)
                # -- publish it
                self.pub_twist.publish(self._message)
            # if it has arrived at the destination (0.8 implies the corner is low in its field of vision)
            #if self.wall_y > 0.7 and self.time_undetected > 4:
            if self.wall_y > 0.8:
                print("wall reached")
                break
            rate.sleep()
        return 1
    @property
    def time_undetected(self):
        return time.time() - self._time_detected

    @property
    def is_detected(self):
        return time.time() - self._time_detected < 1.0

    # this is the subscriber callback this will be called everytime a corner is visible
    def update_wall(self, message):
        self.wall_x = message.x
        self.wall_y = message.y
        self._time_detected = time.time()
        print(message.x)
        rospy.loginfo("wall corner detected: %.1f  %.1f " % (self.wall_x, self.wall_y))

    #this determines the direction  and speed of steering
    def get_control_action(self):
        steer_action = 0.0
        throttle_action = 0.0

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object, with offset to keep wall on one side
            steer_action = (Proportional_constant * self.wall_x) + Target_offset_constant
            # --- this is to limit the turning speed, too fast will likely cause oscilation
            steer_action = saturate(steer_action, -0.4, 0.4)
            rospy.loginfo("Steering command %.2f" % steer_action)
            # --- this is set arbitrarily low, the slower the less likely for oscillation
            throttle_action = get_throttle(self.wall_y)

        return (steer_action, throttle_action)


if __name__ == "__main__":
    rospy.init_node('chase_wall')
    find_wall = WallChaser()