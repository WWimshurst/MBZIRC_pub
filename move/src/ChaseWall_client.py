#!/usr/bin/env python

import sys
import rospy
from move_msgs.srv import *

def chase_wall_client(x):
    rospy.wait_for_service('chase_wall')
    try:
        find_wall = rospy.ServiceProxy('chase_wall', FindWall)
        resp1 = find_wall(x)
        print(resp1.status)
        return resp1.status
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)


if __name__ == "__main__":
    chase_wall_client(1)