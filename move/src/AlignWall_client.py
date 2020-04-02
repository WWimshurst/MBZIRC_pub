#!/usr/bin/env python

import sys
import rospy
from move_msgs.srv import *

def align_wall_client(x):
    rospy.wait_for_service('align_wall')
    try:
        align_wall = rospy.ServiceProxy('align_wall', AlignWall)
        resp1 = align_wall(x)
        print(resp1.status)
        return resp1.status
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)


if __name__ == "__main__":
    align_wall_client(1)