#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('vision')
import sys
import rospy
import cv2
import imutils
from collections import deque
from std_msgs.msg import String
import numpy as np;
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64


class wall_angle_publisher:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_angle", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/husky/camera2/image_raw", Image, self.callback)
        self.angle_pub = rospy.Publisher("wall_angle", Float64, queue_size=10)
        self.wall_angle = Float64()


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        filtered_image = self.imagecorner(cv_image)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(filtered_image))
        except CvBridgeError as e:
            print(e)

    def imagecorner(self, frame):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Magenta color
        low_magenta = np.array([120, 50, 100])
        high_magenta = np.array([180, 255, 255])
        magenta_mask = cv2.inRange(hsv_frame, low_magenta, high_magenta)

        # Yellow color
        low_yellow = np.array([15, 60, 114])
        high_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)

        # Combine masks
        yellow_magenta_mask = cv2.bitwise_or(magenta_mask, yellow_mask)  # This one is all white

        # Morphological transformations
        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(yellow_magenta_mask,kernel,iterations = 2)
        erosion = cv2.erode(dilation,kernel,iterations = 2)

        # Apply Median Blur
        ym_median = cv2.medianBlur(erosion, 11)  # all white

        # Contour detection
        (_, contours,_) = cv2.findContours(ym_median, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_image = np.zeros_like(ym_median)
        contour_image = cv2.drawContours(contour_image, contours, -1, (255,255,255), 1)
        contour_frame = np.copy(frame)
        contour_frame = cv2.drawContours(contour_frame, contours, -1, (255,255,255), 1)
        largest_contour_image = np.zeros_like(ym_median)
        cropped = np.copy(largest_contour_image)
        cropped = cropped[240:440, 40:600]
        cropped_frame = frame[240:440, 40:600]
        line_contour_frame = np.copy(frame)
        if len(contours) != 0:
            largest_contour = max(contours, key = cv2.contourArea)
            largest_contour_image = cv2.drawContours(largest_contour_image, [largest_contour], -1, (255,255,255), 30)
            cropped = np.copy(largest_contour_image)
            cropped = cropped[240:440, 40:600]
            (_, cropped_contours, _) = cv2.findContours(cropped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #cropped_contours_image = cv2.drawContours(cropped_frame, cropped_contours, -1, (0,255,0), 3)
            if len(cropped_contours) != 0:
                max_cropped_contour = max(cropped_contours, key = cv2.contourArea)
                M = cv2.moments(max_cropped_contour)
                if int(M['m00']) != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cropped_frame = cv2.line(cropped_frame,(cx,0),(cx,480),(255,0,0),1)
                    cropped_frame = cv2.line(cropped_frame,(0,cy),(640,cy),(255,0,0),1)
                    # Fit line to contour
                    rows,cols = cropped.shape[:2]
                    [vx,vy,x,y] = cv2.fitLine(max_cropped_contour, cv2.DIST_L2,0,0.01,0.01)
                    if vy > 0.99999 or vy < -0.99999:
                        angle_local = 0
                    elif vx > 0.99999 or vx < -0.99999:
                        angle_local = 90
                    else:
                        angle_local = np.arctan(vx/vy)
                        angle_local = angle_local*(180/np.pi)
                    print('angle: ', angle_local, 'degrees')
                    #print('vx ', vx, 'vy ', vy, 'x ', x, 'y ', y)
                    lefty = (-x*vy/vx) + y
                    righty = ((cols-x)*vy/vx)+y
                    cropped_frame = cv2.line(cropped_frame,(cols-1,righty),(0,lefty),(0,0,255),2)
                    cropped_frame = cv2.drawContours(cropped_frame, cropped_contours, -1, (0,255,0), 1)
                    self.wall_angle = angle_local
                    self.angle_pub.publish(self.wall_angle)

        return cropped_frame


def main(args):
    rospy.init_node('wall_position_publisher', anonymous=True)
    ic = wall_angle_publisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
