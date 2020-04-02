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

def get_blob_relative_position(image, x_corner, y_corner):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    center_x = 0.5 * cols
    center_y = 0.5 * rows
    x = (x_corner - center_x) / center_x
    y = (y_corner - center_y) / center_y
    return x, y

class wall_position_publisher:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_corner", Image, queue_size=10)
        self.corner_pos_pub = rospy.Publisher("corner_location", Point, queue_size=10)
        self.bridge = CvBridge()
        self.corner_location_message = Point()
        self.image_sub = rospy.Subscriber("/husky/camera1/image_raw", Image, self.callback)


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
        low_magenta = np.array([120, 90, 75])
        high_magenta = np.array([170, 255, 255])
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
        (_, contours, _) = cv2.findContours(ym_median, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_image = np.zeros_like(ym_median)
        contour_image = cv2.drawContours(contour_image, contours, -1, (255,255,255), 1)
        contour_frame = np.copy(frame)
        contour_frame = cv2.drawContours(contour_frame, contours, -1, (255,255,255), 4)
        largest_contour_image = np.zeros_like(ym_median)
        line_image = np.copy(contour_image)
        harris_corners = np.zeros_like(ym_median)
        if len(contours) != 0:
            largest_contour = max(contours, key = cv2.contourArea)
            largest_contour_image = cv2.drawContours(largest_contour_image, [largest_contour], -1, (255,255,255), 1)

            #corner detection
            ym_corners = np.zeros_like(ym_median)
            ym_corners = np.float32(ym_corners)
            ym_corners = cv2.cornerHarris(largest_contour_image,20,11,0.1)

            #result is dilated for marking the corners, not important
            ym_corners = cv2.dilate(ym_corners,None)

            ret, ym_corners = cv2.threshold(ym_corners,0.01*ym_corners.max(),255,0)
            ym_corners = np.uint8(ym_corners)
            harris_corners[ym_corners>0.01*ym_corners.max()] = 255

            #find centroids
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(ym_corners)

            # define the criteria to stop and refine the corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

            corners = cv2.cornerSubPix(ym_corners,np.float32(centroids),(5,5),(-1,-1),criteria)
            corners = np.delete(corners, 0, 0)
            #print(corners)

            # Find max x corner
            x_max_index = np.where(corners == np.amax(corners[:,0], axis = 0))
            xy = corners[x_max_index[0], :]

            x = xy[0][0]
            y = xy[0][1]
            all_corners = np.copy(frame)
            for corner in corners:
                cv2.circle(all_corners, (corner[0],corner[1]), 5, (0,255,0), 2)
            corner_frame = np.copy(frame)
            corner_frame = cv2.circle(corner_frame, (x,y), 5, (0,0,255), 2)
            print('xy pixel values at maximum x', (x, y))
            coord = get_blob_relative_position(frame, x, y)
            print(coord)
            self.corner_location_message.x = coord[0]
            self.corner_location_message.y = coord[1]
            self.corner_pos_pub.publish(self.corner_location_message)
            return all_corners
        return frame


def main(args):
    rospy.init_node('wall_position_publisher', anonymous=True)
    ic = wall_position_publisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
