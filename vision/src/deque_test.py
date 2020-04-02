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
from challenge2_dependancies.msg import CornerLocation
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64


def get_blob_relative_position(image, x_corner, y_corner):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    #print("rows: %f", rows)
    #print("columns: %f", cols)
    # print(rows, cols)
    center_x = 0.5 * cols
    center_y = 0.5 * rows
    #print("x coord pix: %f", x_corner)
    #print("y coord pix: %f", y_corner)
    x = (x_corner - center_x) / center_x # if x_corner is less than center_x then x will be negative?
    y = (y_corner - center_y) / center_y
    #print("x coord rel: %f", x)
    #print("y coord rel: %f", y)
    return x, y


class wall_position_publisher:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.corner_pos_pub = rospy.Publisher("corner_location", CornerLocation, queue_size=10)
        self.bridge = CvBridge()
        self.corner_location_message = CornerLocation()
        self.image_sub = rospy.Subscriber("/husky/camera1/image_raw", Image, self.callback)
        self.pts = deque(maxlen=30)

    def corner_coordinates(self, image):
        # this funcion will extract coorinates from image
        return 1, 1

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
        high_yellow = np.array([48, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)

        # Combine masks
        yellow_magenta_mask = cv2.bitwise_or(magenta_mask, yellow_mask)  # This one is all white

        # Apply Median Blur
        median_ym = cv2.medianBlur(yellow_magenta_mask, 25)  # all white

        # Edge Detection
        edges_ym_median = cv2.Canny(median_ym, 100, 200)

        # Line detection
        rho = 2  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 30  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 50  # minimum number of pixels making up a line
        max_line_gap = 10  # maximum gap in pixels between connectable line segments
        line_image = np.copy(edges_ym_median) * 0  # creating a blank to draw lines on
        lines = cv2.HoughLinesP(edges_ym_median, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

        if lines is not None:
            for line in lines:  # try and remove this for loop
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 1)

            lines_edges = cv2.addWeighted(edges_ym_median, 0.8, line_image, 1, 0)

            # Corner Detection
            edges_ym_median_corners = cv2.cornerHarris(line_image, 20, 11, 0.1)

            # result is dilated for marking the corners, not important
            edges_ym_median_corners = cv2.dilate(edges_ym_median_corners, None)

            ret, edges_ym_median_corners = cv2.threshold(edges_ym_median_corners, 0.01 * edges_ym_median_corners.max(),
                                                         255,
                                                         0)
            edges_ym_median_corners = np.uint8(edges_ym_median_corners)
            harris_corners = np.zeros_like(lines_edges)
            harris_corners[edges_ym_median_corners > 0.01 * edges_ym_median_corners.max()] = 255

            # find centroids
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(edges_ym_median_corners)

            # define the criteria to stop and refine the corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

            corners = cv2.cornerSubPix(edges_ym_median_corners, np.float32(centroids), (5, 5), (-1, -1), criteria)
            #xmax = corners[:,0].max()  #another wat maybe quicker
            #print(corners)
            xy_max = np.amax(corners, axis=0)
            index = np.where(corners == xy_max[0])
            xy = corners[index[0]]
            #print(xy.shape)
            #print(type(xy))
            x = xy[0][0]
            y = xy[0][1]
            print((x, y))
            coord = get_blob_relative_position(frame, x, y)
            print(coord)
            self.corner_location_message.x = coord[0]
            self.corner_location_message.y = coord[1]
            self.pts.appendleft((x,y))
            self.corner_pos_pub.publish(self.corner_location_message)
            # Now draw them
            res = np.hstack((centroids, corners))
            res = np.int0(res)
            # print(res) Prints both Harris corner centroids and cornersubpix values
            frame[res[:, 1], res[:, 0]] = [0, 0, 255]
            frame[res[:, 3], res[:, 2]] = [0, 255, 0]
        for i in range(1, len(self.pts)):
            # if either of the tracked points are None, ignore
            # them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            thickness = int(np.sqrt(30 / float(i + 1)) * 2.5) # what is this
            cv2.line(frame, self.pts[i - 1], self.pts[i], (50, 50, 255), thickness) # values of x < x centre will be less than 0, need to convert back before plotting
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
