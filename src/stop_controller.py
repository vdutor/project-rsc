#!/usr/bin/env python2

import rospy
import roslib
import sys
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import imutils
import cv2
import time

class StopDetector:

    def __init__(self, estimated_rtt):
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.image_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.blackLower = (0, 0, 0)
        self.blackUpper = (45, 45, 45)
        self.objectFound = False
        self.getOdom = False
        self.publisher = rospy.Publisher("stop", String, queue_size=10)
        self.estimated_rtt = estimated_rtt

    def process(self, image):
        image = imutils.resize(image, width=600)
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(image, self.blackLower, self.blackUpper)
        # mask1 = cv2.erode(mask1, (20,20), iterations=2)
        # mask1 = cv2.dilate(mask1, (20,20), iterations=2)

        # mask2 = cv2.inRange(hsv, self.redLower2, self.redUpper2)
        # mask2 = cv2.erode(mask2, (20,20), iterations=2)
        # mask2 = cv2.dilate(mask2, (20,20), iterations=2)

        # mask = mask1 | mask2
        mask = mask1
        size = 5
        it = 3
        kernel = np.ones((size,size),np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=it)
        mask = cv2.erode(mask, kernel, iterations=it)
        # mask = cv2.dilate(mask, kernel, iterations=it)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        print len(cnts)

        for cnt in cnts:
            # c = max(cnts, key=cv2.contourArea)
            # ((x, y), radius) = cv2.minEnclosingCircle(c)
            epsilon = 0.1*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            # print len(approx)
            # if len(approx) == 8:
            cv2.drawContours(mask, cnt, -1, (0, 100, 255), 3)
            area = cv2.contourArea(cnt)
            print area

                # print "found stop sign"

        cv2.imshow("image", mask)
        cv2.waitKey(1)

    def detected_object(self):
        if not self.objectFound:
            self.objectFound = True
            self.getOdom = True
            self.detectionTime = time.time()
        elif time.time() - self.detectionTime > self.estimated_rtt:
            # broadcast at final destination
            self.detectionTime = time.time()
            self.getOdom = True
            self.publisher.publish("final destination")



    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        self.process(cv_image)

    def odom_callback(self, data):
        if self.getOdom:
            print data
            self.getOdom = False

def main(args):
    stopDetector = StopDetector(10 * 2)
    rospy.init_node('detect_end_position', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

main(sys.argv)
