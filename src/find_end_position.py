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
        self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.image_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.redLower1 = (0, 102, 66)
        self.redUpper1 = (10, 240, 210)
        self.redLower2 = (165, 102, 66)
        self.redUpper2 = (180, 240, 210)
        self.objectFound = False
        self.getOdom = False
        self.publisher = rospy.Publisher("stop", String, queue_size=10)
        self.estimated_rtt = estimated_rtt

    def process(self, image):
        image = imutils.resize(image, width=600)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.redLower1, self.redUpper1)
        mask1 = cv2.erode(mask1, (20,20), iterations=2)
        mask1 = cv2.dilate(mask1, (20,20), iterations=2)

        mask2 = cv2.inRange(hsv, self.redLower2, self.redUpper2)
        mask2 = cv2.erode(mask2, (20,20), iterations=2)
        mask2 = cv2.dilate(mask2, (20,20), iterations=2)

        mask = mask1 | mask2
        mask = cv2.dilate(mask, (20,20), iterations=2)
        mask = cv2.erode(mask, (20,20), iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 100:
                self.detected_object()
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)

        cv2.imshow("image", image)
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
