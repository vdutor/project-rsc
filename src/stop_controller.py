#!/usr/bin/env python2
from skimage.filter import threshold_otsu
from skimage.transform import resize
from matplotlib import pyplot as plt
from skimage.morphology import closing, square
from skimage.measure import regionprops
from skimage import restoration
from skimage import measure
from skimage.color import label2rgb
import matplotlib.patches as mpatches

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


class Letter:

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.center = ((x2 + x1) / 2, (y2 + y1) / 2 )
        self.area = (y2 - y1) * (x2 - x1)

    def __str__(self):
        return "{}, {}, {}, {}, area: {}, center: {}, {}" \
                .format(self.x1, self.y1, self.x2, self.y2, self.area, self.center[0], self.center[1])

    def dist(self, letter2):
        dx = self.center[0] - letter2.center[0]
        dy = self.center[1] - letter2.center[1]
        return (dx * dx + dy * dy) ** (.5)


class StopDetector:

    def __init__(self, estimated_rtt):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.image_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.objectFound = False
        self.getOdom = False
        self.publisher = rospy.Publisher("stop", String, queue_size=10)
        self.estimated_rtt = estimated_rtt

        self.history_length = 10
        self.history_sign = [(False , False)] * self.history_length
        self.history_it = 0
        self.history_analysis_lenth = 10
        self.counter = 0


    def print_sign_history(self):
        for entry in reversed(self.history_sign):
            print entry


    def process(self, img):
        h = img.shape[0]
        img = img[:h/2, :]
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        letters = self.find_letters(gray_image, False)
        filtered = self.filter_letters(letters)

        sign_detected = self.make_descision(filtered)
        in_center = False
        if sign_detected and self.in_center(filtered):
            in_center = True

        print "Is this the stop sign? ", sign_detected
        print "Is the stop sign in the center ", in_center

        self.history_sign[self.history_it] = (sign_detected, in_center)
        self.history_it += 1
        if self.history_it >= self.history_length:
            self.history_it = 0

        # print "hist: "
        # self.print_sign_history()

        sum_sign_detected = 0
        sum_sign_not_detected = 0
        for entry in self.history_sign[:self.history_analysis_lenth]:
            if entry[0]:
                sum_sign_detected += 1

        for entry in self.history_sign[-self.history_analysis_lenth:]:
            if not entry[0]:
                sum_sign_not_detected += 1

        # print "sum_detected ", sum_sign_detected
        # print "sum_sign_not_detected ", sum_sign_not_detected

        # this line means:
            # at the end of the history there should be too much detections
            # at the beginning of the history there should be a lot of detection
            # at the beginning of the history there should be a detection in the center
        # if sum_sign_not_detected / self.history_analysis_lenth >= 0.8 \
           # and sum_sign_detected / self.history_analysis_lenth >= 0.8 \
           # and any([entry[1] for entry in self.history_sign[:self.history_analysis_lenth]]):
            # print "STOP"
            # self.detected_sign()
        if in_center and sign_detected:
            print "::: STOP"


    def find_letters(self, image, show=True):
        image = restoration.denoise_tv_chambolle(image, weight=0.1)
        thresh = threshold_otsu(image)
        bw = closing(image > thresh, square(2))
        cleared = bw.copy()

        label_image = measure.label(cleared)
        borders = np.logical_xor(bw, cleared)

        label_image[borders] = -1
        image_label_overlay = label2rgb(label_image, image=image)

        fig, ax = plt.subplots(ncols=1, nrows=1, figsize=(12, 12))
        ax.imshow(image_label_overlay)

        result = []
        for region in regionprops(label_image):
            minr, minc, maxr, maxc = region.bbox
            result.append(Letter(minc, minr, maxc, maxr))
            rect = mpatches.Rectangle((minc, minr), maxc - minc, maxr - minr, fill=False, edgecolor='red', linewidth=2)
            ax.add_patch(rect)

        if show:
            plt.show()

        return result

    def filter_letters(self, letters):
        filtered_list = []
        for l in letters:
            if l.area < 500:
                continue
            if l.area > 3600:
                continue

            filtered_list.append(l)

        return filtered_list

    def make_descision(self, letters):

        if len(letters) != 4:
            return False

        for i in range(len(letters) - 1):
            distanceCenters = letters[i].dist(letters[i+1])
            if distanceCenters > 60:
                print "letters are too far apart"
                return False
        return True


    def in_center(self, letters):
        letter_o = sorted(letters, key= lambda x: x.center[0])[2]

        if abs(letter_o.x1 - 300) < 100:
            return True

        return False


    def detected_sign(self):
        if not self.objectFound:
            self.objectFound = True
            self.getOdom = True
            self.detectionTime = time.time()
            self.publisher.publish("final destination")
        # elif time.time() - self.detectionTime > self.estimated_rtt:
            # # broadcast at final destination
            # self.detectionTime = time.time()
            # self.getOdom = True


    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        if self.counter == 0:
            print "processing"
            self.process(cv_image)
        self.counter = (self.counter + 1) % 30

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
