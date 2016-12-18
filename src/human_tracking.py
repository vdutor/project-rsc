#!/usr/bin/env python2

import rospy
import roslib
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from imutils.object_detection import non_max_suppression
import numpy as np
import imutils
import cv2

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image,self.callback)
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def process(self, image):
        image = imutils.resize(image, width=min(400, image.shape[1]))
        orig = image.copy()

        # detect people in the image
        (rects, weights) = self.hog.detectMultiScale(image, winStride=(4, 4),
                                                padding=(8, 8), scale=1.05)

        # draw the original bounding boxes
        for (x, y, w, h) in rects:
            cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

        # show the output images
        # cv2.imshow("Before NMS", orig)
        cv2.imshow("After NMS", image)
        cv2.waitKey(1)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.process(cv_image)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

# initialize the HOG descriptor/person detector
# hog = cv2.HOGDescriptor()
# hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# camera_port = 0
# cap = cv2.VideoCapture(0)

# while True:
    # ret, image = cap.read()
    # image = imutils.resize(image, width=min(400, image.shape[1]))
    # orig = image.copy()

    # # detect people in the image
    # (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
                                            # padding=(8, 8), scale=1.05)

    # # draw the original bounding boxes
    # for (x, y, w, h) in rects:
        # cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # # apply non-maxima suppression to the bounding boxes using a
    # # fairly large overlap threshold to try to maintain overlapping
    # # boxes that are still people
    # rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    # pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

    # # draw the final bounding boxes
    # for (xA, yA, xB, yB) in pick:
        # cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

    # # show the output images
    # # cv2.imshow("Before NMS", orig)
    # cv2.imshow("After NMS", image)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
        # break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()
