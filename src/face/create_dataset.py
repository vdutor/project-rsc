#!/usr/bin/env python2

import rospy
import roslib
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import imutils
import cv2


class DatasetCreater:

    def __init__(self, size, resource_path, name, folder, extension):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image,self.callback)
        path = self._append_path(resource_path)
        folder = self._append_path(folder)
        self.faceCascade = cv2.CascadeClassifier(path + "/haarcascade_frontalface_default.xml")
        self.eyeCascade = cv2.CascadeClassifier(path + "/haarcascade_eye.xml")
        self.size = size
        self.name = name
        self.folder = folder
        self.ext = extension
        self.ctr = 1
        self.store_location = path + folder + name

    def process(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = self.faceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        print("Found {0} faces!".format(len(faces)))

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            roi_gray = gray[y:y+h, x:x+w]
            eyes = self.eyeCascade.detectMultiScale(roi_gray)
            if (len(eyes) >= 2):
                # cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.imshow("face", roi_gray)
                c = cv2.waitKey(0)
                if ' ' == chr(c & 255):
                    roi_color = image[y:y+h, x:x+w]
                    roi_color_resized = imutils.resize(roi_color, width=self.size, height=self.size)
                    _file = self.store_location + str(self.ctr) + self.ext
                    print _file
                    cv2.imwrite(_file, roi_color_resized);
                    self.ctr += 1
                cv2.destroyAllWindows()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.process(cv_image)

    def _append_path(self, path):
        return path if path.endswith("/") else path + "/"


def main(args):
    creator = DatasetCreater(150, args[1], "vincent", "dataset2", ".png")

    rospy.init_node('create_dataset', anonymous=True)

    # start looping
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
