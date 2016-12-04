#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

from enum import Enum
import cv2
import time
import pandas as pd
import csv
import numpy as np
import matplotlib.pyplot as plt

x = 0
y = 0

class LaserData:
    angle_min = -2.35619
    angle_max = 2.09235
    angle_increment = 0.00613592
    scan_time = 0.1
    range_min = 0.02
    range_max = 5.6
    ranges_size = 726
    time_inc = 9.76563e-5
    def __init__(self, ranges):
        self.ranges = ranges

class State(Enum):
    full = 255
    empty = 0

class World:

    def __init__(self, size):
        self.world = np.zeros((size, size))
        self.center = (size/2, size/2)
        self.world[self.center] = State.full.value

    def process_laser_data(self, data):
        x,y = get_robot_position()
        angles = [data.angle_min + idx * data.angle_increment for idx in range(len(data.ranges))]
        obstacle_x = [x + (math.cos(angle) * dist) for angle,dist in zip(angles, data.ranges)]
        obstacle_y = [y + (math.sin(angle) * dist) for angle,dist in zip(angles, data.ranges)]
        # points =
        # plt.ion()
        # plt.scatter(*zip(*zip(obstacle_x, obstacle_y)))
        # plt.pause(0.5)

    def show(self):
        scaled_world = cv2.resize(self.world, (600, 600))
        cv2.imshow('frame', scaled_world)
        cv2.waitKey(0)

    def _add_data(self, points):
        print 'e'


def read_csv():
    df = pd.read_csv("~/catkin_ws/laser_data.csv", delimiter=";", header=0)
    return df


def get_robot_position():
    global x,y
    x += 0.05
    y += 0.05
    return x,y

def store_measurements(data):
    with open("test.csv", "a") as f:
        fileWriter = csv.writer(f, delimiter=';')
        fileWriter.writerow(data.ranges)


def init():
    rospy.init_node('laser node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, process_laser)
    # rospy.Subscriber("scan", LaserScan, store_measurements)
    rospy.spin()

if __name__ == '__main__':
    # init()
    w = world()
    df = read_csv()
    ctr = 0
    while ctr < 1: #len(df): TODO change this back
        data = LaserData(df.loc[ctr])
        print "processing line: ", ctr
        w.process_laser_data(data)
        w.show()
        # time.sleep(3)
        ctr += 1
