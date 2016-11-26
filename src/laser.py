#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

import csv
import numpy as np
import matplotlib.pyplot as plt

x = 0
y = 0

def get_robot_position():
    global x,y
    x += 0.05
    y += 0.05
    return x,y

def store_measurements(data):
    with open("test.csv", "a") as f:
        fileWriter = csv.writer(f, delimiter=';')
        fileWriter.writerow(data.ranges)


def process_laser(data):
    x,y = get_robot_position()
    angles = [data.angle_min + idx * data.angle_increment for idx in range(len(data.ranges))]
    obstacle_x = [x + (math.cos(angle) * dist) for angle,dist in zip(angles, data.ranges)]
    obstacle_y = [y + (math.sin(angle) * dist) for angle,dist in zip(angles, data.ranges)]

    plt.scatter(obstacle_x, obstacle_y)
    plt.show()


def init():
    rospy.init_node('laser node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, process_laser)
    # rospy.Subscriber("scan", LaserScan, store_measurements)
    rospy.spin()

if __name__ == '__main__':
    init()

