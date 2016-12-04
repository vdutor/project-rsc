#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

from Queue import Queue
from Queue import Empty
from threading import Thread
from enum import Enum
import cv2
import time
import pandas as pd
import csv
import numpy as np
import matplotlib.pyplot as plt


def dist(p1, p2):
    dx = (p1[0] - p2[0])
    dy = (p1[1] - p2[1])
    return math.sqrt(dx * dx + dy * dy)


def isNaN(num):
    return num != num


class Robot(Thread):
    def __init__(self, queue_command, queue_position):
        Thread.__init__(self)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.speed = 1 # m/s
        self.rot_speed = 1 # rad/s
        self.queue_commands = queue_command
        self.queue_positions = queue_position
        self.running = True

    def stop(self):
        print "Stopping robot"
        self.running = False

    def run(self):
        while self.running:
            try:
                target = self.queue_commands.get(timeout=3)
            except Empty:
                pass
            else:
                print "going to: ", target
                travel_time = dist((self.x, self.y), (target[2], target[1])) / self.speed
                rotation_time = math.fabs(target[2] - self.theta) / self.rot_speed
                time.sleep(travel_time + rotation_time)
                self.x = target[0]
                self.y = target[1]
                self.theta = target[2]
                self.queue_positions.put(target)


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

    def __init__(self, robot, size, px_to_metre_ratio):
        self.robot = robot
        self.p2m = px_to_metre_ratio
        self.world = np.zeros((size, size))
        self.center = (size/2, size/2)
        self.world[self.center] = State.full.value
        self.process = True

    def process_laser_data(self, data):
        if not self.process:
            print "not processing"
            return
        else:
            print "processing"

        x,y,theta = self.robot.x, self.robot.y, self.robot.theta
        angles = [data.angle_min + (idx * data.angle_increment) + theta for idx in range(len(data.ranges))]
        obstacle_x = [x + (math.cos(angle) * dist) for angle,dist in zip(angles, data.ranges)]
        obstacle_y = [y + (math.sin(angle) * dist) for angle,dist in zip(angles, data.ranges)]
        points = zip(obstacle_x, obstacle_y)

        for p in points:
            if not (isNaN(p[0]) or isNaN(p[1])):
                x = self.center[0] + int(p[0] / self.p2m)
                y = self.center[1] - int(p[1] / self.p2m)
                if x >= 0 and x < self.world.shape[1] and y >= 0 and y < self.world.shape[0]:
                    self.world[y,x] = State.full.value
                else:
                    print "Out of bound"
                    # TODO: make image larger, rescale or translate image

        self.process = False

    def show(self):
        scaled_world = cv2.resize(self.world, (600, 600))
        cv2.imshow('frame', scaled_world)
        cv2.waitKey(500)



def read_csv():
    df = pd.read_csv("~/catkin_ws/laser_data.csv", delimiter=";", header=0)
    return df


def store_measurements(data):
    with open("test.csv", "a") as f:
        fileWriter = csv.writer(f, delimiter=';')
        fileWriter.writerow(data.ranges)


def init():
    rospy.init_node('laser node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, process_laser)
    # rospy.Subscriber("scan", LaserScan, store_measurements)
    rospy.spin()

def laser_driver_thread(arg):
    ctr = 0
    try:
        while ctr < 40: #len(df): TODO change this back
            print "processing line: ", ctr
            data = LaserData(df.loc[ctr])
            world.process_laser_data(data)
            time.sleep(0.5)
            ctr += 1
    except KeyboardInterrupt:
        pass

q_commands = Queue()
q_positions = Queue()
robot = Robot(q_commands, q_positions)
world = World(robot, 1000, .02)

if __name__ == '__main__':
    # init()
    df = read_csv()
    thread = Thread(target = laser_driver_thread, args = (df, ))
    thread.start()
    robot.start()

    target = (0,0,0)
    try:
        while True:
            q_commands.put(target)
            while target != (robot.x, robot.y, robot.theta):
                time.sleep(1)
            print "arrived at position"
            target = (robot.x + 0.5, 0, robot.theta + 0.2)
            world.process = True
            world.show()
    except KeyboardInterrupt:
        robot.stop()
        thread.join()
