#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

from Queue import Queue, Empty
from threading import Thread
import time

from world import World, State, LaserData
from robot import Robot
from utils import read_csv


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
            target = (0, 0, robot.theta + 0.2)
            world.process = True
            world.show()
    except KeyboardInterrupt:
        robot.stop()
        thread.join()
