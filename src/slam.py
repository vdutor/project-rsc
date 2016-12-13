#!/usr/bin/env python2
import rospy
import math

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid



def print_map_meta_data(msg):
    print "Map meta data: "
    print msg


def print_occupancy_grid(grid):
    print "Grid: "
    # print grid
    print grid.info
    print len(grid.data)
    # print grid.data


def init():
    rospy.init_node('slam_node', anonymous=True)
    rospy.Subscriber("map_metadata", MapMetaData, print_map_meta_data)
    rospy.Subscriber("map", OccupancyGrid, print_occupancy_grid)


if __name__ == '__main__':
    init()
    rospy.spin()
