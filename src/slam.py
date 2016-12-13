#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid



def print_map_meta_data(msg):
    print "Map meta data: "
    print msg


def print_occupancy_grid(grid):
    print "Grid: "
    print grid
    print grid.info
    print grid.data


def init():
    rospy.init_node('slam node', anonymous=True)
    rospy.Subscriber("MapMetaData", MapMetaData, print_map_meta_data)
    rospy.Subscriber("Grid", OccupancyGrid, print_occupancy_grid)
    rospy.spin()


if __name__ == '__main__':
    init()
    rospy.spin()
