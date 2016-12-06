import cv2
from utils import is_nan
from enum import Enum
import numpy as np
import math

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
            if not (is_nan(p[0]) or is_nan(p[1])):
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
