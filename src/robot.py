from Queue import Queue
from Queue import Empty
from threading import Thread
from utils import dist
import time
import math

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
