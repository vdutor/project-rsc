import math
import csv
import pandas as pd

def dist(p1, p2):
    dx = (p1[0] - p2[0])
    dy = (p1[1] - p2[1])
    return math.sqrt(dx * dx + dy * dy)

def is_nan(num):
    return num != num

def read_csv():
    df = pd.read_csv("~/catkin_ws/laser_data.csv", delimiter=";", header=0)
    return df

def store_measurements(data):
    with open("test.csv", "a") as f:
        fileWriter = csv.writer(f, delimiter=';')
        fileWriter.writerow(data.ranges)
