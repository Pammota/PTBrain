import math
import time

import cv2

from imu import imu
from Map import Map
from threading import Thread

class IMU_tracking(Thread):

    def __init__(self):
        self.x = 0
        self.y = 53

        self.v = 0

        self.a_x = 0
        self.a_y = 0

        self.theta = 0
        self.inner_map = Map()
        self.imu = imu()

    def run(self):

        dt = 0
        while True:

            a_x, a_y, theta = self.imu.get_data()

            # get projections on axis
            a_x_x = self.a_x * math.cos(self.theta)
            a_x_y = self.a_x * math.sin(self.theta)

            a_y_x = self.a_y * math.sin(self.theta + math.pi)
            a_y_y = self.a_y * math.cos(self.theta + math.pi)

            vx = self.v * math.cos(theta)
            vy = self.v * math.sin(theta)

            # update coordinates
            self.x = self.x + vx * dt + (a_x_x + a_y_x) * (dt ** 2) / 2
            self.y = self.y + vy * dt + (a_x_y + a_y_y) * (dt ** 2) / 2

            # update speed
            vx = vx + (a_x_x + a_y_x) * dt
            vy = vy + (a_x_y + a_y_y) * dt
            self.v = math.sqrt(vx ** 2, vy ** 2)

            self.inner_map.update_map(self.x, self.y, self.theta)
            image = self.inner_map.get_map()
            cv2.imshow("map", image)

            time.sleep(0.01)
            dt = 0.01