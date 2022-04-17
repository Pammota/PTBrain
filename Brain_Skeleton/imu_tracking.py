import math
import time

import cv2

from imu import imu
from Map import Map
from threading import Thread

class IMU_tracking(Thread):

    def __init__(self):
        super(IMU_tracking, self).__init__()
        self.x = 0
        self.y = 2.87

        self.v = 0

        self.a_x = 0
        self.a_y = 0

        self.theta = 0
        self.inner_map = Map()
        self.imu = imu()
        self.imu.start()

    def run(self):

        dt = 0
        while True:

            self.a_x, self.a_y, yaw = self.imu.get_data()
            self.theta = math.radians(yaw)

            self.a_x *= 9.87 * 0.1 / 0.001 / 0.001  # cm / ms ^ 2
            self.a_y *= 9.87 * 0.1 / 0.001 / 0.001

            # get projections on axis
            a_x_x = self.a_x * math.cos(self.theta)
            a_x_y = self.a_x * math.sin(self.theta)

            a_y_x = self.a_y * math.sin(self.theta + math.pi)
            a_y_y = self.a_y * math.cos(self.theta + math.pi)

            vx = self.v * math.cos(self.theta)
            vy = self.v * math.sin(self.theta)

            # update coordinates
            self.x = self.x + vx * dt + (a_x_x + a_y_x) * (dt ** 2) / 2
            self.y = self.y + vy * dt + (a_x_y + a_y_y) * (dt ** 2) / 2

            # update speed
            vx = vx + (a_x_x + a_y_x) * dt
            vy = vy + (a_x_y + a_y_y) * dt
            self.v = math.sqrt(vx ** 2 + vy ** 2)

            #print("accelx = {}, accely = {}".format(self.a_x, self.a_y))
            #print("x = {}, y = {}".format(self.x, self.y))

            self.inner_map.update_map(self.x, self.y, yaw)
            image = self.inner_map.get_map()
            cv2.imshow("map", image)
            cv2.waitKey(1)

            time.sleep(0.01)
            dt = 10 # ms