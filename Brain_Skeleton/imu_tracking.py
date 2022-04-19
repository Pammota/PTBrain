import math
import time

import cv2

from imu import imu
from Map import Map
from threading import Thread

class IMU_tracking(Thread):

    def __init__(self, brain):
        super(IMU_tracking, self).__init__()

        self.brain = brain

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

            speed = self.brain.get_crt_speed()

            self.a_x, self.a_y, yaw = self.imu.get_data()
            self.theta = math.radians(yaw)

            self.a_x *= 9.87 # m / s ^ 2
            self.a_y *= 9.87 # m / s ^ 2

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

            print("vx = {0:.5f} + ({1:.5f} + {2:.5f}) * 0.03".format(vx, a_x_x, a_y_x))
            print("vy = {0:.5f} + ({1:.5f} + {2:.5f}) * 0.03".format(vy, a_x_y, a_y_y))
            print("v = {0:.5f}".format(self.v))

            self.v = math.sqrt(vx ** 2 + vy ** 2)
            print("v' = {0:.5f}\n".format(self.v))

            #print("accelx = {}, accely = {}".format(self.a_x, self.a_y))
            #print("x = {}, y = {}".format(self.x, self.y))

            #print("v = {0:.5f}, a = {1:.5f}".format(self.v, math.sqrt(self.a_x**2 + self.a_y**2)))

            self.inner_map.update_map(self.x, self.y, yaw)
            image = self.inner_map.get_map()
            cv2.imshow("map", image)
            cv2.waitKey(1)

            time.sleep(0.03)
            dt = 0.03  # s