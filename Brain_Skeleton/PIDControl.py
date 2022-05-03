import time

import cv2
import numpy as np

class PIDControl():
    def __init__(self, threshold, kp=0.31, ki=0, kd=0):

        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0
        self.prev_err = 0
        self.timestamp = time.time()
        self.graph = np.ones((600, 1100, 3), dtype="uint8") * 255
        self.dtsum = 0

        cv2.line(self.graph, (50, 50), (50, 550), (0, 0, 0), 1)
        cv2.line(self.graph, (50, 550), (1050, 550), (0, 0, 0), 1)
        cv2.line(self.graph, (50, 300), (1050, 300), (0, 255, 0), 2)

    def update(self, measurement):
        new_timestamp = time.time()
        dt = new_timestamp - self.timestamp

        err = measurement - self.threshold
        self.i += dt*err
        d = (err - self.prev_err)/dt

        self.draw_graph(dt, err)
        self.dtsum += dt
        self.prev_err = err
        self.timestamp = new_timestamp

        print(err, self.i, d)
        return self.kp * err + self.ki * self.i + self.kd * d

    def draw_graph(self, dt, err):

        x1, x2 = int(50 + self.dtsum * 40), int(50 + (self.dtsum + dt) * 40)
        y1, y2 = int(50 + (self.threshold + self.prev_err)*6.25), int(50 + (self.threshold + err)*6.25)

        x1, x2 = np.clip(x1, 50, 1050), np.clip(x2, 50, 1050)
        y1, y2 = np.clip(600 - y1, 50, 550), np.clip(600 - y2, 50, 550)

        cv2.line(self.graph, (x1, y1), (x2, y2), (0, 0, 255), 2)

        cv2.imshow("error_graph", self.graph)
        cv2.waitKey(1)