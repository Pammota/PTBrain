import time

import numpy as np

class PIDControl():
    def __init__(self, threshold, kp=0.4, ki=0.05, kd=5):

        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0
        self.prev_err = 0
        self.timestamp = time.time()

    def update(self, measurement):
        new_timestamp = time.time()
        dt = new_timestamp - self.timestamp

        err = measurement - self.threshold
        self.i += dt*err
        d = (err - self.prev_err)/dt

        self.prev_err = err
        self.timestamp = new_timestamp

        print(err, self.i, d, dt)
        return self.kp * err + self.ki * self.i + self.kd * d
