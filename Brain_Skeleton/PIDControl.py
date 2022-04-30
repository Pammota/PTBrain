import numpy as np

class PIDControl():
    def __init__(self, threshold, kp=0.2, ki=0.1, kd=0.1):

        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.isum = 0
        self.prev_err = 0

    def update(self, measurement, dt):
        err = measurement - self.threshold
        i = self.isum + dt * err