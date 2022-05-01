import numpy as np

class PIDControl():
    def __init__(self, threshold, kp=0.3, ki=0.05, kd=0.01):

        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0
        self.prev_err = 0

    def update(self, measurement, dt):

        err = measurement - self.threshold
        self.i += dt*(3*self.prev_err - 2*err)/2
        d = (err - self.prev_err)/dt

        self.prev_err = err
        #print(err, self.i, d, dt)
        return self.kp * err + self.ki * self.i + self.kd * d
