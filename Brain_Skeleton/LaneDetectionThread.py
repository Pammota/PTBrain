from threading import Thread, Event, Barrier
import time
import random
import os

class LaneDetectionThread(Thread):
    def __init__(self, inP_img, outP_lane):
        """

        :param inP_img: receives a preprocessed image from a pipe
        :param outP_lane: outputs the result of the detection through the pipe
        """
        super(LaneDetectionThread, self).__init__()
        self.inP_img = inP_img
        self.outP_lane = outP_lane

    def run(self):

        start = time.time()

        while True:

            # waits for the preprocessed image and gets it
            image = self.inP_img.recv()

            ######### here takes place the lane detection ###########

            theta = 0
            disp = time.time() - start

            disp -= 5

            disp *= 3

            theta = int(disp)

            time.sleep(0.002)

            ######### here the lane detection ends ###########

            self.outP_lane.send(theta)  # sends the results of the detection back

