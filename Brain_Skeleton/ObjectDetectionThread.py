from threading import Thread, Event, Barrier
import time
import random
import os

class ObjectDetectionThread(Thread):
    def __init__(self, inP_img, outP_obj):
        """

        :param inP_img: receives the preprocessed image through a pipe
        :param outP_obj: outputs the result of the detection through the pipe
        """
        super(ObjectDetectionThread, self).__init__()
        self.inP_img = inP_img
        self.outP_obj = outP_obj

    def run(self):

        while True:

            # waits for the preprocessed image and gets it
            image = self.inP_img.recv()

            ######### here takes place the lane detection ###########

            ######### here the lane detection ends ###########

            self.outP_obj.send(0)  # sends the results of the detection back
