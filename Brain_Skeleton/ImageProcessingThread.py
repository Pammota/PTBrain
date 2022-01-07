from threading import Thread, Event, Barrier
import copy
import time
import random
import cv2
import os

class ImageProcessingThread(Thread):
    def __init__(self, inP_img, outPs_img):
        """

        :param inP_img: input pipe for the raw image
        :param outPs_img: list of output pipes of the processed image
        """
        super(ImageProcessingThread, self).__init__()

        self.inP_img = inP_img
        self.outPs_img = outPs_img

    def run(self):

        while True:

            # waits for the raw image and gets it
            frame, active = self.inP_img.recv()
            frameClone = copy.copy(frame)

            ################ here takes place the processing of the image ###########

            ################ here the processing of the image ends ###########

            # sends the processed frame through pipes to the other threads
            for outP in self.outPs_img:
                outP.send((frameClone, active))
