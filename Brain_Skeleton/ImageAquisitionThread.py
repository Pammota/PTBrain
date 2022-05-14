import time
from threading import Thread
import numpy as np
import cv2

class ImageAquisitionThread(Thread):
    def __init__(self):
        super(ImageAquisitionThread, self).__init__()

        self.camera = cv2.VideoCapture(0)
        self.frame = np.zeros((640, 480, 3))
        self.__running = True

    def run(self):
        while self.__running:
            grabbed, frame = self.camera.read()
            if grabbed:
                self.frame = frame
            time.sleep(0.04)
    
    def stop(self):
        self.__running = False