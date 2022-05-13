import numpy as np
from helpers import *
import cv2

class HaarCascadeClassifier():
    def __init__(self):
        self.detectors = [cv2.CascadeClassifier("cascades/trained_cascade_32_975_1_1000/cascade.xml"),
                          cv2.CascadeClassifier("cascades/trained_cascade_24_975_1/cascade.xml"),
                          cv2.CascadeClassifier("cascades/trained_cascade_32_985_35_1000/cascade.xml")]
        self.sizes = [(32, 32), (24, 24), (32, 32)]
        self.n_neighb = [18, 3, 28]

    def wait_pedestrian(self, cameraThread):
