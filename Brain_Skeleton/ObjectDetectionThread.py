from threading import Thread, Event, Barrier
import time
import random
import os

from tensorflow.keras.models import load_model
import tensorflow as tf

import object_detection as od


class ObjectDetectionThread(Thread):
    def __init__(self, inP_img, outP_obj):
        """

        :param inP_img: receives the preprocessed image through a pipe
        :param outP_obj: outputs the result of the detection through the pipe
        """
        super(ObjectDetectionThread, self).__init__()
        self.inP_img = inP_img
        self.outP_obj = outP_obj

        self.object_detector = None
        self.traffic_light_classifier = None
        self.init_models()

    def run(self):
        for i in range(10):
            print("Initiating!!!!")
        while True:

            # waits for the preprocessed image and gets it
            image, active = self.inP_img.recv()
            if active:
                (img_annotated, output, tl_info) = od.perform_object_detection_video(self.object_detector,
                                              image, self.traffic_light_classifier)
            else:
                img_annotated = image
                output = {}
                tl_info = []

            ######### here takes place the lane detection ###########

            ######### here the lane detection ends ###########

            self.outP_obj.send((img_annotated, output, tl_info))  # sends the results of the detection back

    def init_models(self):
        self.traffic_light_classifier = load_model("model_mobilenet_v3.h5")
        print("done?")
        self.object_detector = od.load_ssd_coco("mobilenet")
        print("done?")
