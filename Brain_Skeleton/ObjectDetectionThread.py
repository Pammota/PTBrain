from threading import Thread, Event
import time
import random
import os
from helpers import *
import cv2
import numpy as np
import tensorflow as tf
import config
from TFLiteModel import TFLiteModel
import object_detection as od
from ObjectStabilizer import ObjectStabilizer

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

        self.traffic_light_classifier_tflite = None
        self.object_detector_tflite = None
        self.stabilizer = ObjectStabilizer(3, 0.5)

        self.init_models()


    def perform_object_detection(self, image):

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        prediction = self.object_detector_tflite.predict(image)

        num_detections = 100

        scores, boxes, num_detections, labels = prediction  #tf2 mobilenet + efficientdet
        #boxes, labels, scores, num_predictions = prediction #t1 mobilenet
        #_, _, _, _, boxes, labels, scores, _ = prediction  # non edgetpu

        """last_idx = 0
        while last_idx < num_detections and scores[0][last_idx] > config.DETECTION_SCORE_THRESHOLD:
            last_idx += 1
        scores = scores[0][:last_idx]
        boxes = [[int(y1 * image.shape[0]), int(x1 * image.shape[1]),
                  int(y2 * image.shape[0]), int(x2 * image.shape[1])]
                 for y1, x1, y2, x2 in boxes[0][:last_idx]]  # rescale bounding boxes according to the image size
        labels = [int(label) for label in labels[0][:last_idx]]

        boxes, labels, scores = NM_Supress(boxes, labels, scores)
        num_detections = len(labels)

        ids = self.stabilizer.add_frame(boxes, labels, scores)

        traffic_lights_info = []

        for idx in range(len(ids)):
            exists, box, object_label, score = self.stabilizer.get_object_data(ids[idx])

            if not exists:
                continue

            y1, x1, y2, x2 = box
            score = int(score * 100)
            color = None
            object_label += 1
            label_text = ""
            color_label_text = ""

            # set the color of the bounding box and the text according to the object_label
            if object_label == config.LABEL_PERSON:
                color = (0, 255, 255)
                label_text = "Person " + str(score)
            if object_label == config.LABEL_CAR:
                color = (255, 255, 0)
                label_text = "Car " + str(score)
            if object_label == config.LABEL_STOP_SIGN:
                color = (128, 0, 0)
                label_text = "Stop Sign " + str(score)
            if object_label == config.LABEL_TRAFFIC_LIGHT:
                color = (255, 255, 255)
                label_text = "Traffic Light " + str(score)

                image_traffic_light = image[y1:y2, x1:x2]
                prediction = self.traffic_light_classifier_tflite.predict(image_traffic_light)
                prediction = prediction[0]  # model operates in batches, so output is an array of predictions
                                    # but our batch size is actually 1, so we have to take that single element
                light_label = np.argmax(prediction, axis=0)
                light_score = int(prediction[light_label] * 100)

                if light_label == 0:
                    color_label_text = "Green " + str(light_score)
                    color = (0, 255, 0)
                elif light_label == 1:
                    color_label_text = "Yellow " + str(light_score)
                elif light_label == 2:
                    color = (255, 0, 0)
                    color_label_text = "Red " + str(light_score)
                else:
                    color_label_text += "NO-LIGHT " + str(light_score)  # This is not a traffic light, or is a traffic light that is off
                traffic_lights_info.append({"color": light_label, "score": score, "box": box})

            if color and label_text:# and accept_box(boxes, box, 5.0):
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(image, label_text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            if color_label_text:
                cv2.putText(image, color_label_text, (x1, y2+13), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        output_frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        detection_output = {"num_detections": len(boxes), "boxes": boxes, "labels:": labels}"""

        return np.zeros([640, 640, 3]), {}, []#output_frame, detection_output, traffic_lights_info

    def run(self):

        (img_annotated, output, tl_info) = np.zeros([640, 640, 3]), {}, []
        while True:

            # waits for the preprocessed image and gets it
            image = self.inP_img.recv()

            ######### here takes place the object detection ###########
            img_annotated, output, tl_info = image, {}, []

            start = time.time()

            if config.RUN_MODE == "NORMAL":
                (img_annotated, output, tl_info) = od.perform_object_detection_video(self.object_detector,
                                          image, self.traffic_light_classifier)
            if config.RUN_MODE == "TFLITE":
                try:
                    (img_annotated, output, tl_info) = self.perform_object_detection(image)
                except cv2.error:
                    (img_annotated, output, tl_info) = np.zeros([640, 640, 3]), {}, []
            if config.RUN_MODE == "NO_DETECTION":
                (img_annotated, output, tl_info) = np.zeros([640, 640, 3]), {}, []

            end = time.time()
            print("Object detection time: {}".format(end - start))

            ######### here the object detection ends ###########

            self.outP_obj.send((end, img_annotated, output, tl_info))  # sends the results of the detection back

    def init_models(self):

        ###################### keras - tensorflow models ######################################
        if config.RUN_MODE == "NORMAL":
            self.traffic_light_classifier = tf.keras.models.load_model("models/model_mobilenet_v3.h5")
            self.object_detector = od.load_ssd_coco("mobilenet")
            print("done?")

        ###################### tflite models (interpreters) ###################################
        if config.RUN_MODE == "TFLITE":
            """self.traffic_light_classifier_tflite = TFLiteModel("models/model_mobilenet_v3_static_input_edgetpu.tflite",
                                                         input_shape=config.CLASSIFIER_INPUT_SHAPE,
                                                         quantized_input=True, quantized_output=True)"""
            self.object_detector_tflite = TFLiteModel("models/mobilenet_cityscapes_frozen_edgetpu.tflite",
                                                       input_shape=config.DETECTOR_INPUT_SHAPE,
                                                       quantized_input=False, quantized_output=False)
