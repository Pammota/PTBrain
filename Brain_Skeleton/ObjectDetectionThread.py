import copy
from threading import Thread
import time
from helpers import *
import cv2
import numpy as np
import config
from TFLiteModel import TFLiteModel
from ObjectStabilizer import ObjectStabilizer

class ObjectDetectionThread(Thread):
    def __init__(self, inP_img, outP_obj, brain):
        """

        :param inP_img: receives the preprocessed image through a pipe
        :param outP_obj: outputs the result of the detection through the pipe
        """
        super(ObjectDetectionThread, self).__init__()
        self.inP_img = inP_img
        self.outP_obj = outP_obj
        self.brain = brain

        self.object_detector = None
        self.traffic_light_classifier = None

        self.traffic_light_classifier_tflite = None
        self.object_detector_tflite = None
        self.stabilizer = ObjectStabilizer(5, 0.5)

        self.init_models()


    def perform_object_detection(self, image):

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        np_image = (np.array(image, dtype=np.float32) - 127.5) / 127.5

        prediction = self.object_detector_tflite.predict(np_image)

        scores, boxes, num_predictions, labels = prediction #mobilenet v2 custom
        # boxes, labels, scores, num_predictions = prediction0
        #print(labels)

        last_idx = 5
        """while scores[0][last_idx] > config.DETECTION_SCORE_THRESHOLD and last_idx < num_predictions[0] - 1:  # config.DETECTION_SCORE_THRESHOLD:
            last_idx += 1"""
        scores = scores[0][:last_idx]
        boxes = [[int(y1 * image.shape[1]), int(x1 * image.shape[0]),
                  int(y2 * image.shape[1]), int(x2 * image.shape[0])]
                 for x1, y1, x2, y2 in boxes[0][:last_idx]]  # rescale bounding boxes according to the image size
        labels = [int(round(label) + 1) for label in labels[0][:last_idx]]
        num_detections = len(labels)
        #scores, boxes, num_detections, labels = prediction  #tf2 mobilenet + efficientdet
        #boxes, labels, scores, num_predictions = prediction #t1 mobilenet
        #_, _, _, _, boxes, labels, scores, _ = prediction  # non edgetpu

        ids = self.stabilizer.add_frame(boxes, labels, scores)

        flags = {"forward": False, "forbidden": False, "parking": False, "sem_yellow": False, "sem_red": False,
                 "sem_green": False, "priority": False, "crosswalk": False, "stop": False}
        bboxes = []

        for idx in range(len(ids)):
            exists, box, label, score = self.stabilizer.get_object_data(ids[idx])

            if not exists:
                continue

            x1, y1, x2, y2 = box
            score = int(score * 100)
            color = None
            label_text = ""

            # set the color of the bounding box and the text according to the object_label

            label_text = config.CLASSES[label]["LABEL"]
            color = config.CLASSES[label]["COLOR"]

            if label_text != "Crosswalk" and score < config.DETECTION_SCORE_THRESHOLD*100:
                continue

            if color and label_text and size_threshold_max(x1, x2, y1, y2, image.shape[0], image.shape[1]) is True \
                                    and size_threshold_min(x1, x2, y1, y2, image.shape[0], image.shape[1]) is True:# and accept_box(boxes, box, 5.0):
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(image, label_text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                #if size_threshold_min(x1, x2, y1, y2, image.shape[0], image.shape[1]) is True:
                flags[label_text] = True
                bboxes.append((label, (x1, y1, x2, y2)))

        """output_frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        output_frame = cv2.resize(output_frame, (640, 480))
        cv2.imshow("image", output_frame)
        cv2.waitKey(1)"""

        return flags, bboxes

    def run(self):

        avg_object_detection_time = 0
        num_frames = 0

        while True:

            # waits for the preprocessed image and gets it
            signal = self.inP_img.recv()
            if signal is False:
                break

            recvd = self.brain.get_crt_frame()
            image = copy.deepcopy(recvd)

            ######### here takes place the object detection ###########
            flags = {"forward": False, "forbidden": False, "parking": False, "sem_yellow": False, "sem_red": False,
                     "sem_green": False, "priority": False, "crosswalk": False, "stop": False}
            bboxes = []

            start = time.time()

            if config.RUN_MODE == "TFLITE":
                try:
                    flags, bboxes = self.perform_object_detection(image)
                except cv2.error:
                    print("cv2 error")
            if config.RUN_MODE == "NO_DETECTION":
                flags = flags
                bboxes = bboxes

            end = time.time()
            if config.PRINT_EXEC_TIMES:
                print("Object detection time: {}".format(end - start))
                num_frames += 1
                avg_object_detection_time *= (num_frames - 1)/num_frames
                avg_object_detection_time += 1/num_frames * (end - start)
                print("Average object detection time: {}".format(avg_object_detection_time))

            ######### here the object detection ends ###########

            self.outP_obj.send((end, flags, bboxes))  # sends the results of the detection back

    def init_models(self):

        ###################### tflite models (interpreters) ###################################
        if config.RUN_MODE == "TFLITE":
            """self.traffic_light_classifier_tflite = TFLiteModel("models/model_mobilenet_v3_static_input_edgetpu.tflite",
                                                         input_shape=config.CLASSIFIER_INPUT_SHAPE,
                                                         quantized_input=True, quantized_output=True)"""
            self.object_detector_tflite = TFLiteModel("models/mobilenet_160_aug_less_city_edgetpu.tflite",
                                                      input_shape=config.DETECTOR_INPUT_SHAPE,
                                                      quantized_input=False, quantized_output=False)
