from threading import Thread
from multiprocessing import Pipe
from ImageProcessingThread import ImageProcessingThread
from LaneDetectionThread import LaneDetectionThread
from ObjectDetectionThread import ObjectDetectionThread
from writethread import WriteThread
from Controller import Controller
import time
import cv2
import numpy as np
import json
import os

class BrainThread(Thread):
    def __init__(self, cameraSpoof=None):
        """

        :param cameraSpoof: holds a path to a video file for the environment to be "simulated"
                            if inexistent, the video feed will be taken from the actual camera
                            default - None
        """
        super(BrainThread, self).__init__()

        self.threads = []  # holds the threads managed by this object

        # constructs the video feed (from file if cameraSpoof exists, otherwise from camera
        self.cameraSpoof = cameraSpoof
        self.camera = cv2.VideoCapture(0 if cameraSpoof is None else cameraSpoof)

        #  holds pipes managed by this object
        self.outP_img = None
        self.inP_lane = None
        self.inP_obj = None
        self.outP_com = None
        self.inP_com = None

        self.controller = Controller()

        # creates and starts the threads managed by this object
        self._init_threads()

    def run(self):

        # grabs the first image from the camera so it can be preprocessed before
        # anything else is processed
        grabbed, frame = self.camera.read()

        # sends the image through the pipe if it exists
        if grabbed is True:
            self.outP_img.send(frame)

        start = time.time()

        startup, ex_startup = False, False
        time_startup = 0

        while True:
            # grabs an image from the camera (or from the video)
            grabbed, frame = self.camera.read()

            # sends the image through the pipe if it exists
            if grabbed is True:
                self.outP_img.send(frame)
            else:
                break

            # waits for the outputs of the other threads and gets them
            lane_info = self.inP_lane.recv()
            obj_info = self.inP_obj.recv()

            ############### here takes place the processing of the info #############

            crt_angle = float(self.controller.angle)
            command = self.controller.update_angle(lane_info)
            self.outP_com.send(command)

            time_elapsed = time.time() - time_startup

            crt_speed = float(self.controller.speed/100.0)
            command, startup = self.controller.update_speed(17, startup, time_elapsed=time_elapsed)
            self.outP_com.send(command)

            if startup is True and ex_startup is False:
                time_startup = time.time()

            ex_startup = startup

            end = time.time()
            if end - start > 10:
                time.sleep(0.01)
                break
            ############### here processing of info ends ############

            # messaage sent to the serial con or logged

            if self.cameraSpoof is not None:
                cv2.imshow("video", frame)
                cv2.waitKey(1)

        command = self.controller.update_angle(0)
        self.outP_com.send(command)

        command, startup = self.controller.update_speed(0)
        self.outP_com.send(command)

    def plot_timeframes_graph(self, timeframes):
        canvas = np.ones((600, 1200, 3)) * 255

        # setting up graph
        ticks_bottom = [i*100 for i in range(11)][1:]

        ticks_left = {"Brain": 100, "Preproc": 200, "Lane Det": 300, "Obj Det": 400}

        for tick in ticks_bottom:
            cv2.line(canvas, (tick, 590), (tick, 600), (0, 0, 0))
            cv2.putText(canvas, str(tick/100), (tick-12, 590), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

        for key, val in ticks_left.items():
            cv2.line(canvas, (0, val), (10, val), (0, 0, 0))
            cv2.putText(canvas, key, (10, val+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
                  (0, 255, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]


        for key, lst in timeframes.items():
            offset = -50
            for c, t in zip(colors, lst):
                cv2.line(canvas, (100+int(t["start"]*100), ticks_left[key] + offset),
                                 (100+int(t["end"]*100), ticks_left[key] + offset),
                                 c, thickness=8)
                offset += 10

        cv2.imshow("graph", canvas)
        cv2.waitKey(0)

    def _init_threads(self):

        # defines the pipes for interthread communication

        self.outP_img, inP_img = Pipe()  # out will be sent from BrainThread (here),
                                   # in will be recieved in ImageProcessingThread

        outP_imgProc_lane, inP_imgProc_lane = Pipe()  # out will be sent from ImageProcessingThread
                                                     # in will be recieved in LaneDetectionThread

        outP_imgProc_obj, inP_imgProc_obj = Pipe()  # out will be sent from ImageProcessingThread
                                                   # in will be recieved in ObjectDetectionThread

        outP_lane, self.inP_lane = Pipe()  # out will be sent from LaneDetectionThread
                                     # in will be recieved in BrainThread (here)

        outP_obj, self.inP_obj = Pipe()  # out will be sent from ObjectDetectionThread
                                   # in will be recieved in BrainThread (here)

        self.outP_com, self.inP_com = Pipe()  # out will be sent from BrainThread (here)
                                            # in will  be received in writeThread

        # adds threads
        self.threads.append(ImageProcessingThread(inP_img, [outP_imgProc_lane, outP_imgProc_obj]))
        self.threads.append(LaneDetectionThread(inP_imgProc_lane, outP_lane))
        self.threads.append(ObjectDetectionThread(inP_imgProc_obj, outP_obj))
        self.threads.append(WriteThread(self.inP_com))

        # starts all threads
        for thread in self.threads:
            thread.start()

    def _kill_threads(self):
        raise NotImplementedError
