from threading import Thread
from multiprocessing import Pipe
from ImageProcessingThread import ImageProcessingThread
from LaneDetectionThread import LaneDetectionThread
from ObjectDetectionThread import ObjectDetectionThread
from writethread import WriteThread
from Controller import Controller
from pynput import keyboard
import time
import cv2
import numpy as np
import json
import os

class BrainThread(Thread):
    def __init__(self, cameraSpoof=None, show_vid=False, show_lane=False, stop_car=False):
        """

        :param cameraSpoof: holds a path to a video file for the environment to be "simulated"
                            if inexistent, the video feed will be taken from the actual camera
                            default - None
        """
        super(BrainThread, self).__init__()

        self.threads = []  # holds the threads managed by this object
        self.writethread = None
        self.lanedetectionthread = None

        # constructs the video feed (from file if cameraSpoof exists, otherwise from camera
        self.cameraSpoof = cameraSpoof
        self.camera = cv2.VideoCapture(0 if cameraSpoof is None else cameraSpoof)

        self.baseSpeed = 17

        self.show_vid = show_vid
        self.show_lane = show_lane
        self.stop_car = stop_car

        #  holds pipes managed by this object
        self.outP_img = None
        self.inP_lane = None
        self.inP_obj = None
        self.outP_com = None
        self.inP_com = None

        #  creates a controller object to control the car
        self.controller = Controller()
        time.sleep(0.1)

        # creates and starts the threads managed by this object
        self._init_threads()

        self.traffic_light_history = []

    def run(self):

        # grabs the first image from the camera so it can be preprocessed before
        # anything else is processed

        # sends the image through the pipe if it exists

        listener = keyboard.Listener(on_press=self.keyPress)
        listener.start()

        start = time.time()

        startup, ex_startup = False, False
        time_startup = 0

        print(self.stop_car)

        while not self.stop_car:

            loop_start_time = time.time()
            # grabs an image from the camera (or from the video)
            grabbed, frame = self.camera.read()

            current_time = time.time()
            print("Grabbed camera image after {}".format(current_time - loop_start_time))

            # sends the image through the pipe if it exists
            if grabbed is True:
                for outP in self.outPs:
                    outP.send(frame)

                    current_time = time.time()
                    print("Sent first image to detection after {}".format(current_time - loop_start_time))
            else:
                break

            """current_time = time.time()
            print("Sent detection info after {}".format(current_time - loop_start_time))"""

            # waits for the outputs of the other threads and gets them
            time_start, lane_info = self.inP_lane.recv()

            current_time = time.time()
            print("Grabbed lane detection info after {}".format(current_time - loop_start_time))
            print("Lane detection pipe delay {}".format(current_time - time_start))

            time_start, annotated_image, obj_info, traffic_lights_info = self.inP_obj.recv()

            current_time = time.time()
            print("Grabbed object detection info after {}".format(current_time - loop_start_time))
            print("Object detection pipe delay {}".format(current_time - time_start))

            #print(traffic_lights_info)

            ############### here takes place the processing of the info #############

            crt_angle = float(self.controller.angle)
            theta_command = self.controller.update_angle(lane_info)

            time_elapsed = time.time() - time_startup

            crt_speed = float(self.controller.speed/100.0)
            if abs(self.controller.angle) > 6:
                speed = self.baseSpeed + 3
            else:
                speed = self.baseSpeed

            must_stop = Controller.must_stop(traffic_lights_info)
            if must_stop:
                print("controller thinks we should stop")
                self.traffic_light_history.append(0)  #appends 0 if color is red or yellow
            else:
                self.traffic_light_history.append(1)  #appends 1 if color is green or no light

            n = max(0, len(self.traffic_light_history) - 5)
            self.traffic_light_history = self.traffic_light_history[n:]
            #print(self.traffic_light_history)
            median_state = sum(self.traffic_light_history[n:])  #sums green states
            if median_state <= 3:  # if less than haf green states then we must stop
                speed = 0

            #print(speed)

            speed_command, startup = self.controller.update_speed(speed, startup, time_elapsed=time_elapsed)

            if startup is True and ex_startup is False:
                time_startup = time.time()

            ex_startup = startup

            if self.cameraSpoof is None:
                self.outP_com.send((theta_command, speed_command))

            end = time.time()
            print("Ended brain loop after {}".format(end - loop_start_time))
            print("---------------------------------------------------------------------\n\n")
            ############### here processing of info ends ############

            """cv2.imshow("video", annotated_image)
            cv2.waitKey(1)"""

        """If we want to stop the threads, we exit from the Brain thread, flush pipes, 
            and send through them a "stop" signal, which would make them break out
            of the infinite loops"""


        theta_command = self.controller.update_angle(0)

        speed_command, startup = self.controller.update_speed(0)

        if self.cameraSpoof is None:
            self.outP_com.send((theta_command, speed_command))

    def keyPress(self, key):
        if key.char == 's':
            self.stop_car = True

    def send_command(self, command):
        if self.cameraSpoof is None:
            for i in range(10):
                time.sleep(0.001)
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

        zero_theta_command = self.controller.update_angle(0)
        zero_speed_command, _ = self.controller.update_speed(0)

        #self.outP_img, inP_img = Pipe()  # out will be sent from BrainThread (here),
                                   # in will be recieved in ImageProcessingThread

        outP_brain_lane, inP_brain_lane = Pipe()  # out will be sent from BrainThread
                                                     # in will be recieved in LaneDetectionThread

        outP_brain_obj, inP_brain_obj = Pipe()  # out will be sent from BrainThread
                                                   # in will be recieved in ObjectDetectionThread

        self.outPs = [outP_brain_obj, outP_brain_lane]

        outP_lane, self.inP_lane = Pipe()  # out will be sent from LaneDetectionThread
                                     # in will be recieved in BrainThread (here)

        outP_obj, self.inP_obj = Pipe()  # out will be sent from ObjectDetectionThread
                                   # in will be recieved in BrainThread (here)

        if self.cameraSpoof is None:
            self.outP_com, self.inP_com = Pipe()  # out will be sent from BrainThread (here)
                                            # in will  be received in writeThread

        # adds threads
        #self.threads.append(ImageProcessingThread(inP_img, [outP_imgProc_lane, outP_imgProc_obj]))
        self.threads.append(LaneDetectionThread(inP_brain_lane, outP_lane, show_lane=self.show_lane))
        self.lanedetectionthread = self.threads[-1]
        self.threads.append(ObjectDetectionThread(inP_brain_obj, outP_obj))
        if self.cameraSpoof is None:
            self.threads.append(WriteThread(self.inP_com, zero_theta_command, zero_speed_command))
            self.writethread = self.threads[-1]

        # starts all threads
        for thread in self.threads:
            thread.start()

    def _kill_threads(self):
        raise NotImplementedError
