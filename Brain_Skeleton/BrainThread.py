from threading import Thread
from multiprocessing import Pipe
from ImageProcessingThread import ImageProcessingThread
from LaneDetectionThread import LaneDetectionThread
from ObjectDetectionThread import ObjectDetectionThread
from Controller import Controller
from writethread import WriteThread
from config import *
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

        self.baseSpeed = 20

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


    def run(self):

        #self.right_maneuver_routine()
        # self.left_maneuver_routine()
        # self.parking_maneuver()
        # self.stop_car = True
        # self.hardcoded_move(0, 0, 10, 0.001)
        while not self.stop_car:

            loop_start_time = time.time()
            # grabs an image from the camera (or from the video)
            grabbed, frame = self.camera.read()

            # sends the image through the pipe if it exists
            if grabbed is True:
                for outP in self.outPs:
                    outP.send(frame)
            else:
                break

            # waits for the outputs of the other threads and gets them
            time_start, lane_info = self.inP_lane.recv()

            current_time = time.time()

            if PRINT_EXEC_TIMES:
                print("Grabbed lane detection info after {}".format(current_time - loop_start_time))
                print("Lane detection pipe delay {}".format(current_time - time_start))

            time_start, obj_info = self.inP_obj.recv()
            current_time = time.time()

            if PRINT_EXEC_TIMES:
                print("Grabbed object detection info after {}".format(current_time - loop_start_time))
                print("Object detection pipe delay {}".format(current_time - time_start))

            ############### here takes place the processing of the info #############

            self.controller.checkState(obj_info, {"theta": lane_info})
            action = self.controller.takeAction()

            if action[ACTION_CROSSWALK] != 0:
                if self.cameraSpoof is None:
                    self.crosswalk_maneuver_routine()
                else:
                    print("Performing crosswalk routine.BRB")
                    time.sleep(2)
            elif action[ACTION_PARKING] != 0:
                if self.cameraSpoof is None:
                    self.parking_maneuver()
                else:
                    print("Performing parking routine.BRB")
                    time.sleep(2)
            elif action[ACTION_DIRECTION] != 0:
                self.intersection_maneuver_routine(action[ACTION_STOP], action[ACTION_RED], action[ACTION_DIRECTION])
            else:
                theta_command = Controller.getAngleCommand(action[ACTION_ANGLE])
                speed_command = Controller.getSpeedCommand(action[ACTION_SPEED])
                if self.cameraSpoof is None:
                    self.outP_com.send((theta_command, speed_command))
                else:
                    print("Sent command of SPEED: {}, ANGLE: {}".format(action[ACTION_SPEED], action[ACTION_ANGLE]))

            end = time.time()
            if PRINT_EXEC_TIMES:
                print("Ended brain loop after {}".format(end - loop_start_time))
            print("---------------------------------------------------------------------\n\n")
            ############### here processing of info ends ############


        """If we want to stop the threads, we exit from the Brain thread, flush pipes, 
            and send through them a "stop" signal, which would make them break out
            of the infinite loops"""

        theta_command = Controller.getAngleCommand(0)
        speed_command = Controller.getSpeedCommand(0)

        if self.cameraSpoof is None:
            self.outP_com.send((theta_command, speed_command))


    def intersection_maneuver_routine(self, stop=False, sem_red=False, direction="forward"):
        if stop is True:
            if self.cameraSpoof is None:
                theta_command = Controller.getAngleCommand(0)
                speed_command, startup = Controller.getSpeedCommand(0)
                self.outP_com.send((theta_command, speed_command))
            print("Stopped at the STOP sign at intersection.BRB")
            time.sleep(2)

        if sem_red is True:
            if self.cameraSpoof is None:
                theta_command = Controller.getAngleCommand(0)
                speed_command = Controller.getSpeedCommand(0)
                self.outP_com.send((theta_command, speed_command))
            print("Currently stopped at the red traffic light at intersection.BRB")
            return

        if direction == "left":
            if self.cameraSpoof is None:
                self.left_maneuver_routine()
            print("Executing a left maneuver")
        elif direction == "right":
            if self.cameraSpoof is None:
                self.right_maneuver_routine()
            print("Executing a right maneuver")
        else:
            print("Executing a forward maneuver")


    def crosswalk_maneuver_routine(self):
        self.hardcoded_move(0, 0, 10, 0.2)
        time.sleep(0.05)
        self.hardcoded_move(0, 20, 10, 0.1)


    def right_maneuver_routine(self):
        self.hardcoded_move(0, 23, 10, 0.04)
        time.sleep(0.05)
        self.hardcoded_move(17, 20, 210, 0.025)
        time.sleep(0.05)
        self.hardcoded_move(0, 0, 10, 0.001)


    def left_maneuver_routine(self):
        self.hardcoded_move(0, 23, 20, 0.05)
        time.sleep(0.05)
        self.hardcoded_move(-12.7, 20, 250, 0.025)
        time.sleep(0.025)
        self.hardcoded_move(0, 0,20, 0.001)


    def parking_maneuver(self):
        self.hardcoded_move(0, -46, 10, 0.02)
        self.hardcoded_move(22.9, -46, 57, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(-22.9, -46, 65, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(-22.9, 0, 20, 0.001)
        time.sleep(2)
        self.hardcoded_move(-22.9, 46, 73, 0.02)
        self.hardcoded_move(22.9, 46, 50, 0.02)
        self.hardcoded_move(0, 46, 25, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(0, 0, 10, 0.001)


    def hardcoded_move(self, theta, speed, r_ange, s_leep):
        index = 0
        theta_command = Controller.getAngleCommand(theta)
        speed_command = Controller.getSpeedCommand(speed)
        for index in range(r_ange):
            self.outP_com.send((theta_command, speed_command))
            time.sleep(s_leep)


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

        zero_theta_command = Controller.getAngleCommand(0)
        zero_speed_command = Controller.getSpeedCommand(0)

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
