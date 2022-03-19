from threading import Thread
from multiprocessing import Pipe
from ImageProcessingThread import ImageProcessingThread
from LaneDetectionThread import LaneDetectionThread
from ObjectDetectionThread import ObjectDetectionThread
from Controller import Controller
from writethread import WriteThread
from config import *
import serial
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

        self.num_frames = 0
        self.last_intersection = 0

        # holds the threads managed by this object
        self.threads = []

        # constructs the video feed (from file if cameraSpoof exists, otherwise from camera
        self.cameraSpoof = cameraSpoof
        self.camera = cv2.VideoCapture(0 if cameraSpoof is None else cameraSpoof)

        # params for video debugging
        self.show_vid = show_vid
        self.show_lane = show_lane
        self.stop_car = stop_car

        #  holds pipes managed by this object
        self.outP_img = None
        self.inP_lane = None
        self.inP_obj = None
        self.outP_com = None
        self.inP_com = None
        self.outP_brain_lane = None
        self.outP_brain_obj = None

        # booleans that keep track of wether threads are actively working or just waiting
        self.laneDetectionThread_working = False
        self.objectDetectionThread_working = False

        #  creates a controller object to control the car
        self.controller = Controller()
        time.sleep(0.1)

        # creates and starts the threads managed by this object
        self._init_threads()

        # initializes the distance sensor
        devFile = '/dev/ttyACM1'

        """self.serialCom = serial.Serial(devFile, 9600)
        self.serialCom.flushInput()
        self.serialCom.flushOutput()"""


    def run(self):

        #self.right_maneuver_routine()
        # self.left_maneuver_routine()
        # self.parking_maneuver()
        # self.stop_car = True
        # self.hardcoded_move(0, 0, 10, 0.001)

        obj_info = {"forward": False, "forbidden": False, "parking": False, "sem_yellow": False, "sem_red": False,
                 "sem_green": False, "priority": False, "crosswalk": False, "stop": False}
        bboxes = []

        while not self.stop_car:
            loop_start_time = time.time()
            # grabs an image from the camera (or from the video)
            grabbed, frame = self.camera.read()

            # sends the image through the pipe if it exists
            if grabbed is True:
                """for outP in self.outPs:
                    outP.send(frame)"""
                self.laneDetectionThread_working = True
                self.outP_brain_lane.send(frame)
                if self.num_frames % 2 == 0:
                    self.objectDetectionThread_working = True
                    self.outP_brain_obj.send(frame)
            else:
                break

            # waits for the outputs of the other threads and gets them
            time_start, lane_info, left_line, right_line, road_line = self.inP_lane.recv()
            self.laneDetectionThread_working = False

            current_time = time.time()

            if PRINT_EXEC_TIMES:
                print("Grabbed lane detection info after {}".format(current_time - loop_start_time))
                print("Lane detection pipe delay {}".format(current_time - time_start))

            if self.num_frames % 2 == 1:
                time_start, obj_info, bboxes = self.inP_obj.recv()
                self.objectDetectionThread_working = False
                current_time = time.time()

                if PRINT_EXEC_TIMES:
                    print("Grabbed object detection info after {}".format(current_time - loop_start_time))
                    print("Object detection pipe delay {}".format(current_time - time_start))

            ############### here takes place the processing of the info #############

            #DSFront_info = self.get_distance_info()

            self.controller.checkState(obj_info, lane_info)#, DSFront_info)
            action = self.controller.takeAction()

            if action is None:
                break

            if action[ACTION_CROSSWALK] != 0:
                if self.cameraSpoof is None:
                    self.crosswalk_maneuver_routine()
                    print("Performing crosswalk routine.BRB")
                else:
                    time.sleep(2)

            elif action[ACTION_PARKING] != 0:
                if self.cameraSpoof is None:
                    self.parking_maneuver()
                print("Performing parking routine.BRB")
                time.sleep(2)
            elif action[ACTION_DIRECTION] != 0 and ((self.num_frames - self.last_intersection > 10)\
                    or self.last_intersection == 0):
                self.intersection_maneuver_routine(action[ACTION_STOP], action[ACTION_RED], action[ACTION_DIRECTION])
            else:
                theta_command = Controller.getAngleCommand(action[ACTION_ANGLE])
                speed_command = Controller.getSpeedCommand(action[ACTION_SPEED])
                if self.cameraSpoof is None:
                    self.outP_com.send((theta_command, speed_command))
                else:
                    print("Sent command of SPEED: {}, ANGLE: {}".format(action[ACTION_SPEED], action[ACTION_ANGLE]))

            ############ draw bounding boxes of objects on the screen
            for label, bbox in bboxes:
                label_text = CLASSES[label]["LABEL"]
                label_color = CLASSES[label]["COLOR"]

                x1, y1, x2, y2 = bbox

                cv2.rectangle(frame, (x1, y1), (x2, y2), label_color, 2)
                cv2.putText(frame, label_text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.55, label_color, 2)

            ############ draw lines from lane detection
            if len(left_line) == 4:
                self.draw_line(left_line, (0, 255, 0), frame)
            if len(right_line) == 4:
                self.draw_line(right_line, (0, 255, 0), frame)
            if len(road_line) == 4:
                self.draw_line(road_line, (255, 255, 255), frame)


            cv2.imshow("CAR POV", frame)
            cv2.waitKey(1)

            self.num_frames += 1
            end = time.time()
            if PRINT_EXEC_TIMES:
                print("Ended brain loop after {}".format(end - loop_start_time))
            print("---------------------------------------------------------------------\n\n")
            ############### here processing of info ends ############


        """If we want to stop the threads, we exit from the Brain thread, flush pipes, 
            and send through them a "stop" signal, which would make them break out
            of the infinite loops"""

        raise InterruptedError

    def intersection_maneuver_routine(self, stop=False, sem_red=False, direction="forward"):
        if stop == 1:
            if self.cameraSpoof is None:
                theta_command = Controller.getAngleCommand(0)
                speed_command = Controller.getSpeedCommand(0)
                self.outP_com.send((theta_command, speed_command))
            print("Stopped at the STOP sign at intersection.BRB")
            time.sleep(2)

        if sem_red == 1:
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
            self.forward_maneuver()
            print("Executing a forward maneuver")

        self.last_intersection = self.num_frames
        self.controller.ongoing_intersection = False


    def crosswalk_maneuver_routine(self):
        print("Executing Crosswalk routine")
        self.hardcoded_move(0, 0, 10, 0.2)
        time.sleep(0.05)
        self.hardcoded_move(0, 20, 10, 0.1)


    def right_maneuver_routine(self):
        self.hardcoded_move(0, 17, 10, 0.04)
        time.sleep(0.05)
        self.hardcoded_move(22.9, 17, 210, 0.025)
        time.sleep(0.05)
        self.hardcoded_move(0, 13, 1, 0.001)


    def left_maneuver_routine(self):
        self.hardcoded_move(0, 23, 5, 0.05)
        time.sleep(0.05)
        self.hardcoded_move(-16, 17, 290, 0.025)
        time.sleep(0.025)
        #self.hardcoded_move(0, 13, 3, 0.04)

    def parking_maneuver(self):
        print("Executing parking routine")
        self.hardcoded_move(0, -20, 10, 0.02)
        self.hardcoded_move(22.9, -20, 87, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(-22.9, -20, 81, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(-22.9, 0, 1, 0.001)
        time.sleep(2)
        self.hardcoded_move(-22.9, 20, 66, 0.02)
        self.hardcoded_move(22.9, 20, 63, 0.02)
        self.hardcoded_move(0, 20, 25, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(0, 13, 1, 0.001)
        # self.hardcoded_move(0, 0, 10, 0.001)

    def forward_maneuver(self):
        self.hardcoded_move(13, 0, 150, 0.02)

    def hardcoded_move(self, theta, speed, r_ange, s_leep):
        index = 0
        theta_command = Controller.getAngleCommand(theta)
        speed_command = Controller.getSpeedCommand(speed)
        if r_ange > 200:
            r_ange -= 30
        if s_leep > 0.011:
            s_leep -= 0.01
        for index in range(r_ange):
            self.outP_com.send((theta_command, speed_command))
            grabbed, image = self.camera.read()
            cv2.imshow("image", image)
            cv2.waitKey(1)
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

    def get_distance_info(self):
        rec_data = self.serialCom.read(10)
        data_left = self.serialCom.inWaiting()
        rec_data += self.serialCom.read(data_left)
        print(rec_data)

        rec_numbers = [int(s) for s in rec_data.split() if s.isdigit()]

        rec_number = 3
        try:
            rec_number = rec_numbers[0]
        except IndexError:
            rec_number = rec_number = 3
        print(rec_number)
        return rec_number


    def _init_threads(self):

        # defines the pipes for interthread communication

        zero_theta_command = Controller.getAngleCommand(0)
        zero_speed_command = Controller.getSpeedCommand(0)

        #self.outP_img, inP_img = Pipe()  # out will be sent from BrainThread (here),
                                   # in will be recieved in ImageProcessingThread

        self.outP_brain_lane, inP_brain_lane = Pipe()  # out will be sent from BrainThread
                                                     # in will be recieved in LaneDetectionThread

        self.outP_brain_obj, inP_brain_obj = Pipe()  # out will be sent from BrainThread
                                                   # in will be recieved in ObjectDetectionThread

        #self.outPs = [outP_brain_obj, outP_brain_lane]

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
        self.threads.append(ObjectDetectionThread(inP_brain_obj, outP_obj))
        if self.cameraSpoof is None:
            self.threads.append(WriteThread(self.inP_com, zero_theta_command, zero_speed_command))

        # starts all threads
        for thread in self.threads:
            thread.start()

    def terminate(self):
        theta_command = Controller.getAngleCommand(0)
        speed_command = Controller.getSpeedCommand(0)

        if self.cameraSpoof is None:
            self.outP_com.send((theta_command, speed_command))

        print("Stopped car")

        self._kill_threads()

    def _kill_threads(self):

        if self.laneDetectionThread_working:
            _, _ = self.inP_lane.recv()
        if self.objectDetectionThread_working:
            _, _ = self.inP_obj.recv()

        self.outP_brain_lane.send(None)
        self.outP_brain_obj.send(None)
        self.outP_com.send(None)

        print("Stopped threads")

    def draw_line(self, line, color, image):
        y1_cv, x1_cv, y2_cv, x2_cv = line
        x1_cv += 270
        x2_cv += 270
        radius = 5
        color_left_most_point = (0, 255, 0)  # GREEN for left_most point
        color_right_most_point = (255, 0, 0)  # BLUE fpr right_most point
        cv2.circle(image, (y1_cv, x1_cv), radius, color_left_most_point, 1)
        cv2.circle(image, (y2_cv, x2_cv), radius, color_right_most_point, 1)
        cv2.line(image, (y1_cv, x1_cv), (y2_cv, x2_cv), color, 2)
