import copy
import traceback
from threading import Thread
from multiprocessing import Pipe
from LaneDetectionThread import LaneDetectionThread
from ObjectDetectionThread import ObjectDetectionThread
from ImageAquisitionThread import ImageAquisitionThread
from Controller import Controller
from writethread import WriteThread
from DistSensor import DistanceSensor
from config import *
import serial
import time
import cv2
import numpy as np
from readthread import ReadThread
from imu_tracking import IMU_tracking
from PathTracking import *


class BrainThread(Thread):
    def __init__(self, cameraSpoof=None, show_vid=False, show_lane=False, stop_car=False):
        """

        :param cameraSpoof: holds a path to a video file for the environment to be "simulated"
                            if inexistent, the video feed will be taken from the actual camera
                            default - None
        """
        super(BrainThread, self).__init__()

        self.times = []
        self.thetas = []
        self.loop_times = []

        self.frame = None
        self.num_frames = 0
        self.last_intersection = 0

        # holds the threads managed by this object
        self.threads = []
        self.laneDetectionThread = None
        self.cameraThread = ImageAquisitionThread()
        self.imuThread = None

        # constructs the video feed (from file if cameraSpoof exists, otherwise from camera
        self.cameraSpoof = cameraSpoof


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
        self.controller = Controller(self)
        time.sleep(0.1)

        # initializes the distance sensor
        devFileDS = '/dev/ttyACM0'
        devFileNucleo = '/dev/ttyACM1'

        self.serialComNucleo = serial.Serial(devFileNucleo, 19200, timeout=0.03)
        self.serialComNucleo.flushInput()
        self.serialComNucleo.flushOutput()

        self.distanceSensor = DistanceSensor(devFileDS)
        self.distanceSensor.start()

        self.speed = 0

        # creates and starts the threads managed by this object

        self._init_threads()

    def run(self):

        #self.right_maneuver_routine()
        # self.left_maneuver_routine()
        # self.parking_maneuver()
        # self.stop_car = True
        # self.hardcoded_move(0, 0, 10, 0.001)

        obj_info = {"forward": False, "forbidden": False, "parking": False, "sem_yellow": False, "sem_red": False,
                 "sem_green": False, "priority": False, "crosswalk": False, "stop": False}
        bboxes = []
        DSFront_info = 1000
        time.sleep(0.5)
        start = time.time()
        dtstart = time.time()
        dt = 0

        while not self.stop_car:
            loop_start_time = time.time()
            frame = self.get_crt_frame()

            if self.num_frames % 2 == 0:
                self.objectDetectionThread_working = True
                self.outP_brain_obj.send(True)
                if PRINT_EXEC_TIMES:
                    print("Sent image to object detection afer {}".format(time.time() - loop_start_time))

            self.laneDetectionThread_working = True
            self.outP_brain_lane.send(True)
            if PRINT_EXEC_TIMES:
                print("Sent image to lane detection after {}".format(time.time() - loop_start_time))


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


            #dstime = time.time()
            DSFront_info = self.distanceSensor.get_current_distance()
            #print("Distance detection time: {}".format(time.time() - dstime))
            self.controller.checkState(obj_info, lane_info, DSFront_info)

            action = self.controller.takeAction()
            if action is None:
                break
            self.speed = action[ACTION_SPEED]

            self.show_image(frame, bboxes, lane_info, left_line, right_line, road_line)


            if action[ACTION_CROSSWALK] != 0:
                if self.cameraSpoof is None:
                    self.crosswalk_maneuver_routine()
                    print("Performing crosswalk routine.BRB")
                else:
                    time.sleep(2)

            elif action[ACTION_PARKING] != 0:
                if self.cameraSpoof is None:
                    self.parking_maneuver()
                else:
                    time.sleep(2)
                print("Performing parking routine.BRB")
            elif action[ACTION_DIRECTION] != 0:
                self.intersection_maneuver_routine(action[ACTION_ANGLE], action[ACTION_STOP], action[ACTION_RED], action[ACTION_DIRECTION])
            else:
                theta_command = Controller.getAngleCommand(action[ACTION_ANGLE])
                speed_command = Controller.getSpeedCommand(action[ACTION_SPEED])
                if self.cameraSpoof is None:
                    self.outP_com.send((theta_command, speed_command))
                else:
                    print("Sent command of SPEED: {}, ANGLE: {}".format(action[ACTION_SPEED], action[ACTION_ANGLE]))


            self.num_frames += 1
            end = time.time()
            if PRINT_EXEC_TIMES:
                print("Ended brain loop after {}".format(end - loop_start_time))
                print("---------------------------------------------------------------------\n\n")

            self.times.append(end - start)
            self.thetas.append(abs(lane_info['theta']))
            self.loop_times.append(end - loop_start_time)


            ############### here processing of info ends ############


        """If we want to stop the threads, we exit from the Brain thread, flush pipes, 
            and send through them a "stop" signal, which would make them break out
            of the infinite loops"""

        self.terminate()

    def intersection_maneuver_routine(self, theta, stop=False, sem_red=False, direction="forward"):

        if self.cameraSpoof is None:
            theta_command = Controller.getAngleCommand(0)
            speed_command = Controller.getSpeedCommand(0)
            self.outP_com.send((theta_command, speed_command))

        if stop == 1:
            print("Stopped at the STOP sign at intersection.BRB")
            time.sleep(3)

        if sem_red == 1:
            if self.cameraSpoof is None:
                theta_command = Controller.getAngleCommand(0)
                speed_command = Controller.getSpeedCommand(0)
                self.outP_com.send((theta_command, speed_command))
            print("Currently stopped at the red traffic light at intersection.BRB")
            return

        time.sleep(0.1)

        imuThread = IMU_tracking(self)
        imuThread.start()

        x_offsets = []
        y_offsets = []
        thetas = []

        for i in range(5):
            frame = self.get_crt_frame()
            self.outP_brain_lane.send(True)
            time_start, lane_info, left_line, right_line, road_line = self.inP_lane.recv()
            self.show_image(frame, [], lane_info, left_line, right_line, road_line)

            x_offsets.append(self.laneDetectionThread.x_offset)
            y_offsets.append(self.laneDetectionThread.y_offset)
            thetas.append(self.laneDetectionThread.theta_yaw_map)

        x_offset = np.median([x_off for x_off in x_offsets if x_off is not None])
        y_offset = np.median([y_off for y_off in y_offsets if y_off is not None])
        theta_yaw_map = np.median([tym for tym in thetas if tym is not None])
        yaw = imuThread.yaw

        try:
            self.path_tracking(case=direction, x_car=x_offset, y_car=y_offset,
                               theta_yaw_map=theta_yaw_map, yaw=yaw,
                               v=15,
                               dt=0.05, L=25.8, imuTracker=imuThread)
        except KeyboardInterrupt:
            self.terminate()
        except Exception as e:
            print(str(e))
            print(traceback.format_exc())

        imuThread.stop()
        self.controller.dir_idx += 1
        self.controller.pathPlanner.next()
        self.last_intersection = self.num_frames
        self.controller.passed_horiz_line = False
        self.controller.ongoing_intersection = False

    def crosswalk_maneuver_routine(self):
        print("Finished track")
        self.hardcoded_move(0, 0, 10, 0.2)
        time.sleep(5)
        self.hardcoded_move(0, 20, 10, 0.1)


    def right_maneuver_routine(self):
        self.hardcoded_move(0, 16, 18, 0.04)
        time.sleep(0.05)
        self.hardcoded_move(20, 16, 200, 0.025)
        time.sleep(0.05)
        self.hardcoded_move(0, 13, 1, 0.001)


    def left_maneuver_routine(self):
        self.hardcoded_move(0, 23, 5, 0.05)
        time.sleep(0.05)
        self.hardcoded_move(-16, 17, 262, 0.025)
        time.sleep(0.025)
        #self.hardcoded_move(0, 13, 3, 0.04)

    def parking_maneuver(self):
        print("Executing parking maneuver")
        self.hardcoded_move(0, -20, 10, 0.02)
        self.hardcoded_move(22.9, -20, 87, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(-22.9, -20, 81, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(-22.9, 0, 1, 0.001)
        time.sleep(2)
        self.hardcoded_move(-22.9, 20, 60, 0.02)
        self.hardcoded_move(22.9, 20, 60, 0.02)
        self.hardcoded_move(0, 20, 25, 0.02)
        time.sleep(0.02)
        self.hardcoded_move(0, 13, 1, 0.001)
        # self.hardcoded_move(0, 0, 10, 0.001)

    def forward_maneuver(self, theta):
        print("Aici e theta din intersectie: " + str(theta))
        self.hardcoded_move(0, 13, 160, 0.02)
        time.sleep(0.01)
        self.hardcoded_move(theta + 10, 13, 160, 0.02)

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
            time.sleep(s_leep + 0.02)


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


    def get_crt_speed(self):
        return self.speed

    def get_crt_frame(self):
        frame = self.cameraThread.frame
        return copy.deepcopy(frame)

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
        self.laneDetectionThread = LaneDetectionThread(inP_brain_lane, outP_lane, self)
        self.threads.append(self.laneDetectionThread)
        self.threads.append(ObjectDetectionThread(inP_brain_obj, outP_obj, self))
        if self.cameraSpoof is None:
            self.threads.append(WriteThread(self.inP_com, self.serialComNucleo,
                                            zero_theta_command, zero_speed_command))
            #self.threads.append(ReadThread(self.serialComNucleo))
        
        self.threads.append(self.cameraThread)

        """self.imuThread = IMU_tracking(self)
        self.threads.append(self.imuThread)"""

        # starts all threads
        for thread in self.threads:
            thread.start()


    def terminate(self):

        theta_command = Controller.getAngleCommand(0)
        speed_command = Controller.getSpeedCommand(0)

        if self.cameraSpoof is None:
            self.outP_com.send((theta_command, speed_command))

        print("Stopped car")

        self.controller.env_conn.stop()

        time.sleep(0.1)
        self._kill_threads()

    def _kill_threads(self):

        if self.laneDetectionThread_working:
            _, _, _, _, _ = self.inP_lane.recv()
        if self.objectDetectionThread_working:
            _, _, _ = self.inP_obj.recv()

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
        cv2.line(image, (y1_cv, x1_cv), (y2_cv, x2_cv), color, 4)

    def show_image(self, frame, bboxes, lane_info, left_line, right_line, road_line):
        ############ draw bounding boxes of objects on the screen
        for label, bbox in bboxes:
            label_text = CLASSES[label]["LABEL"]
            label_color = CLASSES[label]["COLOR"]

            x1, y1, x2, y2 = bbox

            cv2.rectangle(frame, (x1, y1), (x2, y2), label_color, 2)
            cv2.putText(frame, label_text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.55, label_color, 2)

        ############ draw lines from lane detection
        if left_line is not None:
            self.draw_line(left_line, (255, 0, 0), frame)
        if right_line is not None:
            self.draw_line(right_line, (0, 0, 255), frame)
        if road_line is not None:
            self.draw_line(road_line, (255, 255, 255), frame)
        cv2.putText(img=frame, text=str(lane_info["theta"]), org=(350, 200), fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                    fontScale=1,
                    color=(0, 255, 0), thickness=3)

        cv2.imshow("CAR POV", frame)
        cv2.waitKey(1)

    def path_tracking(self, case=None,
                        x_car=None, y_car=None, theta_yaw_map=None,
                      yaw=None, v=None, dt=None, L=None,
                      imuTracker=None
                      ):

        # info for intersection
        size_pixel = 500
        size_cm = 155
        ref_points = []

        x0, y0 = 98, 10

        pathGenerator = PathGenerator()

        intersection = False
        isForward = False

        # if case == "left":
        #     ref_thresh = 10
        #     final_thresh = 20
        #     ref_points = pathGenerator.generate_circle_points(r=90, d=9, x_c=30, y_c=30, alpha_min=0, alpha_max=1.57)
        #     end_point = (30, 120)
        #     intersection = True
        #     isForward = False
        # if case == "right":
        #     ref_thresh = 10
        #     final_thresh = 20
        #     ref_points = pathGenerator.generate_circle_points(r=40, d=6, x_c=180, y_c=30, alpha_min=1.57, alpha_max=3.14)
        #     end_point = (180, 90)
        #     intersection = True
        #     isForward = False
        # if case == "forward":
        #     ref_thresh = 10
        #     final_thresh = 20
        #     ref_points = pathGenerator.generate_line_points(x1=120, y1=30, x2=120, y2=180, n=7)
        #     end_point = (120, 180)
        #     intersection = True
        #     isForward = True

        if case == "stop":
            self.terminate()
        if case == "left":
            x0, y0 = 105, 10
            ref_thresh = 5
            final_thresh = 15
            ref_points = pathGenerator.generate_circle_points(r=95, d=9, x_c=10, y_c=13, alpha_min=0, alpha_max=1.57)
            end_point = (10, 103)
            intersection = True
            isForward = False
        if case == "right":
            ref_thresh = 10
            final_thresh = 10
            ref_points = pathGenerator.generate_circle_points(r=47, d=6, x_c=145, y_c=10, alpha_min=1.57,
                                                              alpha_max=3.14)
            ref_points.append((145, 57))
            # ref_points.append((155, 77))
            end_point = (155, 65)
            # end_point = (155, 57)
            intersection = True
            isForward = False
        if case == "forward":
            ref_thresh = 10
            final_thresh = 20
            ref_points = pathGenerator.generate_line_points(x1=98, y1=10, x2=98, y2=145, n=3)
            end_point = (98, 145)
            intersection = True
            isForward = True

        if case == "roundabout_forward":
            # info for roundabout
            x0, y0 = 117, 10
            v = 13
            ref_thresh = 5
            final_thresh = 10
            isForward = False
            size_pixel = 500
            size_cm = 234
            # ref_points_aux = pathGenerator.generate_circle_points(r=67, d=5, x_c=99, y_c=117, alpha_min=0,
            #                                                       alpha_max=1.2)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            # ref_points_aux = pathGenerator.generate_circle_points(r=67, d=5, x_c=99, y_c=117, alpha_min=5.3,
            #                                                       alpha_max=6.28)
            # for point in ref_points_aux:
            #     ref_points.append(point)

            # ref_points_aux = pathGenerator.generate_circle_points(r=57, d=5, x_c=99, y_c=117, alpha_min=0,
            #                                                       alpha_max=1.0)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            # ref_points_aux = pathGenerator.generate_circle_points(r=57, d=5, x_c=99, y_c=117, alpha_min=5.5,
            #                                                       alpha_max=6.28)
            # for point in ref_points_aux:
            #     ref_points.append(point)

            # ref_points_aux = pathGenerator.generate_line_points(x1=117, y1=40, x2=140, y2=80, n=3)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            # ref_points_aux = pathGenerator.generate_line_points(x1=140, y1=120, x2=117, y2=184, n=3)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            # ref_points.append((160, 100))
            # ref_points = [(117, 40), (140, 80), (140, 120), (87, 184), (67, 214)]
            # ref_points_aux = []
            # for i in range(0,len(ref_points)):
            #     x, y = ref_points[i]
            #     ref_points_aux.append((x-i, y))
            # ref_points = ref_points_aux
            # ref_points_aux = pathGenerator.generate_line_points(x1=117, y1=20, x2=117, y2=55, n=3)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            # ref_points_aux = pathGenerator.generate_line_points(x1=117, y1=179, x2=117, y2=224, n=3)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            # ref_points.append((120, 40))
            # ref_points.append((117, 30))
            # ref_points.append((117, 20))
            # ref_points.append((117, 204))
            # ref_points.append((117, 194))
            # ref_points.append((117, 184))
            # ref_points.append((87, 214))

            # dt = 0.01

            # trapez tracking

            ref_points.append((117, 40))
            ref_points.append((145, 70))
            ref_points.append((143, 90))
            # ref_points.append(())
            # ref_points.append((147, 107))
            ref_points.append((130, 127))
            ref_points.append((105, 159))
            ref_points.append((87, 194))




            # ref_points_aux = pathGenerator.generate_line_points(x1=135, y1=10, x2=135, y2=40, n=3)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            #
            # ref_points_aux = pathGenerator.generate_line_points(x1=135, y1=224, x2=135, y2=194, n=3)
            # for point in ref_points_aux:
            #     ref_points.append(point)
            intersection = True
            end_point = (80, 194)
            # ref_points.append(end_point)

        if case == "roundabout_right":
            # info for roundabout
            x0, y0 = 117, 10
            v = 13
            ref_thresh = 5
            final_thresh = 10
            isForward = False
            size_pixel = 500
            size_cm = 234
            intersection = True

            end_point = (197, 120)
            # ref_points.append(end_point)
            # ref_points.append((117, 40))
            # ref_points.append((147, 50))
            # ref_points.append((167, 70))

            ref_points = pathGenerator.generate_circle_points(r=80, d=10, x_c=197, y_c=40, alpha_max=3.14, alpha_min=1.57)
            ref_points.append(end_point)

            # ref_points.append((117, 40))
            # ref_points.append(())

        if intersection == True:
            map = Map(size_pixel=size_pixel, size_cm=size_cm, ref_points=ref_points)
            # ref_thresh = 10
            # final_thresh = 20
            pathTracking = PathTracking(self.outP_com, map=map, ref_points=ref_points, size_pixel=size_pixel, size_cm=size_cm,
                                        x_car=x_car + x0, y_car=y0 - y_car, theta_yaw_map=theta_yaw_map, yaw=yaw,
                                        v=v, dt=dt, ref_thresh=ref_thresh, final_thresh=final_thresh,
                                        end_point=end_point, imu_tracker=imuTracker, L=L, isForward=isForward)
            pathTracking.run()

    def overtaking_maneuver(self, distance):
        x_obs, y_obs = 100, 100
        ref_thresh = 20
        final_thresh = 25
        L = 45 # length of car
        x_front, y_front = x_obs, y_obs + L
        x = x_obs
        y = y_obs - distance
        print("distance = {} cm".format(distance))
        if distance < 50:
            steering = 0
            timp = float((50 - distance) / 13)
            speed = -13

            speed_command, theta_command = Controller.getSpeedCommand(speed), Controller.getAngleCommand(steering)
            self.outP_com.send((theta_command, speed_command))

            time.sleep(timp)
            y = y + speed * timp


        ref_points = []
        lane_x = 30
        lane_y = 25
        # ref_points.append((x_obs -35, y_obs + lane_y))
        ref_points.append((x - 40, y + 50))
        ref_points.append((x_front - 48, y_front + 70))
        # end_point = (x_front, y_front + )
        end_point = (x_front - 40, y_front + 100)
        # end_point = (x_front - 30, y_front + 140)
        ref_points.append(end_point)

        size_pixel = 300
        size_cm = 300
        imuTracker = IMU_tracking(self)
        pathTracking = PathTracking(self.outP_com, map=map, ref_points=ref_points, size_pixel=size_pixel,
                                    size_cm=size_cm,
                                    x_car=x, y_car=y, theta_yaw_map=90, yaw=90,
                                    v=13, dt=0.05, ref_thresh=ref_thresh, final_thresh=final_thresh,
                                    end_point=end_point, imu_tracker=imuTracker, L=L, isForward=False)
        pathTracking.run()


