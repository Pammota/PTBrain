from threading import Thread
import time
import math
import numpy as np
import cv2

class LaneDetectionThread(Thread):
    def __init__(self, inP_img, outP_lane, show_lane=False):
        """

        :param inP_img: receives a preprocessed image from a pipe
        :param outP_lane: outputs the result of the detection through the pipe
        """
        super(LaneDetectionThread, self).__init__()
        self.inP_img = inP_img
        self.outP_lane = outP_lane
        self.show_lane = show_lane

    def getTheta(self, lane, isRight):
        x1, y1, x2, y2 = lane

        if isRight:
            slope = float((x1 - x2) / (y2 - y1))
        else:
            slope = float((x1 - x2) / (y2 - y1))

        theta = math.atan(slope)

        return - theta * 35


    def preprocessing(self, frame):
        frame_copy = frame[int(int(frame.shape[0] * 0.7)):, :]

        frame_grayscale = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)

        frame_blurred = cv2.GaussianBlur(frame_grayscale, (7, 7), cv2.BORDER_DEFAULT)

        frame_edge = cv2.Canny(frame_blurred, 100, 200)

        return frame_edge

    def averagelanes(self, lanes):
        x1_f = 0
        y1_f = 0
        x2_f = 0
        y2_f = 0

        for lane in lanes:
            x1, y1, x2, y2 = lane

            x1_f += x1
            y1_f += y1

            x2_f += x2
            y2_f += y2

        x1_f = int(x1_f / len(lanes))
        y1_f = int(y1_f / len(lanes))
        x2_f = int(x2_f / len(lanes))
        y2_f = int(y2_f / len(lanes))
        return [x1_f, y1_f, x2_f, y2_f]

    def draw_lane(self, image, lane, color):
        x1, y1, x2, y2 = lane
        cv2.line(image, (x1, y1), (x2, y2), color, 5)

    def general_equation_form(self, x1, y1, x2, y2):
        A = (y1 - y2)
        B = (x1 - x2)
        C = (x1 * y2 - x2 * y1)
        return A, B, -C

    def intersection(self, L1, L2):
        D = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x, -y
        else:
            return False

    def angle(self, left_lane, right_lane, width, height):
        # print("height = " + str(height))
        # print("width = " + str(width))
        x1l, y1l, x2l, y2l = left_lane
        y1l = -(y1l - height)
        # print("y1l = " + str(y1l) + ", x1l = " + str(x1l))
        y2l = -(y2l - height)
        # print("y2l = " + str(y2l) + ", x2l = " + str(x2l))

        x1r, y1r, x2r, y2r = right_lane
        # print("y2r before = " + str(y2r))
        y1r = -(y1r - height)
        # print("y1r = " + str(y1r) + ", x1r = " + str(x1r))
        y2r = -(y2r - height)
        # print("y2r = " + str(y2r) + ", x2r = " + str(x2r))

        L1 = self.general_equation_form(x1l, y1l, x2l, y2l)  # L1 -> left lane
        L2 = self.general_equation_form(x1r, y1r, x2r, y2r)  # l2 -> right lane

        x0, y0 = self.intersection(L1, L2)

        # print("x0 = " + str(x0) + ", y0 = " + str(y0))
        # print("x0 - width / 2 = " + str((x0 - width / 2)))
        # print("tan = " + str(float((x0 - width / 2) / y0)))
        theta = math.atan(float((x0 - width / 2) / y0))
        return theta * 35

    def run(self):
        start = time.time()
        theta_list = []
        theta_average = 0.0

        while True:

            # waits for the preprocessed image and gets it
            frame, active = self.inP_img.recv()

            ######### here takes place the lane detection ###########
            frame_edge = self.preprocessing(frame)

            lines = cv2.HoughLinesP(frame_edge, rho=1, theta=np.pi / 180, threshold=70, minLineLength=10,
                                    maxLineGap=100)
            left_lanes = []
            right_lanes = []
            frame_copy = frame[int(frame.shape[0] * 0.7):, :]  # used for displaying

            if lines is not None:
                # classify lanes based on their slope
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    slope = float((y2 - y1) / (x2 - x1))
                    if slope < 0.0:
                        left_lanes.append([x1, y1, x2, y2])
                    else:
                        right_lanes.append([x1, y1, x2, y2])

                if len(left_lanes) and len(right_lanes):  # identify both lanes

                    left_lane = self.averagelanes(left_lanes)
                    self.draw_lane(frame_copy, left_lane, (255, 0, 0))

                    right_lane = self.averagelanes(right_lanes)
                    self.draw_lane(frame_copy, right_lane, (0, 0, 255))

                    theta = self.angle(left_lane, right_lane, frame_copy.shape[1], frame_copy.shape[0])
                else:
                    if len(left_lanes):  # identify only left_lanes
                        left_lane = self.averagelanes(left_lanes)
                        theta = self.getTheta(left_lane, False)
                        left_lane = self.averagelanes(left_lanes)
                        self.draw_lane(frame_copy, left_lane, (255, 0, 0))
                    else:
                        if len(right_lanes):
                            # print("right")
                            right_lane = self.averagelanes(right_lanes)
                            theta = self.getTheta(right_lane, True)
                            right_lane = self.averagelanes(right_lanes)
                            self.draw_lane(frame_copy, right_lane, (0, 0, 255))
                        else:
                            theta = theta_list[theta_list[len(theta_list)-1]]
                # print(str(i) + ": theta = " + str(theta))

                if len(theta_list) != 50:
                    theta_average = 0
                    theta_list.append(theta)
                else:
                    theta_average = theta_list[0]
                    theta_list = theta_list[1:]
                    theta_list.append(theta)

                # theta_average = 0
                # for angle in theta_list:
                #     theta_average += angle
                # theta_average /= len(theta_list)

            print("theta_average = " + str(theta_average))
            cv2.imshow("PHT", frame_copy)

            end = time.time()
            # print("Frame time = " + str(end - start))
            print("\n")

            ######### here the lane detection ends ###########

            self.outP_lane.send(theta_average)   # sends the results of the detection back
