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
        self.writer = cv2.VideoWriter('PHT_Video.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15, (640, 480))
        self.list_of_frames = []

        self.width = 640
        self.height = 480
        self.x_top = 270  # Coordinates of the polygon we use for creating the Homography matrix
        self.y_left_top = 80
        self.y_right_top = 560
        self.input_coordinates_IPM = np.array(
            [[0, 480], [self.y_left_top, self.x_top], [self.y_right_top, self.x_top], [640, 480]], dtype=np.float32)
        self.output_coordinates_IPM = np.array([[199, 36], [417, 0], [439, 444], [205, 410]],
                                               dtype=np.float32)  # Output coordinates calculated manually in our flat real word plane of the road
        self.matrix_IPM = cv2.getPerspectiveTransform(self.input_coordinates_IPM, self.output_coordinates_IPM)
        ''' ================================================================================================================================ '''


    # preprocess our frame_ROI
    def preProcess(self, frame_ROI):
        frame_ROI_gray = cv2.cvtColor(frame_ROI, cv2.COLOR_BGR2GRAY)
        frame_ROI_blurred = cv2.GaussianBlur(frame_ROI_gray, (11, 11), 0)
        # cv2.imshow("ROI_Blurred", frame_ROI_blurred)

        frame_ROI_preprocessed = cv2.Canny(frame_ROI_blurred, 30, 255)

        return frame_ROI_preprocessed

    def get_intercept_theta_line(self, line, frame_ROI):
        height_ROI = frame_ROI.shape[0]
        # get cv2 coordinates of our line
        y1_cv, x1_cv, y2_cv, x2_cv = line[0]
        # cv2.line(frame_ROI, (y1_cv, x1_cv), (y2_cv, x2_cv), (0, 0, 255), 2)

        # conversion to usual XoY coordinate system
        x1 = y1_cv
        x2 = y2_cv
        y1 = abs(x1_cv - height_ROI)
        y2 = abs(x2_cv - height_ROI)

        # get intercept and theta -> apply np.polyfit
        # y = slope * x + intercept_oY
        coefficients = np.polynomial.polynomial.polyfit((x1, x2), (y1, y2), 1)
        if coefficients is not None:
            # coefficients[1] = slope
            # coefficients[0] = intercept_oY
            theta = math.degrees(math.atan(coefficients[1]))
            # get intercept_oX  -> when y = 0;
            intercept_oX = int((-coefficients[0]) / coefficients[1])
            # print("theta = " + str(theta) + ";   intercept_oX = " + str(intercept_oX))
            return theta, intercept_oX

    def filter_line(self, theta, intercept_oX, width_ROI, y_cv_margin):

        margin_y_cv_left = int(width_ROI / 2) - y_cv_margin
        margin_y_cv_right = int(width_ROI / 2) + y_cv_margin

        if abs(theta) >= 35:    # if it's not horizontal
            # check by intercept_oX -> highest priority
            if -50 <= intercept_oX <= margin_y_cv_left:    # left line
                return 0
            if margin_y_cv_right <= intercept_oX <= width_ROI + 50:  # right line
                return 1
            # check by theta and intercept_oX
            if theta > 0:   # candidate left line
                if -50 <= intercept_oX <= margin_y_cv_right:
                    return 0
            if theta < 0:   # candidate right line
                if margin_y_cv_left <= intercept_oX <= width_ROI + 50:
                    return 1
        return -1

    def get_and_filter_lines(self, frame_ROI_preprocessed, frame_ROI):
        lines = cv2.HoughLinesP(frame_ROI_preprocessed, rho=1, theta=np.pi / 180, threshold=70, minLineLength=30,
                                         maxLineGap=70)
        left_lines = []
        right_lines = []

        width_ROI = frame_ROI.shape[1]
        y_cv_margin = 145  # margin wrt to vertical center of frame_ROI
        margin_y_cv_left = int(width_ROI / 2) + y_cv_margin
        margin_y_cv_right = int(width_ROI / 2) - y_cv_margin
        # draw lines for margin admitted
        cv2.line(frame_ROI, (int(width_ROI / 2) - y_cv_margin, 0), (int(width_ROI / 2) - y_cv_margin, frame_ROI.shape[0]), (0, 255, 0), 2)
        cv2.line(frame_ROI, (int(width_ROI / 2) + y_cv_margin, 0),
                 (int(width_ROI / 2) + y_cv_margin, frame_ROI.shape[0]), (0, 255, 0), 2)


        if lines is not None:
            for line in lines:
                theta, intercept_oX = self.get_intercept_theta_line(line, frame_ROI)
                line_code = self.filter_line(theta, intercept_oX, width_ROI, y_cv_margin)
                if line_code == 0:
                    y1_cv, x1_cv, y2_cv, x2_cv = line[0]
                    self.drawLane(line, frame_ROI, (0, 0, 255))
                    # cv2.line(frame_ROI, (y1_cv, x1_cv), (y2_cv, x2_cv), (0, 0, 255), 2)     # RED color -> left_line
                    left_lines.append(line)
                if line_code == 1:
                    y1_cv, x1_cv, y2_cv, x2_cv = line[0]
                    self.drawLane(line, frame_ROI, (255, 0 ,0))
                    # cv2.line(frame_ROI, (y1_cv, x1_cv), (y2_cv, x2_cv), (255, 0, 0), 2)     # BLUE color -> right_line
                    right_lines.append(line)

        return left_lines, right_lines

    def polyfit(self, lines, frame_ROI):
        # coordinates used for estimating our line
        x_points = []
        y_points = []

        for line in lines:
            y1_cv, x1_cv, y2_cv, x2_cv = line[0]    # coordinates in cv2 coordinate system
            # cv2.line(frame_ROI, (y1_cv, x1_cv), (y2_cv, x2_cv), ())

            # conversion to usual XoY coordinate system
            x1 = y1_cv
            x2 = y2_cv
            y1 = abs(x1_cv - self.x_top)
            y2 = abs(x2_cv - self.x_top)

            x_points.append(x1)
            x_points.append(x2)
            y_points.append(y1)
            y_points.append(y2)

        # get our estimated line
        coefficient = np.polynomial.polynomial.polyfit(x_points, y_points, 1)
        # print(str(coefficient[1]) + "*x + " + str(coefficient[0]))

        # expand our estimated line from bottom to the top of the ROI
        y1 = 0
        y2 = self.x_top
        x1 = int((y1 - coefficient[0]) / coefficient[1])
        x2 = int((y2 - coefficient[0]) / coefficient[1])

        # convert our estimated line from XoY in cv2 coordinate system
        y1_cv = x1
        y2_cv = x2
        x1_cv = abs(y1 - self.x_top)
        x2_cv = abs(y2 - self.x_top)

        cv2.line(frame_ROI, (y1_cv, x1_cv), (y2_cv, x2_cv), (0, 255, 0), 3)

        return coefficient  # return the coordinates of our estimated line and its line equation

    def both_lines_detected(self, left_line_coefficients, right_line_coefficients, frame_ROI):
        height_ROI = frame_ROI.shape[0]
        x_cv_theta = 0   # the x_cv2 coordinate where we intersect -> wrt to ROI

        # transform in XoY coordinate
        y_theta = abs(x_cv_theta - height_ROI)
        x_left_theta = int((y_theta - left_line_coefficients[0]) / left_line_coefficients[1])
        x_right_theta = int((y_theta - right_line_coefficients[0]) / right_line_coefficients[1])

        # convert back to cv2 coordinate system
        y_cv_left_line = x_left_theta
        y_cv_right_line = x_right_theta

        cv2.line(frame_ROI, (y_cv_left_line, x_cv_theta), (y_cv_right_line, x_cv_theta), (200, 200, 200), 2)

        y_cv_vanishing_point = int((y_cv_right_line + y_cv_left_line) / 2)
        # print(y_cv_vanishing_point - y_cv_right_line)
        cv2.line(frame_ROI, (int(frame_ROI.shape[1] / 2) - 25, frame_ROI.shape[0]), (y_cv_vanishing_point, x_cv_theta), (232, 32, 1), 5)

        return y_cv_vanishing_point, x_cv_theta

    def only_one_line_detected(self, line_coefficients, frame_ROI, is_left_line=True):
        height_ROI = frame_ROI.shape[0]
        offset_center_road = 190    # experimental value
        x_cv_theta = -50  # the x_cv2 coordinate where we intersect -> wrt to ROI

        # transform in XoY coordinate
        y_theta = abs(x_cv_theta - height_ROI)
        x_theta = int((y_theta - line_coefficients[0]) / line_coefficients[1])

        y_cv_line = x_theta
        y_cv_vanishing_point = x_theta

        if is_left_line:
            y_cv_vanishing_point += offset_center_road
        else:
            y_cv_vanishing_point -= offset_center_road

        cv2.line(frame_ROI, (y_cv_vanishing_point, x_cv_theta), (y_cv_line, x_cv_theta), (200, 200, 200), 2)
        cv2.line(frame_ROI, (int(frame_ROI.shape[1] / 2) - 25, frame_ROI.shape[0]), (y_cv_vanishing_point, x_cv_theta),
                 (232, 32, 1), 5)

        return y_cv_vanishing_point, x_cv_theta

        pass

    def get_theta(self, frame_ROI_preprocessed, frame_ROI):
        left_lines, right_lines = self.get_and_filter_lines(frame_ROI_preprocessed, frame_ROI)
        found_line = False
        if left_lines and right_lines:
            # print("right and left")
            found_line = True
            left_line_coefficients = self.polyfit(left_lines, frame_ROI)
            right_line_coefficients = self.polyfit(right_lines, frame_ROI)
            y_cv_vanishing_point, x_cv_theta = self.both_lines_detected(left_line_coefficients, right_line_coefficients, frame_ROI)

        else:
            if right_lines:
                found_line = True
                right_line_coefficients = self.polyfit(right_lines, frame_ROI)
                y_cv_vanishing_point, x_cv_theta = self.only_one_line_detected(right_line_coefficients, frame_ROI, is_left_line=False)
            else:
                if left_lines:
                    found_line = True
                    left_line_coefficients = self.polyfit(left_lines, frame_ROI)
                    y_cv_vanishing_point, x_cv_theta = self.only_one_line_detected(left_line_coefficients, frame_ROI, is_left_line=True)

        if found_line:
            x_cv_center = frame_ROI.shape[0]
            y_cv_center = int(frame_ROI.shape[1] / 2) - 25  # camera lasata in partea stanga

            theta = math.degrees(math.atan((y_cv_center - y_cv_vanishing_point) / (x_cv_center - x_cv_theta)))
            return theta
        else:
            return -1000

    def drawLane(self, line, image, color_line):
        y1, x1, y2, x2 = line[0]
        radius = 10
        color_left_most_point = (0, 255, 0)
        color_right_most_point = (255, 0, 0)
        cv2.circle(image, (y1, x1), radius, color_left_most_point, 1)
        cv2.circle(image, (y2, x2), radius, color_right_most_point, 1)
        cv2.line(image, (y1, x1), (y2, x2), color_line, 2)


    def run(self):

        theta_average = 0

        while True:

            # waits for the preprocessed image and gets it
            frame = self.inP_img.recv()

            start = time.time()

            # choosing our ROI
            # cv2.line(frame, (0, self.x_top - 5), (640, self.x_top - 5), (0, 0, 255), 2)
            frame_ROI = frame[self.x_top:, :]

            # preprocessing our ROI of the frame
            frame_ROI_preprocessed = self.preProcess(frame_ROI)
            theta = self.get_theta(frame_ROI_preprocessed, frame_ROI)
            if theta != -1000:  # we didn't detect any line
                theta_average = (0.6 * theta_average + 0.4 * theta) * 0.8
            if theta_average > 23:
                theta_average = 23
            if theta_average < -23:
                theta_average = -23
            print("theta_average = {}".format(theta_average))

            cv2.imshow("ROI", frame_ROI)
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)

            end = time.time()
            print("Lane detection time: {}".format(end - start))

            ######### here the lane detection ends ###########

            self.outP_lane.send(-theta_average)   # sends the results of the detection back

# class LaneDetectionThread(Thread):
#     def __init__(self, inP_img, outP_lane, show_lane=False):
#         """
#
#         :param inP_img: receives a preprocessed image from a pipe
#         :param outP_lane: outputs the result of the detection through the pipe
#         """
#         super(LaneDetectionThread, self).__init__()
#         self.inP_img = inP_img
#         self.outP_lane = outP_lane
#         self.show_lane = show_lane
#         self.writer = cv2.VideoWriter('PHT_Video.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15, (640, 480))
#         self.list_of_frames = []
#
#     def getTheta(self, lane, isRight):
#         x1, y1, x2, y2 = lane
#         slope = float((x1 - x2) / (y2 - y1))
#
#         theta = math.atan(slope)
#         theta = theta * 15
#
#         if theta >= 23:
#             theta = 22.5
#         else:
#             if theta <= -23:
#                 theta = -22.5
#         return theta
#
#
#     def preprocessing(self, frame):
#         frame_copy = frame[int(int(frame.shape[0] * 0.65)):, :]
#
#         frame_grayscale = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
#
#         frame_blurred = cv2.GaussianBlur(frame_grayscale, (7, 7), cv2.BORDER_DEFAULT)
#
#         frame_edge = cv2.Canny(frame_blurred, 100, 200)
#
#         return frame_edge
#
#     def averagelanes(self, lanes):
#         x1_f = 0
#         y1_f = 0
#         x2_f = 0
#         y2_f = 0
#
#         for lane in lanes:
#             x1, y1, x2, y2 = lane
#
#             x1_f += x1
#             y1_f += y1
#
#             x2_f += x2
#             y2_f += y2
#
#         x1_f = int(x1_f / len(lanes))
#         y1_f = int(y1_f / len(lanes))
#         x2_f = int(x2_f / len(lanes))
#         y2_f = int(y2_f / len(lanes))
#         return [x1_f, y1_f, x2_f, y2_f]
#
#     def draw_lane(self, image, lane, color):
#         x1, y1, x2, y2 = lane
#         cv2.line(image, (x1, y1), (x2, y2), color, 5)
#
#     def general_equation_form(self, x1, y1, x2, y2):
#         A = (y1 - y2)
#         B = (x1 - x2)
#         C = (x1 * y2 - x2 * y1)
#         return A, B, -C
#
#     def intersection(self, L1, L2):
#         D = L1[0] * L2[1] - L1[1] * L2[0]
#         Dx = L1[2] * L2[1] - L1[1] * L2[2]
#         Dy = L1[0] * L2[2] - L1[2] * L2[0]
#         if D != 0:
#             x = Dx / D
#             y = Dy / D
#             return x, -y
#         else:
#             return False
#
#     def angle(self, left_lane, right_lane, width, height):
#         # print("height = " + str(height))
#         # print("width = " + str(width))
#         x1l, y1l, x2l, y2l = left_lane
#         y1l = -(y1l - height)
#         # print("y1l = " + str(y1l) + ", x1l = " + str(x1l))
#         y2l = -(y2l - height)
#         # print("y2l = " + str(y2l) + ", x2l = " + str(x2l))
#
#         x1r, y1r, x2r, y2r = right_lane
#         # print("y2r before = " + str(y2r))
#         y1r = -(y1r - height)
#         # print("y1r = " + str(y1r) + ", x1r = " + str(x1r))
#         y2r = -(y2r - height)
#         # print("y2r = " + str(y2r) + ", x2r = " + str(x2r))
#
#         L1 = self.general_equation_form(x1l, y1l, x2l, y2l)  # L1 -> left lane
#         L2 = self.general_equation_form(x1r, y1r, x2r, y2r)  # l2 -> right lane
#
#         x0, y0 = self.intersection(L1, L2)
#
#         # print("x0 = " + str(x0) + ", y0 = " + str(y0))
#         # print("x0 - width / 2 = " + str((x0 - width / 2)))
#         # print("tan = " + str(float((x0 - width / 2) / y0)))
#         theta = math.atan(float((x0 - width / 2) / y0))
#         theta = theta * 30
#         if theta >= 23:
#             theta = 22.5
#
#         if theta <= -23:
#             theta = -22.5
#         return theta
#
#     def run(self):
#         start = time.time()
#         theta_list = []
#         theta_average = 0.0
#
#         while True:
#
#             # waits for the preprocessed image and gets it
#             frame = self.inP_img.recv()
#
#             ######### here takes place the lane detection ###########
#             frame_edge = self.preprocessing(frame)
#
#             lines = cv2.HoughLinesP(frame_edge, rho=1, theta=np.pi / 180, threshold=60, minLineLength=10,
#                                     maxLineGap=100)
#             left_lanes = []
#             right_lanes = []
#             frame_copy = frame[int(frame.shape[0] * 0.65):, :]  # used for displaying
#
#             if lines is not None:
#                 # classify lanes based on their slope
#                 for line in lines:
#                     x1, y1, x2, y2 = line[0]
#                     if x1 != x2 and y1 != y2:
#                         slope = float((x1 - x2) / (y2 - y1))
#                         angle = math.atan(slope)
#                         if not(angle > 80 or angle < -80):
#                             if slope < 0.0:
#                                 left_lanes.append([x1, y1, x2, y2])
#                             else:
#                                 right_lanes.append([x1, y1, x2, y2])
#
#                 if len(left_lanes) and len(right_lanes):  # identify both lanes
#
#                     left_lane = self.averagelanes(left_lanes)
#                     self.draw_lane(frame_copy, left_lane, (255, 0, 0))
#                     # self.draw_lane(frame, left_lane, (255, 0, 0))
#
#                     right_lane = self.averagelanes(right_lanes)
#                     self.draw_lane(frame_copy, right_lane, (0, 0, 255))
#                     # self.draw_lane(frame, right_lane, (0, 0, 255))
#
#                     theta = self.angle(left_lane, right_lane, frame_copy.shape[1], frame_copy.shape[0])
#                 else:
#                     if len(left_lanes):  # identify only left_lanes
#                         left_lane = self.averagelanes(left_lanes)
#                         theta = self.getTheta(left_lane, False)
#                         self.draw_lane(frame_copy, left_lane, (255, 0, 0))
#                         # self.draw_lane(frame, left_lane, (255, 0, 0))
#                     else:
#                         if len(right_lanes):
#                             # print("right")
#                             right_lane = self.averagelanes(right_lanes)
#                             theta = self.getTheta(right_lane, True)
#                             self.draw_lane(frame_copy, right_lane, (0, 0, 255))
#                             # self.draw_lane(frame, left_lane, (255, 0, 0))
#                         else:
#                             if len(theta_list) != 0:
#                                 theta = theta_list[len(theta_list)-1]
#                             else:
#                                 theta = 0
#                 # print(str(i) + ": theta = " + str(theta))
#
#                 if len(theta_list) != 5:
#                     theta_average = theta
#                     theta_list.append(theta)
#                 else:
#                     theta_average = theta
#                     theta_list = theta_list[1:]
#                     theta_list.append(theta)
#
#                 # theta_average = 0
#                 # for angle in theta_list:
#                 #     theta_average += angle
#                 # theta_average /= len(theta_list)
#
#             # self.writer.write(frame)
#             # addition = np.zeros((640, 60, 3), dtype=np.uint8)
#             # cv2.putText(addition, str(theta_average), (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
#             frame = cv2.resize(frame, (640, 480))
#             # frame = cv2.hconcat(addition, frame)
#             self.list_of_frames.append(frame)
#             print("theta_average = " + str(theta_average))
#
#             #cv2.imshow("PHT", frame_copy)
#
#             ########### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! here commented out !!!!!!!!!!!!!!!!!!!!!!!!!!!!
#             """cv2.imshow("PHT", frame_copy)
#             cv2.imshow("Frame", frame)"""
#
#             # print("Frame time = " + str(end - start))
#             #print("\n")
#
#             ######### here the lane detection ends ###########
#
#             self.outP_lane.send(theta_average)   # sends the results of the detection back