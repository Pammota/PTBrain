import time

import cv2
import numpy as np
import math
from Controller import Controller

class PathGenerator:

    def generate_circle_points(self, r=None, d=None, x_c=None, y_c=None, alpha_min=None, alpha_max=None):
        points = []
        alpha = 2 * math.asin(float(d / (2 * r)))
        k = 0
        while k * alpha + alpha_min <= alpha_max:
            x = r * math.cos((k * alpha + alpha_min) % alpha_max)
            y = r * math.sin((k * alpha + alpha_min) % alpha_max)
            points.append((x + x_c, y + y_c))
            k += 1
        return points

    def generate_line_points(self, x1=None, y1=None, x2=None, y2=None, n=None):
        if x1 != x2:
            x_points = np.linspace(x1, x2, num=n, endpoint=True)
        else:
            x_points = np.zeros(n) + x1
        if y1 != y2:
            y_points = np.linspace(y1, y2, num=n, endpoint=True)
        else:
            y_points = np.zeros(n) + y1

        points = []
        for x, y in zip(x_points, y_points):
            points.append((x, y))
        return points


class Map:

    def __init__(self, size_pixel=None, size_cm=None, ref_points=None):
        self.size_pixel = size_pixel
        self.pixel_resolution = float(size_cm / size_pixel)
        self.map = np.zeros((self.size_pixel, self.size_pixel), np.uint8)

        # draw ref points
        for point in ref_points:
            x, y = self.get_point_map(point)
            # x_cv = int(abs(y / self.pixel_resolution - self.size_pixel))
            # y_cv = int(x / self.pixel_resolution)
            if 0 <= x < self.size_pixel and 0 <= y < self.size_pixel:
                self.map[y][x] = 255

    def get_point_map(self, p):
        # self.map[int(abs(y / self.pixel_resolution - self.size_pixel))][int(x / self.pixel_resolution)] = 255
        x, y = p
        return int(x / self.pixel_resolution), int(abs(y / self.pixel_resolution - self.size_pixel))

    def draw_line(self, p1, p2):
        p1 = self.get_point_map(p1)
        p2 = self.get_point_map(p2)
        x1, y1 = p1
        x2, y2 = p2
        cv2.line(self.map, (x1, y1), (x2, y2), 255, 1)

class PathTracking:

    def __init__(self, outP_com, map=None, ref_points=None, size_pixel=None, size_cm=None,
                 x_car=None, y_car=None, theta_yaw_map=None, yaw=None, v=None, dt=None,
                 ref_thresh=None, final_thresh=None, end_point=None,
                 imu_tracker=None, L=None
                 ):
        # info about the map
        self.map = Map(size_pixel=size_pixel, size_cm=size_cm, ref_points=ref_points)
        self.ref_points = ref_points

        #misc
        self.outP_com = outP_com

        # info about the car
        self.x_car = x_car
        self.y_car = y_car
        # print("x_car = {}, y_car = {}".format(self.x_car, self.y_car))
        self.theta_car = theta_yaw_map  # heading angle of the car wrt ot the map cs
        print("theta_yaw_map = {}".format(theta_yaw_map))
        print("yaw in trigo cs = {}".format(self.yaw_to_trigo(yaw)))
        self.theta_offset = (self.yaw_to_trigo(yaw) - theta_yaw_map + 360) % 360
        self.v = v
        self.dt = dt
        self.L = L

        # destination info
        self.x_end, self.y_end = end_point
        self.ref_thresh = ref_thresh
        self.final_thresh = final_thresh

        # IMU
        self.imu_tracker = imu_tracker

    def get_ref_point(self):
        point_ref_min = None
        min_dist = 100000000

        # print(self.ref_points)

        for point_ref in self.ref_points:
            # print(point_ref)
            x_ref, y_ref = point_ref
            # print("x_ref = {}. y_ref = {}".format(x_ref, y_ref))
            slope_car = math.tan(math.radians(self.theta_car))
            # print("slope_car = {}".format(slope_car))
            # print("theta_perp_car = {} degrees".format((self.theta_car + 90) % 360))
            slope_perp_car = math.tan(math.radians((self.theta_car + 90) % 360))
            # print("slope_perp_car = {}".format(slope_perp_car))
            # print("eq_perp_car:")
            eq_perp_car = self.get_line_eq(slope_perp_car, (self.x_car, self.y_car))
            # print("eq_ref:")
            eq_ref = self.get_line_eq(slope_car, (x_ref, y_ref))
            x_int, y_int = self.line_intersection(eq_perp_car, eq_ref)
            # print("x_int = {}, y_int = {}".format(x_int, y_int))

            if y_ref - y_int >= 0:
                d = self.distance((x_ref, y_ref), (self.x_car, self.y_car))
                # print("d = {} cm".format(d))
                if d >= self.ref_thresh and d <= min_dist:
                    # print("FOUND REFERENCE POINT TO FOLLOW")
                    point_ref_min = point_ref
                    min_dist = d

        return point_ref_min

    def yaw_to_trigo(self, yaw):
        return 360 - (yaw + 270) % 360

    def distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def line_intersection(self, eq1, eq2):
        # y = m*x + c
        m1, c1 = eq1
        m2, c2 = eq2
        x = (c1 - c2) / (m2 - m1)
        y = m1 * x + c1
        return x, y

    def get_line_eq(self, slope, point):
        x, y = point
        c = y - slope * x
        # print("y = {} * x + {}".format(slope, c))
        return (slope, c)

    def get_theta_ref(self, p_ref):
        x_ref, y_ref = p_ref

        if x_ref == self.x_car and y_ref > self.y_car:
            return 90
        if x_ref == self.x_car and y_ref < self.y_car:
            return 270

        if y_ref == self.y_car and x_ref < y_ref:
            return 180
        if y_ref == self.y_car and x_ref > y_ref:
            return 0

        if x_ref < self.x_car and y_ref > self.y_car:
            return 180 + math.degrees(math.atan((y_ref - self.y_car) / (x_ref - self.x_car)))
        if x_ref > self.x_car and y_ref > self.y_car:
            return math.degrees(math.atan((y_ref - self.y_car) / (x_ref - self.x_car)))
        if x_ref < self.x_car and y_ref < self.y_car:
            return 180 + math.degrees(math.atan((y_ref - self.y_car) / (x_ref - self.x_car)))
        return 360 + math.degrees(math.atan((y_ref - self.y_car) / (x_ref - self.x_car)))



    def run(self):

        speed_command = Controller.getSpeedCommand(self.v)

        # while self.distance((self.x_car, self.y_car), (self.x_end, self.y_end)) >= self.final_thresh:
        #     print("final_distance = {} cm".format(self.distance((self.x_car, self.y_car), (self.x_end, self.y_end))))
        #     print("x_car = {}, y_car = {}".format(self.x_car, self.y_car))
        #     print("heading_car = {} degree".format(self.theta_car))
        #     point_ref = self.get_ref_point()
        #     x_ref, y_ref = point_ref
        #
        #     theta_ref = (math.degrees(math.atan((y_ref - self.y_car) / (x_ref - self.x_car))) + 360) % 360
        #     # print("theta_ref = {} degree".format(theta_ref))
        #     if x_ref < self.x_car:
        #         theta_ref = (theta_ref + 180) % 360
        #     print("theta_ref = {} degree".format(theta_ref))
        #     steering_angle = int(theta_ref - self.theta_car)  # data goes to the brain
        #     if steering_angle > 23:
        #         steering_angle = 23
        #     if steering_angle < -23:
        #         steering_angle = -23
        #     print("Steering angle = {}".format(-steering_angle))
        #     angle_command = Controller.getAngleCommand(-steering_angle)
        #
        #     self.outP_com.send((angle_command, speed_command))
        #
        #     self.map.draw_line((self.x_car, self.y_car), (x_ref, y_ref))
        #
        #     yaw = self.imu_tracker.yaw  # yaw data from IMU
        #     yaw = (self.yaw_to_trigo(yaw) - self.theta_offset + 360) % 360
        #     print("yaw transformed = {}".format(yaw))
        #     self.theta_car = yaw
        #
        #
        #     self.x_car = self.x_car + self.v * math.cos(math.radians(self.theta_car + 5)) * self.dt
        #     self.y_car = self.y_car + self.v * math.sin(math.radians(self.theta_car)) * self.dt
        #     time.sleep(self.dt)
        #     cv2.imshow("Map", self.map.map)
        #     cv2.waitKey(1)

        while self.distance((self.x_car, self.y_car), (self.x_end, self.y_end)) >= self.final_thresh:
            print("x_car = {}, y_car = {}".format(self.x_car, self.y_car))
            print("theta car = {}".format(self.theta_car))
            point_ref = self.get_ref_point()
            x_ref, y_ref = point_ref
            self.map.draw_line((self.x_car, self.y_car), (x_ref, y_ref))

            theta_ref = self.get_theta_ref(point_ref) % 360
            print("theta ref = {}".format(theta_ref))

            steering_angle = theta_ref - self.theta_car
            if steering_angle > 23:
                steering_angle = 23
            if steering_angle < -23:
                steering_angle = -23
            print("steering angle = {}".format(steering_angle))

            angle_command = Controller.getAngleCommand(-int(steering_angle))
            self.outP_com.send((angle_command, speed_command))

            self.x_car = self.x_car + self.v * math.cos(math.radians(self.theta_car + steering_angle)) * self.dt
            self.y_car = self.y_car + self.v * math.sin(math.radians(self.theta_car + steering_angle)) * self.dt
            self.theta_car = float(self.theta_car + self.v * math.tan(math.radians(steering_angle)) / self.L * self.dt)
            time.sleep(self.dt)
            cv2.imshow("Map", self.map.map)
            cv2.waitKey(1)





