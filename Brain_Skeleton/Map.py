import math

import cv2
import numpy as np

class Map:
    def __init__(self, px_per_m=100):
        self.px_per_m = px_per_m

        self.physical_map = np.zeros((340, 430))
        self.orientation_visualization = np.zeros((340, 100))

        self.prev_x = 0
        self.prev_y = 53

        self.h = 430
        self.w = 340

    def get_map(self, include_orientation=True):
        if include_orientation:
            return np.hstack((self.physical_map, self.orientation_visualization))
        else:
            return self.physical_map

    def update_map(self, x, y, yaw):
        x = int(x*self.px_per_m)
        y = int(y*self.px_per_m)

        x = max(0, x)
        x = min(x, self.h)

        x = self.h - x

        y = max(0, y)
        y = min(y, self.w)

        self.physical_map = cv2.line(self.physical_map, (self.prev_x, self.prev_y), (x, y),
                                     (255, 255, 255), 2)

        self.prev_x = x
        self.prev_y = y

        self.orientation_visualization = np.zeros((340, 200))

        arr_len = 90
        (arr_x1, arr_y1) = (50, 50)
        arr_x2 = int(arr_x1 + arr_len * math.cos(math.radians(yaw)))
        arr_y2 = int(arr_y1 + arr_len * math.sin(math.radians(yaw)))

        offset_x = (arr_x2 - arr_x1) // 2
        offset_y = (arr_y2 - arr_y1) // 2

        arr_x1 -= offset_x
        arr_x2 -= offset_x
        arr_y1 -= offset_y
        arr_y2 -= offset_y

        self.orientation_visualization = cv2.arrowedLine(self.orientation_visualization,
                                                         (arr_x1, arr_y1), (arr_x2, arr_y2),
                                                         (255, 255, 255), 2)

