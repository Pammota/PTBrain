import time

import numpy as np
from helpers import *
from ImageAquisitionThread import ImageAquisitionThread
import cv2

def aggregate(gray, detectors, n_agg, sizes, n_neighb):

    rects_agg = []

    for detector, size, neighb in zip(detectors, sizes, n_neighb):
        rects = detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=neighb,
                    minSize=size, flags=cv2.CASCADE_SCALE_IMAGE)
        rects_agg.append(rects)

    groups = []

    for rects in rects_agg:
        group_idx = None
        for rect in rects:
            for i, group in enumerate(groups):
                for other_rect in group:
                    iou = get_IoU_cascade(rect, other_rect)
                    if iou > 0.25:
                        group.append(rect)
                        group_idx = i
                        break
                if group_idx is not None:
                    continue
            if group_idx is not None:
                continue
            else:
                groups.append([rect])

    x1s = [np.median([rect[0] for rect in group]) for group in groups if len(group) >= n_agg]
    y1s = [np.median([rect[1] for rect in group]) for group in groups if len(group) >= n_agg]
    ws = [np.median([rect[2] for rect in group]) for group in groups if len(group) >= n_agg]
    hs = [np.median([rect[3] for rect in group]) for group in groups if len(group) >= n_agg]

    new_rects = [(x1, y1, w, h) for (x1, y1, w, h) in zip(x1s, y1s, ws, hs)]

    return new_rects

class HaarCascadeClassifier():
    def __init__(self):
        self.detectors = [cv2.CascadeClassifier("cascades/trained_cascade_32_975_1_1000/cascade.xml"),
                          cv2.CascadeClassifier("cascades/trained_cascade_24_975_1/cascade.xml"),
                          cv2.CascadeClassifier("cascades/trained_cascade_32_985_35_1000/cascade.xml")]
        self.sizes = [(32, 32), (24, 24), (32, 32)]
        self.n_neighb = [18, 3, 28]

    def wait_pedestrian(self, cameraThread):

        on_screen = True

        cameraThread.start()
        time.sleep(0.5)

        start = time.time()

        while time.time() - start > 600:
            frame = cameraThread.frame
            frame = cv2.resize(frame, (320, 240))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            rects = aggregate(gray, self.detectors, 2, self.sizes, self.n_neighb)

            for rect in rects:
                x, y, w, h, = rect
                cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 0, 255), 2)
            cv2.imshow("haarcascade", frame)
            cv2.waitKey(0)


if __name__ == "__main__":
    hc = HaarCascadeClassifier()
    hc.wait_pedestrian(ImageAquisitionThread())
