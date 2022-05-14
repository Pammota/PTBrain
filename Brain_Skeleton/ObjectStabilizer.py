import numpy as np
import copy
from helpers import *

class ObjectStabilizer:
    def __init__(self, num_frames=5, threshold=0.5, IoU_thresh = 0.4):

        self.IoU_thresh = IoU_thresh

        self.num_frames = num_frames
        if threshold <= 0 or threshold >= 1:
            threshold = 0.5
        self.threshold = int(num_frames * threshold)

        self.objects = []

    def add_frame(self, boxes, labels, scores):
        ids = np.zeros_like(labels) - 1
        i = 0
        while i < len(self.objects):  # Step 1: for each entity, check if it is continued in the current frame
            exists, ent_box, ent_label, ent_score = self.get_object_data(i)

            disappears = True
            for j in range(len(labels)):
                if labels[j] != ent_label or ids[j] >= 0:
                    continue

                IoU = get_IoU(ent_box, boxes[j])
                if IoU > self.IoU_thresh:  #is the same object
                    disappears = False
                    self.objects[i]["boxes"].append(boxes[j])
                    self.objects[i]["scores"] = scores[j]
                    ids[j] = i

            if disappears:
                self.objects[i]["boxes"].append(None)

            if len(self.objects[i]["boxes"]) > self.num_frames:
                self.objects[i]["boxes"] = self.objects[i]["boxes"][1:]

            if disappears and not exists and len(self.objects[i]["boxes"]) == self.num_frames:
                # If it does not continue, and it did not exist before, then it certainly is extinct
                # (even if a new instance appears, it would still be extinct, except if its not yet all built)
                self.objects.pop(i)
                i -= 1
            i += 1

        for i in range(len(labels)):  # for all the objects in the current frame that were not associated, we create a new entity
            if ids[i] < 0:
                ids[i] = len(self.objects)
                self.objects.append({"label": labels[i], "boxes": [boxes[i]], "score": scores[i]})

        return ids

    def get_object_data(self, obj_idx):  # here will be the position stabilisation

        obj = self.objects[obj_idx]
        b_box = None
        num_appearances = 0
        boxes = copy.deepcopy(obj.get("boxes"))
        boxes.reverse()
        for box in boxes:
            if box is not None:
                num_appearances += 1
                if b_box is None:
                    b_box = box
        exists = num_appearances > self.threshold
        return exists, b_box, obj.get("label"), obj.get("score")
