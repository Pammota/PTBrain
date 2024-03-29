import numpy as np
import config

def accept_box(boxes, box, tolerance):
    """
    Eliminate duplicate bounding boxes.
    """
    box_index = boxes.index(box)

    for idx in range(box_index):
        other_box = boxes[idx]
        if abs(center(other_box, "x") - center(box, "x")) < tolerance and abs(
                center(other_box, "y") - center(box, "y")) < tolerance:
            return False

    return True

def center(box, coord_type):
    """
    Get center of the bounding box.
    """
    coord_type_idx = 1 if coord_type == "x" else 0
    return (box[coord_type_idx] + box[coord_type_idx + 2]) / 2


def get_IoU(a, b):

    try:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
    except TypeError:
        return 0.0

    cy1 = max(ay1, by1)
    cy2 = min(ay2, by2)
    cx1 = max(ax1, bx1)
    cx2 = min(ax2, bx2)

    intersection = max(0, cx2 - cx1 + 1) * max(0, cy2 - cy1 + 1)

    union = float(area(a) + area(b) - intersection)
    return float(intersection / union)


def area(box):
    x1, y1, x2, y2 = box
    return (y2 - y1 + 1) * (x2 - x1 + 1)


def NM_Supress(boxes, labels, scores):

    partitions_boxes = {}
    partitions_scores = {}

    for box, label, score in zip(boxes, labels, scores):
        same_boxes = partitions_boxes.get(label, [])
        same_boxes.append(box)
        partitions_boxes[label] = same_boxes

        same_scores = partitions_scores.get(label, [])
        same_scores.append(score)
        partitions_scores[label] = same_scores

    boxes_supressed = []
    labels_supressed = []
    scores_supressed = []

    for k, same_boxes in partitions_boxes.items():
        same_scores = partitions_scores[k]

        did_eliminate = True
        while did_eliminate:
            did_eliminate = False
            idx = 0
            while idx < len(same_boxes):
                box = same_boxes[idx]
                jdx = idx + 1
                while jdx < len(same_boxes):
                    box2 = same_boxes[jdx]
                    IoU = get_IoU(box, box2)
                    if IoU > config.IOU_THRESHOLD:
                        did_eliminate = True
                        if area(box) > area(box2):
                            same_boxes.pop(jdx)
                            same_scores.pop(jdx)
                            jdx -= 1
                        else:
                            same_boxes.pop(idx)
                            same_scores.pop(idx)
                            idx -= 1
                            break
                    jdx += 1
                idx += 1
        boxes_supressed += same_boxes
        scores_supressed += same_scores
        for idx in range(len(same_boxes)):
            labels_supressed.append(k)

    return boxes_supressed, labels_supressed, scores_supressed