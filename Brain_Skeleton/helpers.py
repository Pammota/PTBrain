
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
