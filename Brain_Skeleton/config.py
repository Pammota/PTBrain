ACTION_SPEED = 0
ACTION_ANGLE = 1
ACTION_STOP = 2
ACTION_RED = 3
ACTION_DIRECTION = 4
ACTION_CROSSWALK = 5
ACTION_PARKING = 6

RUN_MODE = "TFLITE"
PRINT_EXEC_TIMES = True

CLASSES = {
    1: {"LABEL": "forward", "COLOR": (255, 0, 0)},
    2: {"LABEL": "forbidden", "COLOR": (0, 0, 255)},
    3: {"LABEL": "parking", "COLOR": (255, 0, 0)},
    4: {"LABEL": "sem_yellow", "COLOR": (0, 0, 255)},
    5: {"LABEL": "sem_red", "COLOR": (0, 0, 255)},
    6: {"LABEL": "sem_green", "COLOR": (0, 255, 0)},
    7: {"LABEL": "priority", "COLOR": (0, 255, 0)},
    8: {"LABEL": "crosswalk", "COLOR": (255, 0, 0)},
    9: {"LABEL": "stop", "COLOR": (0, 0, 255)}
}

DETECTION_SCORE_THRESHOLD = 0.1
IOU_THRESHOLD = 0.1

CLASSIFIER_INPUT_SHAPE = (224, 224)
DETECTOR_INPUT_SHAPE = (320, 320)
