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
    1: {"LABEL": "forward", "COLOR": (0, 0, 255)},
    2: {"LABEL": "forbidden", "COLOR": (255, 0, 0)},
    3: {"LABEL": "parking", "COLOR": (0, 0, 255)},
    4: {"LABEL": "sem_yellow", "COLOR": (255, 0, 0)},
    5: {"LABEL": "sem_red", "COLOR": (255, 0, 0)},
    6: {"LABEL": "sem_green", "COLOR": (0, 255, 0)},
    7: {"LABEL": "priority", "COLOR": (0, 255, 0)},
    8: {"LABEL": "crosswalk", "COLOR": (0, 0, 255)},
    9: {"LABEL": "stop", "COLOR": (255, 0, 0)}
}

DETECTION_SCORE_THRESHOLD = 0.1
IOU_THRESHOLD = 0.1

CLASSIFIER_INPUT_SHAPE = (224, 224)
DETECTOR_INPUT_SHAPE = (320, 320)
