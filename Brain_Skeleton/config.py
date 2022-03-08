
RUN_MODE = "NO_DETECTION"

CLASSES = {
    1: {"LABEL": "Forward", "COLOR": (0, 0, 255)},
    2: {"LABEL": "Forbidden", "COLOR": (255, 0, 0)},
    3: {"LABEL": "Parking", "COLOR": (0, 0, 255)},
    4: {"LABEL": "Sem_Yellow", "COLOR": (255, 0, 0)},
    5: {"LABEL": "Sem_Red", "COLOR": (255, 0, 0)},
    6: {"LABEL": "Sem_No_Light", "COLOR": (255, 255, 255)},
    7: {"LABEL": "Sem_Green", "COLOR": (0, 255, 0)},
    8: {"LABEL": "Priority", "COLOR": (0, 255, 0)},
    9: {"LABEL": "Crosswalk", "COLOR": (0, 0, 255)},
    10: {"LABEL": "Stop", "COLOR": (255, 0, 0)},

}

DETECTION_SCORE_THRESHOLD = 0
IOU_THRESHOLD = 0.1

CLASSIFIER_INPUT_SHAPE = (224, 224)
DETECTOR_INPUT_SHAPE = (320, 320)
