from TFLiteModel import TFLiteModel
from imutils.paths import list_images
import config
import time
import cv2

model = TFLiteModel("models/efficientdet_lite3_512_ptq_edgetpu.tflite", input_shape=config.DETECTOR_INPUT_SHAPE,
                    quantized_input=False, quantized_output=False)

imagePaths = list(list_images("frameAnnotations-DataLog02142012_001_external_camera.avi_annotations"))[:50]
median_time = 0

for path in imagePaths:

    start = time.time()
    image = cv2.imread(path)
    prediction = model.predict(image)
    end = time.time()

    median_time += end-start
    print("Inference time: {}s".format(end - start))

print("Median inference time: {}s".format(median_time/len(imagePaths)))

