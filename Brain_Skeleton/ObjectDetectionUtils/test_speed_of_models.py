from Brain_Skeleton.ObjectDetectionUtils.TFLiteModel import TFLiteModel
from imutils.paths import list_images
import config
import time
import cv2

model = TFLiteModel("models/mobilenet_cityscapes_frozen_normalized.tflite", input_shape=config.DETECTOR_INPUT_SHAPE,
                    quantized_input=False, quantized_output=False)

imagePaths = list(list_images("jena"))[:50]
median_time = 0

for path in imagePaths:

    start = time.time()
    image = cv2.imread(path)
    prediction = model.predict(image)
    end = time.time()

    median_time += end-start
    print("Inference time: {}s".format(end - start))

print("Median inference time: {}s".format(median_time/len(imagePaths)))

