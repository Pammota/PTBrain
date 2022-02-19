from tensorflow.keras.applications.mobilenet_v3 import preprocess_input
import tflite_runtime.interpreter as tflite
import tensorflow as tf
import numpy as np
import cv2

class TFLiteModel:
    def __init__(self, model_path, input_shape=None, quantized_input=False, quantized_output=False):
        # configuring the interpreter
        if "_edgetpu" in model_path:
            self.interpreter = tflite.Interpreter(model_path=model_path,
                                 experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
        else:
            self.interpreter = tflite.Interpreter(model_path=model_path)
        if input_shape is not None:
            self.interpreter.resize_tensor_input(0, (1, input_shape[0], input_shape[1], 3))
        self.interpreter.allocate_tensors()

        # Get input and output tensors.
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.input_shape = self.input_details[0]['shape']

        # Set input and output quantization schemes, if applicable
        if quantized_input:
            self.input_scale, self.input_zero_point = self.input_details[0]['quantization']
        if quantized_output:
            self.output_scale, self.output_zero_point = self.output_details[0]['quantization']

        self.quantized_input = quantized_input
        self.quantized_output = quantized_output


    def predict(self, image):

        # prepare data for being inputted in model

        image_shape = (self.input_shape[1], self.input_shape[2])

        image = cv2.resize(image, image_shape)
        image = preprocess_input(image)
        image = np.expand_dims(image, axis=0)

        input_data = np.array(image, dtype=np.float32)

        if self.quantized_input:
            input_data = input_data / self.input_scale + self.input_zero_point
            input_data = np.array(input_data, dtype=np.int8)
        else:
            input_data = np.array(input_data, dtype=np.uint8)
        input_tensor = tf.convert_to_tensor(input_data)

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        self.interpreter.invoke()
        outputs = []
        try:
            # Pick outputs and dequantize them if needed
            for output_detail in self.output_details:
                output = None
                if self.quantized_output:
                    output = np.array(self.interpreter.get_tensor(output_detail['index']), dtype=np.int16)
                    output = (np.array(output[0], dtype="float32") - self.output_zero_point) * self.output_scale
                else:
                    output = np.array(self.interpreter.get_tensor(output_detail['index']), dtype=np.float32)
                outputs.append(output)
        except ValueError:
            print("[ERROR] Something went wrong. Detection unsuccessful.")

        return outputs
