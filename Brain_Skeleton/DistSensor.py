from threading import Thread
import serial
import time

class DistanceSensor(Thread):
    def __init__(self, devFile):
        super(DistanceSensor, self).__init__()

        self.serialComDS = serial.Serial(devFile, 9600)
        self.serialComDS.flushInput()
        self.serialComDS.flushOutput()
        self.current_distance = 100

    def get_current_distance(self):
        return self.current_distance

    def get_distance_info(self):
        rec_data = self.serialComDS.read(10)
        data_left = self.serialComDS.inWaiting()
        rec_data += self.serialComDS.read(data_left)
        #print(rec_data)

        rec_numbers = [int(s) for s in rec_data.split() if s.isdigit()]

        try:
            rec_number = rec_numbers[0]
        except IndexError:
            rec_number = 3
        print(rec_number)
        return rec_number

    def run(self):
        self.current_distance = self.get_distance_info()

        while True:
            time.sleep(0.03)
            self.current_distance = self.get_distance_info()