import time

import serial
from threading import Thread

class ReadThread(Thread):
    # ===================================== INIT =========================================
    def __init__(self):

        super(ReadThread, self).__init__()

        devFile = '/dev/ttyACM1'

        # comm init
        self.serialCom = serial.Serial(devFile, 9600)
        self.serialCom.flushInput()
        self.serialCom.flushOutput()

        self.serialCom = self.serialCom
        self.is_active = True

    # ===================================== RUN ==========================================
    def run(self):

        while self.is_active:
            rec_data = self.serialCom.read(20)
            time.sleep(0.03)
            data_left = self.serialCom.inWaiting()
            rec_data += self.serialCom.read(data_left)
            print(rec_data)
