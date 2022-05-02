# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import sys
sys.path.append('.')
import RTIMU
import config
import os.path
import time
import math
import threading

class imu(threading.Thread):
    def __init__(self): 
        threading.Thread.__init__(self)
        self.running = True

        self.SETTINGS_FILE = "RTIMULib"
        print("Using settings file " + self.SETTINGS_FILE + ".ini")
        if not os.path.exists(self.SETTINGS_FILE + ".ini"):
            print("Settings file does not exist, will be created")
        self.s = RTIMU.Settings(self.SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(self.s)
        print("IMU Name: " + self.imu.IMUName())
        if (not self.imu.IMUInit()):
            print("IMU Init Failed")
            self.stop()
            sys.exit(1)
        else:
            print("IMU Init Succeeded")
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        self.true_accelx = 0
        self.true_accely = 0
        self.true_accelz = 0
        self.yaw = 0

        self.poll_interval = self.imu.IMUGetPollInterval()
        print("Recommended Poll Interval: %dmS\n" % self.poll_interval)
        
    def run(self):
        while self.running == True:
            if self.imu.IMURead():
                self.data = self.imu.getIMUData()
                self.fusionPose = self.data["fusionPose"]
                self.accel = self.data["accel"]
                self.roll  = math.degrees(self.fusionPose[0])
                self.pitch = math.degrees(self.fusionPose[1])
                self.yaw   = math.degrees(self.fusionPose[2])
                self.accelx = self.accel[0]
                self.accely = self.accel[1]
                self.accelz = self.accel[2]

                #print("yaw = %f" % (self.yaw))

                #print("accelx = %f, accely = %f accelz = %f" %(self.accelx, self.accely, self.accelz))

                self.true_accelx = self.accelx + math.sin(math.radians(self.pitch))
                self.true_accely = self.accely - math.sin(math.radians(self.roll))
                self.true_accelz = self.accelz + math.sin(math.radians(self.pitch)) - math.sin(math.radians(self.roll))
                #print("accelx = %f, accely = %f, accelz = %f"%(self.true_accelx, self.true_accely, self.true_accelz))
                time.sleep(config.IMU_SAMPLING_FREQUENCY)

    def get_data(self):
        return self.true_accelx, self.true_accely, self.yaw

    def stop(self): 
        self.running = False