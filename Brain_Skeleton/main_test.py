from imu import imu
import time

imu_ = imu()

imu_.start()

time.sleep(10)
imu_.stop()