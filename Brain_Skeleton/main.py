from threading import Thread, Event, Barrier
from BrainThread import BrainThread
import time
import os

brain = BrainThread("C:\\Users\\flavi\\OneDrive\\Desktop\\BFMC\\Records\\bfmc2020_online_1.avi")

brain.start()
