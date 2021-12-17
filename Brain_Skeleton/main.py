from threading import Thread, Event, Barrier
from BrainThread import BrainThread
import time
import os

brain = BrainThread()

brain.start()
