from threading import Thread
from BrainThread import BrainThread
from threading import Event
import argparse
import config
import time
import os

ag = argparse.ArgumentParser()

ag.add_argument('-p', '--path_to_vid', required=False)
ag.add_argument('-v', '--show_vid', required=False)
ag.add_argument('-l', '--show_lane', required=False)
ag.add_argument('-s', '--stop_car', required=False)
#ag.add_argument('-m', '--run_mode', required=False)
args = vars(ag.parse_args())

if args['path_to_vid'] is None:
    if args['stop_car'] is None:
        brain = BrainThread()
    else:
        brain = BrainThread(stop_car=args['stop_car'])
else:
    if args['stop_car'] is None:
        brain = BrainThread(cameraSpoof=args['path_to_vid'])
    else:
        brain = BrainThread(cameraSpoof=args['path_to_vid'], stop_car=args['stop_car'])
brain.start()

blocker = Event()

try:
    blocker.wait()
finally:
    brain.terminate()

print("Exitted")
