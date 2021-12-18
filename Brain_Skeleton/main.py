from threading import Thread
from BrainThread import BrainThread
import argparse
import time
import os

ag = argparse.ArgumentParser()

ag.add_argument('-p', '--path_to_vid', required=False)
ag.add_argument('-v', '--show_vid', required=False)
ag.add_argument('-l', '--show_lane', required=False)

args = vars(ag.parse_args())

if args['path_to_vid'] is None:
    brain = BrainThread()
else:
    brain = BrainThread(args['path_to_vid'])

brain.start()
