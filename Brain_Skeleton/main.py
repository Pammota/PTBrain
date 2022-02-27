from threading import Thread
from BrainThread import BrainThread
from ImageProcessingThread import ImageProcessingThread
from LaneDetectionThread import LaneDetectionThread
from ObjectDetectionThread import ObjectDetectionThread
from multiprocessing import Pipe
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

def main():
    outP_img, inP_img = Pipe()  # out will be sent from BrainThread,
                                     # in will be recieved in ImageProcessingThread

    outP_imgProc_lane, inP_imgProc_lane = Pipe()  # out will be sent from ImageProcessingThread
                                                  # in will be recieved in LaneDetectionThread

    outP_imgProc_obj, inP_imgProc_obj = Pipe()  # out will be sent from ImageProcessingThread
                                                # in will be recieved in ObjectDetectionThread

    outP_lane, inP_lane = Pipe()  # out will be sent from LaneDetectionThread
                                       # in will be recieved in BrainThread

    outP_obj, inP_obj = Pipe()  # out will be sent from ObjectDetectionThread
                                     # in will be recieved in BrainThread

    # adds threads
    processes = []
    processes.append(ImageProcessingThread(inP_img, [outP_imgProc_lane, outP_imgProc_obj]))
    processes.append(LaneDetectionThread(inP_imgProc_lane, outP_lane, show_lane=args['show_lane']))
    processes.append(ObjectDetectionThread(inP_imgProc_obj, outP_obj))

    outPs_brain = [outP_img]
    inPs_brain = [inP_lane, inP_obj]

    for process in processes:
        process.start()

    if args['path_to_vid'] is None:
        if args['stop_car'] is None:
            brain = BrainThread(inPs_brain, outPs_brain)
        else:
            brain = BrainThread(inPs_brain, outPs_brain, stop_car=args['stop_car'])
    else:
        if args['stop_car'] is None:
            brain = BrainThread(inPs_brain, outPs_brain, cameraSpoof=args['path_to_vid'])
        else:
            brain = BrainThread(inPs_brain, outPs_brain, cameraSpoof=args['path_to_vid'], stop_car=args['stop_car'])
    brain.start()


if __name__ == '__main__':
    main()
