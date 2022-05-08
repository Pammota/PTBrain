import os
import time

from environmentalserver.environmental import EnvironmentalHandler
from localisationssystem.locsys import LocalisationSystem
from trafficlights.trafficlights import trafficlights
from vehicletovehicle.vehicletovehicle import vehicletovehicle

COLORS = ['sem_red', 'sem_yellow', 'sem_green']

class V2X():
    def __init__(self, ID = 120):

        ### initialize communication threads

        self.env = EnvironmentalHandler(ID)
        self.gps = LocalisationSystem(ID)
        self.sems = trafficlights()
        self.v2x = vehicletovehicle()

    def start(self):
        ### start the communication threads
        self.env.start()
        self.gps.start()
        self.sems.start()
        self.v2x.start()


    def get_position(self):
        """
        returns a complex number which represents the coordinates of the car
        """
        return self.gps.coor()


    def stream(self, obstacle_id, x, y):
        """
        streams a message containing an obstacle_id and its position to the environmental server
        """
        self.env.stream(obstacle_id, x, y)


    def get_vehicles_data(self):
        """
        returns the id, timestamp recieved, position and orientation of another car
        """
        (ID, timestamp, pos, ang) = self.v2x.ID, self.v2x.timestamp, self.v2x.pos, self.v2x.ang
        return ID, timestamp, pos, ang


    def get_sem_data(self):
        """
        returns a list with element i representing the state of the i-th semaphore
        """
        sem_data = [None, COLORS[self.sems.s1_state], COLORS[self.sems.s2_state],
                    COLORS[self.sems.s3_state], COLORS[self.sems.s4_state]]
        return sem_data


    def stop(self):
        self.env.stop()
        self.gps.stop()
        self.sems.stop()
        self.v2x.stop()

if __name__ == "__main__":
    v2x = V2X()
    v2x.start()

    time.sleep(2)

    v2x.stop()
