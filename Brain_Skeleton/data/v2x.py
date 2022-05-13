import os
import time

from data.environmentalserver.environmental import EnvironmentalHandler
from data.localisationssystem.locsys import LocalisationSystem
from data.trafficlights.trafficlights import trafficlights
from data.vehicletovehicle.vehicletovehicle import vehicletovehicle

COLORS = ['sem_red', 'sem_yellow', 'sem_green']

class V2X():
    def __init__(self, ID = 88):

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


    def get_position(self):
        """
        returns a complex number which represents the coordinates of the car
        """
        coords_complex = self.gps.coor()[0]
        return {"x": coords_complex.real, "y": coords_complex.imag}


    def stream(self, obstacle_id, coords):
        """
        streams a message containing an obstacle_id and its position to the environmental server
        """
        try:
            self.env.stream(obstacle_id, coords['x'], coords['y'])
        except Exception as e:
            print(str(e))
            print("Could not stream data. This is not an error.")

    def get_vehicles_data(self):
        """
        returns the id, timestamp recieved, position and orientation of another car
        """
        (ID, timestamp, pos, ang) = self.v2x.ID, self.v2x.timestamp, self.v2x.pos, self.v2x.ang
        return {"ID": ID, "timestamp": timestamp, "pos": pos, "and": ang}


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

"""if __name__ == "__main__":
    v2x = V2X()
    v2x.start()

    time.sleep(2)

    v2x.stop()"""
