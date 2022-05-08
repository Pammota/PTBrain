import os
from environmentalserver.environmental import EnvironmentalHandler
from localisationssystem.locsys import LocalisationSystem
from trafficlights.trafficlights import trafficlights
from vehicletovehicle.vehicletovehicle import vehicletovehicle

class V2X():
    def __init__(self, ID = 120):

        ### initialize communication threads

        self.env = EnvironmentalHandler(ID)
        self.gps = LocalisationSystem(ID)
        self.sems = trafficlights()
        self.v2x = vehicletovehicle()

        self.env.start()
        self.gps.start()
        self.sems.start()
        self.v2x.start()

    def stop(self):