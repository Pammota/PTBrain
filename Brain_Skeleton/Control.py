import numpy as np

class Control():
    def __init__(self):
        #call the GraphPars for path planning based on the maneuvers we want
        # flags given by oobject detection -
        self.flags = {}
        self.state = "Lane Follow"
        self.action = {"speed": 0, "angle": 0, "stop": False, "red": False, "direction": "forward",
                       "crosswalk": False, "parking": False}
        self.directions = ["forward"]
        self.dir_idx = 0
        self.had_parking = False
        self.base_speed = 20
        self.theta = 0
        self.thetas = []
        self.passed_horiz_line = False
        self.ongoing_intersection = False


    def CheckState(self, OD_info, LD_info):
        self.SetFlags(OD_info)
        self.SetTheta(LD_info)
        self.passed_horiz_line = False

        if self.state == "Lane Follow":
            if self.passed_horiz_line == True:
                self.state = "Intersection"
                self.ongoing_intersection = True

        if self.state == "Intersection":
            if self.ongoing_intersection == False:
                self.state = "Lane Follow"
                self.dir_idx += 1


    def SetFlags(self, OD_info):
        self.flags = OD_info

    def SetTheta(self, LD_info):
        self.thetas.append(LD_info["theta"])
        if len(self.thetas) == 3:
            self.theta = np.average(self.thetas)
            self.thetas = []