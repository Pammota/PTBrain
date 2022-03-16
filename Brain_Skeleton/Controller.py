import numpy as np

class Controller():
    def __init__(self):
        self.flags = {}
        self.state = "Lane Follow"
        self.directions = ["right", "left", "right", "forward", "left", "forward"]
        self.dir_idx = 0
        self.had_parking = False
        self.base_speed = 13
        self.theta = 0
        self.thetas = []
        self.passed_horiz_line = False
        self.ongoing_intersection = False

        self.executed = {"parking": False, "crosswalk": False}


    def checkState(self, OD_info, LD_info):
        self.setFlags(OD_info)
        self.setTheta(LD_info)
        self.passed_horiz_line = LD_info["horiz_line"]

        if self.state == "Lane Follow":
            if self.passed_horiz_line and not self.flags["crosswalk"]:
                if self.dir_idx > len(self.directions):
                    self.state = "Terminate"
                else:
                    self.setExecuted(parking=False, crosswalk=False)
                    self.state = "Intersection"
                    self.ongoing_intersection = True

        if self.state == "Intersection":
            if not self.ongoing_intersection:
                self.state = "Lane Follow"
                self.dir_idx += 1

    def takeAction(self):
        if self.state == "Lane Follow":
            if self.flags["parking"] is True:
                self.had_parking = True
            else:
                if self.flags["crosswalk"] and self.passed_horiz_line and not self.executed["crosswalk"]:
                    self.passed_horiz_line = False
                    self.setExecuted(crosswalk=True)
                    return [0, 0, 0, 0, 0, 1, 0]  # activate crosswalk flag
            if self.had_parking is True and self.flags["parking"] is False and not self.executed["parking"]:
                self.setExecuted(parking=True)
                self.had_parking = False
                return [0, 0, 0, 0, 0, 0, 1]  #activate parking flag
            return [self.base_speed, self.theta, 0, 0, 0, 0, 0]

        if self.state == "Intersection":
            if self.flags["stop"] is True:
                self.ongoing_intersection = False
                return [0, 0, 1, 0, self.directions[self.dir_idx], 0, 0]
            else:
                if self.flags["sem_red"]:
                    return [0, 0, 0, 1, self.directions[self.dir_idx], 0, 0]
                else:
                    self.ongoing_intersection = False
                    return [0, 0, 0, 0, self.directions[self.dir_idx], 0, 0]

        return None

    def setFlags(self, OD_info):
        self.flags = OD_info

    def setTheta(self, LD_info):
        self.thetas.append(LD_info["theta"])
        if len(self.thetas) == 1:
            self.theta = np.average(self.thetas)
            self.thetas = []

    def setExecuted(self, parking=None, crosswalk=None):
        if parking is not None:
            self.executed["parking"] = parking
        if crosswalk is not None:
            self.executed["crosswalk"] = crosswalk

    @staticmethod
    def getAngleCommand(theta):
        if theta >= 23:
            theta = 22
        elif theta <= -23:
            theta = -22
        return {'action': '2', 'steerAngle': float(theta)}

    @staticmethod
    def getSpeedCommand(speed):
        return {'action': '1', 'speed': float(speed/100.0)}
