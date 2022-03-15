import numpy as np

class Controller():
    def __init__(self):
        #call the GraphPars for path planning based on the maneuvers we want
        # flags given by oobject detection -
        self.flags = {}
        self.state = "Lane Follow"
        self.directions = ["forward"]
        self.dir_idx = 0
        self.had_parking = False
        self.base_speed = 20
        self.theta = 0
        self.thetas = []
        self.passed_horiz_line = False
        self.ongoing_intersection = False


    def checkState(self, OD_info, LD_info):
        self.SetFlags(OD_info)
        self.SetTheta(LD_info)
        self.passed_horiz_line = False

        if self.state == "Lane Follow":
            if self.passed_horiz_line is True:
                self.state = "Intersection"
                self.ongoing_intersection = True

        if self.state == "Intersection":
            if self.ongoing_intersection is False:
                self.state = "Lane Follow"
                self.dir_idx += 1

    def takeAction(self):
        if self.state == "Lane Follow":
            if self.flags["parking"] is True:
                self.had_parking = True
            else:
                if self.flags["crosswalk"] is True:
                    return [0, 0, 0, 0, 0, 1, 0]  # activate crosswalk flag
            if self.had_parking is True and self.flags["parking"] is False:
                return [0, 0, 0, 0, 0, 0, 1]  #activate parking flag
            return [self.base_speed, self.theta, 0, 0, 0, 0, 0]

        if self.state == "Intersection":
            if self.flags["stop"] is True:
                return [0, 0, 1, 0, self.directions[self.dir_idx], 0, 0]
            else:
                if self.flags["sem_red"]:
                    return [0, 0, 0, 1, self.directions[self.dir_idx], 0, 0]
                else:
                    return [0, 0, 0, 0, self.directions[self.dir_idx], 0, 0]

        return [0, 0, 0, 0, "forward", 0, 0]

    def setFlags(self, OD_info):
        self.flags = OD_info

    def setTheta(self, LD_info):
        self.thetas.append(LD_info["theta"])
        if len(self.thetas) == 3:
            self.theta = np.average(self.thetas)
            self.thetas = []

    @staticmethod
    def getAngleCommand(theta):
        """error = theta
        self.error_sum = self.error_sum + error
        if abs(error) < 6:
            self.error_sum = 0"""
        #+ self.kp * theta + self.kd * (error - self.last_error) + self.ki * self.error_sum"""
        if theta >= 23:
            theta = 22
        elif theta <= -23:
            theta = -22
        #self.last_error = error
        return {'action': '2', 'steerAngle': float(theta)}

    @staticmethod
    def getSpeedCommand(speed):
        return {'action': '1', 'speed': float(speed/100.0)}
