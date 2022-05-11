import time

#from data.v2x import V2X
from PathPlanner import PathPlanner

import numpy as np
from PIDControl import PIDControl


RANDOM_POSITION = False


class Controller():
    def __init__(self):
        self.flags = {"forward": False, "forbidden": False, "parking": False, "sem_yellow": False, "sem_red": False,
                 "sem_green": False, "priority": False, "crosswalk": False, "stop": False}
        self.flags_history = []
        self.state = "Lane Follow"
        self.directions = ["left", "forward", "left", "right", "left", "right", "right", "stop"]
        self.dir_idx = 0
        self.had_parking = False
        self.base_speed = 17
        self.theta = 0
        self.thetas = []
        self.passed_horiz_line = False
        self.ongoing_intersection = False
        self.front_distances = []
        self.timer_start = 0
        self.timer_crt = 0
        self.pedestrian_present = False
        self.PIDController = None

        self.executed = {"parking": False, "crosswalk": False}

        self.env_conn = None #V2X()

        self.coord = None
        self.veh_data = None
        self.sem_data = None

        #self.pathPlanner = self.__localize()


    def checkState(self, OD_info, LD_info, DSFront_info=100):
        self.timer_crt = time.time()

        self.setFlags(OD_info)
        self.setTheta(LD_info)
        self.passed_horiz_line = LD_info["horiz_line"]
        self.front_distances.append(DSFront_info)
        self.front_distances = self.front_distances[-7:]

        """self.coord = self.env_conn.get_position()
        self.veh_data = self.env_conn.get_vehicles_data()
        self.sem_data = self.env_conn.get_sem_data()"""

        if self.state == "Lane Follow":
            if self.passed_horiz_line and not self.flags["crosswalk"]:
                if self.dir_idx > len(self.directions):
                    self.state = "Terminate"
                else:
                    #self.setExecuted(parking=False, crosswalk=False)
                    self.state = "Intersection"
                    self.ongoing_intersection = True
            if self.passed_horiz_line and self.flags["crosswalk"] and not self.executed["crosswalk"]:
                self.state = "Crosswalk"
                self.timer_start = time.time()

            # if there is a car ahead and not PID defined, define a PID
            if self.PIDController is None and self.front_distance() < 100:
                self.PIDController = PIDControl(40)
                print("ACTIVATED PID!!!")
            # if there is no car ahead and a PID defined, undefine the PID
            if self.PIDController is not None and self.front_distance() > 100:
                self.PIDController = None
                print("DACTIVATED PID")

        elif self.state == "Intersection":
            if not self.ongoing_intersection:
                self.state = "Lane Follow"

        elif self.state == "Crosswalk":
            if not self.executed["crosswalk"]:
                if self.timer_crt - self.timer_start > 3:
                    if self.front_distance() > 60:
                        print("Stopped at the crosswalk")
                        self.setExecuted(crosswalk=True)
                        self.timer_start = time.time()
                    else:
                        print("A pedestrian is on the crosswalk!")
                        self.pedestrian_present = True
            else:
                if self.pedestrian_present:
                    if self.timer_crt - self.timer_start > 1:
                        self.pedestrian_present = False
                        self.timer_start = time.time()
                elif self.timer_crt - self.timer_start > 2:
                    self.state = "Lane Follow"


    def takeAction(self):
        if self.state == "Lane Follow":
            if self.flags["parking"]:
                print("set had_parking to True")
                self.had_parking = True
            elif self.had_parking is True and not self.flags["parking"] and not self.executed["parking"]:
                self.setExecuted(parking=True)
                print("Set had_parking to false")
                self.had_parking = False
                return [0, 0, 0, 0, 0, 0, 1]  #activate parking flag
            # if a PID is defined => we have a car ahead
            elif self.PIDController is not None:  # keep distance from the car in front
                speed_off = self.PIDController.update(self.front_distance())
                speed_off = np.clip(speed_off, -13, 13)
                print("PID GIVEN SPEED = {}".format(speed_off))
                return [self.base_speed + speed_off, self.theta, 0, 0, 0, 0, 0]
            return [self.base_speed, self.theta, 0, 0, 0, 0, 0]

        if self.state == "Intersection":
            if self.flags["stop"]:
                self.ongoing_intersection = False
                return [0, self.theta, 1, 0, self.directions[self.dir_idx], 0, 0]
            else:
                if self.flags["sem_red"]:
                    return [0, self.theta, 0, 1, self.directions[self.dir_idx], 0, 0]
                else:
                    self.ongoing_intersection = False
                    return [0, self.theta, 0, 0, self.directions[self.dir_idx], 0, 0]

        if self.state == "Crosswalk":
            if self.executed["crosswalk"]:
                return [self.base_speed, 0, 0, 0, 0, 0, 0]
            else:
                return [0, 0, 0, 0, 0, 0, 0]
        return None

    def setFlags(self, OD_info):
        self.flags_history.append(OD_info)
        self.flags_history = self.flags_history[-25:]
        for k in self.flags.keys():
            if k == "crosswalk":
                self.flags[k] = (np.sum([1 if fl[k] is True else 0 for fl in self.flags_history]) > 1)
            elif k == "stop":
                self.flags[k] = (np.sum([1 if fl[k] is True else 0 for fl in self.flags_history[-20:]]) > 4)
            else:
                self.flags[k] = (np.sum([1 if fl[k] is True else 0 for fl in self.flags_history[-10:]]) > 3)

    def setTheta(self, LD_info):
        self.thetas.append(LD_info["theta"])
        if len(self.thetas) == 1:
            self.theta = np.average(self.thetas)
            self.thetas = []

    def setExecuted(self, parking=None, crosswalk=None):
        if crosswalk is not None:
            self.executed["crosswalk"] = crosswalk

    def front_distance(self):
        return np.median(self.front_distances)


    def __localize(self):
        if not RANDOM_POSITION:
            return PathPlanner(["0", "A", "B", "D", "A", "0"])

        start_con_time = time.time()

        while time.time() - start_con_time < 60:
            try:
                self.coord = self.env_conn.get_position()


            except:
                pass

        return PathPlanner(["0", "A", "B", "D", "A", "0"])

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
