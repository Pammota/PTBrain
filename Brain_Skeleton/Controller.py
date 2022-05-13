import time

from data.v2x import V2X
from PathPlanner import PathPlanner

import numpy as np
from PIDControl import PIDControl
from GraphPars import GraphPars


RANDOM_POSITION = True


class Controller():
    def __init__(self):
        self.flags = {"forward": False, "forbidden": False, "parking": False, "sem_yellow": False, "sem_red": False,
                 "sem_green": False, "priority": False, "crosswalk": False, "stop": False}
        self.flags_history = []
        self.state = "Lane Follow"
        self.directions = ["right", "right", "forward", "left", "right", "stop", "stop"]
        self.dir_idx = 0
        self.had_parking = False
        self.base_speed = 13
        self.theta = 0
        self.thetas = []
        self.passed_horiz_line = False
        self.ongoing_intersection = False
        self.front_distances = []
        self.timer_start = 0
        self.timer_crt = 0
        self.pedestrian_present = False
        self.PIDController = None
        self.passed_one_intersection = False

        self.executed = {"parking": False, "crosswalk": False}

        self.env_conn = V2X()
        self.env_conn.start()

        self.coords = None
        self.veh_data = None
        self.sem_data = None

        self.graph = GraphPars()

        self.tasks_list = ["parking", "crosswalk", "semaphore"]
        self.pathPlanner = None
        self.pathPlanner = self.__localize(None)
        print(self.pathPlanner.nodes_list)
        self.v1 = None
        self.v2 = None


    def checkState(self, OD_info, LD_info, DSFront_info=100):
        self.timer_crt = time.time()

        self.setFlags(OD_info)
        self.setTheta(LD_info)
        self.passed_horiz_line = LD_info["horiz_line"]
        self.front_distances.append(DSFront_info)
        self.front_distances = self.front_distances[-7:]

        try:
            self.coords = self.env_conn.get_position()
        except:
            self.coords = None

        try:
            self.veh_data = self.env_conn.get_vehicles_data()
        except:
            self.veh_data = None

        try:
            self.sem_data = self.env_conn.get_sem_data()
        except:
            self.sem_data = None

        direction, self.v1, self.v2 = self.pathPlanner.current()

        if self.state == "Lane Follow":
            if self.passed_horiz_line and not self.flags["crosswalk"]:
                #self.setExecuted(parking=False, crosswalk=False)
                if self.validate_intersection():
                    self.state = "Intersection"
                    self.passed_one_intersection = True
                    self.ongoing_intersection = True
            if self.passed_horiz_line and self.flags["crosswalk"] and not self.executed["crosswalk"]:
                self.state = "Crosswalk"
                if not self.passed_one_intersection:
                    self.__localize(["crosswalk"])
                self.timer_start = time.time()

            # if there is a car ahead and not PID defined, define a PID
            if self.PIDController is None and self.front_distance() < 50:
                self.PIDController = PIDControl(40)
                print("ACTIVATED PID!!!")
            # if there is no car ahead and a PID defined, undefine the PID
            if self.PIDController is not None and self.front_distance() > 70:
                self.PIDController = None
                print("DACTIVATED PID")

        elif self.state == "Intersection":
            if not self.ongoing_intersection:
                self.state = "Lane Follow"

        elif self.state == "Crosswalk":
            if not self.executed["crosswalk"]:
                if self.timer_crt - self.timer_start > 5:
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
                    self.env_conn.stream(4, self.coords)
                    self.state = "Lane Follow"


    def takeAction(self):

        print(self.state)

        if self.state == "Lane Follow":
            if self.flags["parking"]:
                self.had_parking = True
                self.env_conn.stream(3, self.coords)
            elif self.had_parking is True and not self.flags["parking"] and not self.executed["parking"]:
                self.setExecuted(parking=True)
                self.had_parking = False
                if not self.passed_one_intersection:
                    self.__localize(["parking"])
                return [0, 0, 0, 0, 0, 0, 1]  #activate parking flag
            # if a PID is defined => we have a car ahead
            elif self.PIDController is not None:  # keep distance from the car in front
                speed_off = self.PIDController.update(self.front_distance())
                speed_off = np.clip(speed_off, -13, 13)
                print("PID GIVEN SPEED = {}".format(speed_off))
                return [self.base_speed + speed_off, self.theta, 0, 0, 0, 0, 0]
            return [self.base_speed, self.theta, 0, 0, 0, 0, 0]

        if self.state == "Intersection":
            try:
                direction, self.v1, self.v2 = self.pathPlanner.current()
            except:
                direction = self.directions[self.dir_idx]

            self.validate_sem()
            self.send_sign_data()

            if direction.split("_")[0] == "roundabout" and not self.passed_one_intersection:
                self.__localize(["roundabout"])

            if self.flags["stop"]:
                return [0, self.theta, 1, 0, direction, 0, 0]
            else:
                if self.flags["sem_red"] or self.flags["sem_yellow"]:
                    return [0, self.theta, 0, 1, direction, 0, 0]
                else:
                    return [0, self.theta, 0, 0, direction, 0, 0]

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


    def send_sign_data(self):
            if self.flags["sem_red"] or self.flags["sem_yellow"] or self.flags["sem_green"]:
                self.env_conn.stream(9, self.coords)
            if self.flags["stop"]:
                self.env_conn.stream(1, self.coords)
            if self.flags["priority"]:
                self.env_conn.stream(2, self.coords)
            if self.v2 == "J":
                self.env_conn.stream(7, self.coords)

    def validate_sem(self):

        if self.sem_data is None:
            return
        if self.v1 == "B" and self.v2 == "E":  ## sem with id 1
            sem_color = self.sem_data[1]
        else:
            if self.v1 == "D" and self.v2 == "E":  ## sem with id 2
                sem_color = self.sem_data[2]
            else:
                if self.v1 == "F" and self.v2 == "E":  ## sem with id 4
                    sem_color = self.sem_data[4]
                else:
                    if self.v1 == "0" and self.v2 == "A":
                        sem_color = self.sem_data[3]  ## default is the variable sem (at the initial point)
                    else:
                        sem_color = None
        self.flags["sem_red"] = False
        self.flags["sem_yellow"] = False
        self.flags["sem_green"] = False
        if sem_color is not None:
            self.flags[sem_color] = True

    def validate_intersection(self):
        try:
            valid = self.graph.validate_intersection(self.coords)
            return valid
        except:
            pass
        return True

    def __localize(self, fulfilled=None):
        if not RANDOM_POSITION:
            return PathPlanner(["A", "B", "E", "H", "I", "0"])

        start_con_time = time.time()

        if fulfilled is None:
            return self.pathPlanner

        while time.time() - start_con_time < 3:
            try:
                time.sleep(0.03)
                self.coords = self.env_conn.get_position()

                if self.coords is None:
                    print("COORDS ARE None")
                else:
                    print(self.coords)
                starting_points = self.graph.get_starting_position(self.coords)
                print(starting_points)

                if self.coords is not None:
                    if starting_points[1] == "E":
                        self.tasks_list.remove("semaphore")
                    if fulfilled is not None:
                        self.tasks_list = [task for task in self.tasks_list if task not in fulfilled]
                    tasks_list = self.tasks_list
                    pathPlanner = PathPlanner(tasks_list=tasks_list, starting_points=starting_points)
                    return pathPlanner
            except Exception as e:
                print(str(e))

        return PathPlanner(["I", "J", "G", "D", "A", "0"])

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
