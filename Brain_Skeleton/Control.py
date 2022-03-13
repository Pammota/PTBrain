

class Control():
    def __init__(self, OD_info, LD_info):
        #call the GraphPars for path planning based on the maneuvers we want
        #AiPrioritate, Stop, Crosswalk, Priority(cedeaza), Park, RED, YELLOW, Green, Intersection1, Intersection0
        flags = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        states = ["IDLE", "Decision center", "Crossroads", "Stop", "Left_maneuver", "Right_maneuver", "Lane Follow", "Parking"]
        curr_state = "IDLE"
        self.Check_State(OD_info, LD_info, flags, states, curr_state)

    def Check_State(self, OD_info, LD_info, flags, states, curr_state):
       self.Set_Flags(OD_info, LD_info, flags)
       if curr_state == "IDLE":
           curr_state = "Decision center"
       elif curr_state == "Decision center":
           if flags[8] == 1 and flags[9] == 1:  ## we are at the begining of the intersection
               curr_state = "Crossroads"
           else:
               curr_state = "Lane Follow"
       elif  curr_state == "Crossroads":
           if flags[1] == 1 or flags[3] == 1 or flags[5] == 1: ## obj detection gave some stops
               curr_state = "Stop"
           elif flags[8] == flags[9] == 0:
               curr_state = ""
       elif curr_state == "Stop":
           ##logic for making the car stop for 2 or 3 seconds
           flags[1] = flags[3] = flags[5] = 0 #make sure the car won't stop again on the next step
           curr_state = "Crossroads"
       #elif curr_state == ""



    def Set_Flags(self, OD_info, LD_info, flags):
        #flags.... -> set here
        a=0
