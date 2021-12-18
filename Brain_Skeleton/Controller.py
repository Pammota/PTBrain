
class Controller:
    def __init__(self, time_threshold=0.25):
        self.speed = 0
        self.angle = 0
        self.time_threshold = time_threshold

    def update_angle(self, theta):
        self.angle = theta
        return {'action': '2', 'steerAngle': float(self.angle)}


    def update_speed(self, speed, startup=False, time_elapsed=0):
        if speed > 0:
            if self.speed == 0:
                self.speed = 26
                return {'action': '1', 'speed': float(self.speed/100.0)}, True
            else:
                if startup is True and time_elapsed < self.time_threshold:
                    return {'action': '1', 'speed': float(self.speed/100.0)}, True
                else:
                    self.speed = speed
                    return {'action': '1', 'speed': float(self.speed/100.0)}, False
        elif speed < 0:
            if self.speed == 0:
                self.speed = -20
                return {'action': '1', 'speed': float(self.speed/100.0)}, True
            else:
                if startup is True and time_elapsed < self.time_threshold:
                    return {'action': '1', 'speed': float(self.speed/100.0)}, True
                else:
                    self.speed = speed
                    return {'action': '1', 'speed': float(self.speed/100.0)}, False
        else:
            self.speed = 0
            return {'action': '1', 'speed': float(self.speed/100.0)}, True
