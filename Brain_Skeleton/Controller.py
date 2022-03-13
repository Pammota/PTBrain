
class Controller:
    def __init__(self, time_threshold=0.15):
        self.speed = 0
        self.angle = 0
        self.kp = 0.08
        self.kd = 0.001
        self.ki = 0.0001
        self.last_error = 0
        self.error_sum = 0
        self.time_threshold = time_threshold

    def update_angle(self, theta):
        error = theta
        self.error_sum = self.error_sum + error
        if abs(error) < 6:
            self.error_sum = 0
        self.angle = theta + self.kp * theta + self.kd * (error - self.last_error) + self.ki * self.error_sum
        if self.angle > 23:
            self.angle = 23
        elif self.angle < -23:
            self.angle = -23
        self.last_error = error
        return {'action': '2', 'steerAngle': float(self.angle)}

    @staticmethod
    def must_stop(traffic_lights_info):

        if len(traffic_lights_info) == 0:
            #print("empty")
            return False
        dominant = max(traffic_lights_info, key=lambda x: x.get('score'))

        color = dominant.get('color')
        #print(color)
        if color == 1:
            return True
        if color == 2:
            return True
        return False

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
            return {'action': '1', 'speed': float(self.speed/100.0)}, False
