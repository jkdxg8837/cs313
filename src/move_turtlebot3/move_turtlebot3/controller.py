
class Controller:
    def __init__(self, P=0.9, D=0.5, set_point=0.0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point #reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D