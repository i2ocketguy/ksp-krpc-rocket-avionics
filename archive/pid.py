import time

class PID:
    def __init__(self, P=1.0, I=0.1, D=0.01, clamp=500):
        self.Kp = P     # P, proportional, controls instantaneous error
        self.Ki = I     # I, integral, controls historic error
        self.Kd = D     # D, derivative, controls rate of change of error
        self.clamp = clamp
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.set_point = 0.0    # value we are trying to track
        self.last_measure = 0.0
        self.last_time = time.time()
        self.error = 0

    def update(self, measure):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            dt = 1

        self.error = self.set_point - measure
        self.P = self.error
        self.I = self.I + self.error
        self.I = self.clamp_integral(self.I)
        self.D = (measure-self.last_measure)/dt

        self.last_measure = measure
        self.last_time = current_time

        # _result = (self.Kp*self.P) + (self.Ki*self.I) - (self.Kd*self.D)
        # print("PID Output: %f" % float(_result))
        # print("P: %f, I: %f, D: %f" % (self.P, self.I, self.D))
        return (self.Kp*self.P) + (self.Ki*self.I) - (self.Kd*self.D)

    def clamp_integral(self, integrator):
        if integrator > self.clamp:
            integrator = self.clamp
        elif integrator < -self.clamp:
            integrator = -self.clamp

        return integrator


