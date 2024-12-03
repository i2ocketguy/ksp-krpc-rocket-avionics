import time
import numpy as np

class PID:
    def __init__(
            self,
            set_point,
            P,
            I,
            D,
            min_output,
            max_output,
            clamp=500,
            deadband=0.01,
            rate_limit=None):
        self.Kp = P     # P, proportional, controls instantaneous error
        self.Ki = I     # I, integral, controls historic error
        self.Kd = D     # D, derivative, controls rate of change of error
        self.clamp = clamp
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.set_point = set_point    # value we are trying to track
        self.last_input = 0.0
        self.last_time = time.time()
        self.error = 0
        self.deadband = deadband
        self.max_output = max_output
        self.min_output = min_output
        self.rate_limit = rate_limit
        self.last_output = 0.0

    def update(self, current_input):
        self.error = self.set_point - current_input
        current_time = time.time()

        self.P = self.error

        if self.last_time < current_time:
            if not self.check_deadband():
                self.I, self.D = self.update_integral_derivative(current_time, current_input)

        self.last_input = current_input
        self.last_time = current_time

        output = (self.Kp * self.P) + (self.Ki * self.I) - (self.Kd * self.D)
        output = self.clamp_output(output)

        # If a rate limit is set and this is not the first output, apply the rate limit
        if self.rate_limit is not None:
            output_change = output - self.last_output
            if abs(output_change) > self.rate_limit:
                output = self.last_output + self.rate_limit * np.sign(output_change)

        self.last_output = output

        return output

    def update_integral_derivative(self, current_time, current_input):
        dt = current_time - self.last_time
        if np.absolute(self.error) > np.power(10.0, -6.0):
            integrator = self.update_integral_term(dt)
            derivative = self.update_derivative_term(dt, current_input)

        return integrator, derivative

    def update_integral_term(self, dt):
        if abs(self.Ki) > 0:
            integrator = self.I + self.error * dt
            integrator = self.clamp_integral(integrator)
        else:
            integrator = 0

        return integrator

    def update_derivative_term(self, dt, current_input):
        if abs(self.Kd) > 0:
            derivative = (current_input - self.last_input) / dt
        else:
            derivative = 0

        return derivative

    def check_deadband(self):
        return abs(self.error) < self.deadband

    def clamp_integral(self, integrator):
        if integrator > self.clamp:
            integrator = self.clamp
        elif integrator < -self.clamp:
            integrator = -self.clamp

        return integrator

    def clamp_output(self, output):
        if output > self.max_output:
            return self.max_output
        elif output < self.min_output:
            return self.min_output
        else:
            return output

    def reset_integral(self):
        self.I = 0

    def update_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def set_min_output(self, new_min):
        self.min_output = new_min

    def set_max_output(self, new_max):
        self.max_output = new_max

    def debug(self):
        _result = (self.Kp * self.P) + (self.Ki * self.I) - (self.Kd * self.D)
        print("PID Output: %f" % float(_result))
        print("P: %f, I: %f, D: %f" % (self.P, self.I, self.D))
