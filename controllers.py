import time
import numpy as np


class PID:
    def __init__(
        self,
        set_point: float,
        P: float,
        I: float,
        D: float,
        min_output: float,
        max_output: float,
        deadband: float = 0.01,
        rate_limit: float = None,
        velocity_form: bool = True,
    ):
        # Controller gains
        self.Kp = P
        self.Ki = I
        self.Kd = D

        # Controller limits
        self.max_output = max_output
        self.min_output = min_output
        self.rate_limit = rate_limit
        self.deadband = deadband
        self.velocity_form = velocity_form

        # State variables
        self.set_point = set_point
        self.last_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0
        self.last_derivative = 0.0
        self.last_time = time.time()

    def update(self, current_sample):
        current_time = time.time()
        dt = current_time - self.last_time

        # Skip update if no time has passed
        if dt <= 0:
            return self.last_output

        current_error = self.set_point - current_sample

        # Apply deadband
        if self.check_deadband(current_error):
            self.last_time = current_time
            return self.last_output

        if self.velocity_form:
            output = self.velocity_update(current_error, dt)
        else:
            output = self.position_update(current_error, dt)

        output = self.apply_output_limits(output)

        # Update state variables for next call/loop
        self.last_error = current_error
        self.last_time = current_time
        self.last_output = output

        return output

    def velocity_update(self, current_error: float, dt: float) -> float:
        """Velocity form PID update"""
        # Calculate change in error
        delta_error = current_error - self.last_error

        # Calculate filtered derivative
        derivative = delta_error / dt
        filter_tau = max(self.Kd / 10, dt)  # Prevent tau = 0
        alpha = dt / (filter_tau + dt)
        filtered_derivative = alpha * derivative + (1 - alpha) * self.last_derivative
        self.last_derivative = filtered_derivative

        # Calculate change in control signal
        delta_u = (
            self.Kp * delta_error  # P term
            + self.Ki * current_error * dt  # I term (using current_error)
            + self.Kd * filtered_derivative  # D term
        )

        # Calculate new output
        new_output = self.last_output + delta_u

        # Anti-windup: if output would saturate, don't include integral term
        # if new_output > self.max_output or new_output < self.min_output:
        #     delta_u = self.Kp * delta_error + self.Kd * filtered_derivative
        #     new_output = self.last_output + delta_u

        return new_output

    def position_update(self, current_error: float, dt: float) -> float:
        """Position form PID update with anti-windup"""
        # Calculate proportional term
        p_term = self.Kp * current_error

        # Calculate derivative term with filtering (only if Kd != 0)
        d_term = 0.0
        if self.Kd != 0:
            derivative = (current_error - self.last_error) / dt
            filter_tau = max(self.Kd / 10, dt)
            alpha = dt / (filter_tau + dt)
            filtered_derivative = alpha * derivative + (1 - alpha) * self.last_derivative
            self.last_derivative = filtered_derivative
            d_term = self.Kd * filtered_derivative

        # Calculate temporary output without integral term
        temp_output = p_term + d_term

        # Handle integral calculations only if Ki != 0
        i_term = 0.0
        if self.Ki != 0:
            # Compute trial integral term
            self.integral += current_error * dt
            
            # Limit integral contribution to prevent saturation
            i_term = self.Ki * self.integral
            if temp_output + i_term > self.max_output:
                self.integral = (self.max_output - temp_output) / self.Ki
            elif temp_output + i_term < self.min_output:
                self.integral = (self.min_output - temp_output) / self.Ki
            
            i_term = self.Ki * self.integral

        return p_term + i_term + d_term

    def check_deadband(self, error):
        return abs(error) <= self.deadband

    def apply_output_limits(self, output: float) -> float:
        """Apply both rate and magnitude limits to output"""
        # Apply rate limiting if configured
        if self.rate_limit is not None:
            max_change = self.rate_limit * (time.time() - self.last_time)
            output = np.clip(
                output, self.last_output - max_change, self.last_output + max_change
            )

        # Apply magnitude limits
        return np.clip(output, self.min_output, self.max_output)

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        self.last_derivative = 0.0
        self.last_time = time.time()

    def set_gains(self, P=None, I=None, D=None):
        """Update controller gains safely during operation"""
        if P is not None:
            self.Kp = P
        if I is not None:
            self.Ki = I
        if D is not None:
            self.Kd = D

    def set_setpoint(self, setpoint: float):
        """Update controller setpoint"""
        self.set_point = setpoint

    def set_min_output(self, new_min):
        self.min_output = new_min

    def set_max_output(self, new_max):
        self.max_output = new_max

    def debug(self) -> dict:
        """Return dictionary of current controller state and parameters

        Example usage:
        # In your control loop
        while True:
            output = pid.update(current_value)

            # Print debug info every second
            if time.time() % 1 < 0.1:  # Prints roughly every second
                debug_info = pid.debug()
                print(f"Error: {debug_info['error']:.3f}")
                print(f"Output: {debug_info['output']:.3f}")
                print(f"Integral: {debug_info['integral']:.3f}")
                print(f"dt: {debug_info['dt']:.3f}")
        """
        return {
            # Current states
            "time": time.time(),
            "dt": time.time() - self.last_time,
            "error": self.last_error,
            "output": self.last_output,
            "integral": self.integral,
            "derivative": self.last_derivative,
            # Configuration
            "mode": "velocity" if self.velocity_form else "position",
            "setpoint": self.set_point,
            # Gains
            "Kp": self.Kp,
            "Ki": self.Ki,
            "Kd": self.Kd,
            # Limits
            "output_min": self.min_output,
            "output_max": self.max_output,
            "rate_limit": self.rate_limit,
            "deadband": self.deadband,
        }


class CascadeController:
    def __init__(self, outerloop_pid: PID, innerloop_pid: PID, debug=False):
        """
        Initialize cascade controller with pre-configured PID controllers

        Args:
            outerloop_pid (PID): The outer loop PID controller
            innerloop_pid (PID): The inner loop PID controller
            debug (bool): Enable debug output printing
        """

        if innerloop_pid.Kp / outerloop_pid.Kp < 5:
            print("Warning: Cascading controller innerloop pid gain may be too low.")

        self.outerloop_pid = outerloop_pid
        self.innerloop_pid = innerloop_pid
        self.debug = debug

    def update(self, outerloop_input, innerloop_input):
        """
        Update cascade controller with new inputs

        Args:
            outerloop_input: Input for primary (outer) controller
            innerloop_input: Input for secondary (inner) controller

        Returns:
            float: Control output from secondary controller
        """
        outerloop_output = self.outerloop_pid.update(outerloop_input)
        self.innerloop_pid.set_point = outerloop_output
        innerloop_output = self.innerloop_pid.update(innerloop_input)

        if self.debug:
            print(
                f"\rCascade Debug | OuterLoop: out={outerloop_output:.2f} | Secondary: out={innerloop_output:.2f}",
                end="",
            )

        return innerloop_output

    def set_min_output(self, new_min):
        self.innerloop_pid.set_min_output(new_min)

    def set_max_output(self, new_max):
        self.innerloop_pid.set_max_output(new_max)

    def reset(self):
        self.innerloop_pid.reset()
        self.outerloop_pid.reset()
