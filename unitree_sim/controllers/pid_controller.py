"""
controllers/pid_controller.py

A simple PID (Proportional-Integral-Derivative) controller class
for controlling states like position, velocity, or orientation.
"""

import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0, dt=0.01, output_limits=(None, None)):
        """
        Initialize a PID controller.

        Args:
            kp (float): proportional gain
            ki (float): integral gain
            kd (float): derivative gain
            setpoint (float): desired target value
            dt (float): control timestep
            output_limits (tuple): (min, max) saturation of output
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.dt = dt
        self.output_limits = output_limits

        # Internal state
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        """Reset integral and previous error."""
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, measurement):
        """
        Compute control action given a measurement.

        Args:
            measurement (float): current system output

        Returns:
            control (float): PID output
        """
        error = self.setpoint - measurement
        self._integral += error * self.dt
        derivative = (error - self._prev_error) / self.dt

        # PID formula
        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Apply output limits if any
        min_out, max_out = self.output_limits
        if min_out is not None:
            output = max(min_out, output)
        if max_out is not None:
            output = min(max_out, output)

        self._prev_error = error
        return output
