"""
tests/test_pid.py

Test the PID controller with a simple system:
we want to reach a setpoint starting from 0.
"""

import matplotlib.pyplot as plt
from controllers.pid_controller import PIDController

def run_pid_test():
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=10.0, dt=0.1, output_limits=(-5, 5))

    measurement = 0.0
    outputs = []
    measurements = []
    time = []

    for t in range(100):
        control = pid.compute(measurement)
        # Simple system: next state = current + control*dt
        measurement += control * 0.1
        outputs.append(control)
        measurements.append(measurement)
        time.append(t * pid.dt)

    # Plot response
    plt.figure()
    plt.plot(time, measurements, label="Measurement")
    plt.axhline(pid.setpoint, color="r", linestyle="--", label="Setpoint")
    plt.xlabel("Time [s]")
    plt.ylabel("Value")
    plt.legend()
    plt.title("PID Controller Response")
    plt.show()

if __name__ == "__main__":
    run_pid_test()
