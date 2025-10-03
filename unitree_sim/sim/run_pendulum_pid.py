import mujoco
import mujoco.viewer
import numpy as np
import time

from controllers.pid_controller import PIDController

# Load pendulum model
model = mujoco.MjModel.from_xml_path("sim/pendulum.xml")
data = mujoco.MjData(model)

# Target angle (radians)
target_angle = np.pi / 4  # 45 degrees

# PID controller to control hinge joint
pid = PIDController(kp=10.0, ki=1.0, kd=0.5, setpoint=target_angle, dt=0.01, output_limits=(-2, 2))

print("✅ Pendulum model loaded. Starting PID control...")

# Launch viewer for visualization
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running():
        # Current angle of the hinge joint
        angle = data.qpos[0]

        # Compute torque from PID
        torque = pid.compute(angle)

        # Apply torque to actuator
        data.ctrl[0] = torque

        # Step simulation
        mujoco.mj_step(model, data)

        # Sync with wall clock
        viewer.sync()
