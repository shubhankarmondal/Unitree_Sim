import mujoco
import mujoco.viewer
import numpy as np

# Load a basic MuJoCo model (built-in XML string for a particle)
xml = """
<mujoco>
  <worldbody>
    <body name="ball" pos="0 0 1">
      <geom type="sphere" size="0.1" rgba="0 0 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("✅ MuJoCo model loaded.")

# Step through the simulation
for i in range(100):
    mujoco.mj_step(model, data)

print("✅ MuJoCo simulation stepped 100 steps successfully.")

# If you want to render (requires GUI), uncomment:
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     while viewer.is_running():
#         mujoco.mj_step(model, data)
#         viewer.sync()
