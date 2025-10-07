# Unitree Go2 MuJoCo Simulation

This repository contains simulation and control scripts for the **Unitree Go2 quadruped** using the MuJoCo physics engine.  
It provides standalone PD-based control for **standing and trotting behaviors** without requiring live DDS or SDK2 middleware.

---

## ⚙️ Setup Instructions

This repo only contains research simulation code.  
To run simulations you also need Unitree dependencies in `../Unitree_Support/`:

```bash
# Clone Unitree SDKs and models into Unitree_Support/
git clone https://github.com/unitreerobotics/unitree_sdk2 ../Unitree_Support/unitree_sdk2
git clone https://github.com/unitreerobotics/unitree_sdk2_python ../Unitree_Support/unitree_sdk2_python
git clone https://github.com/unitreerobotics/unitree_mujoco ../Unitree_Support/unitree_mujoco
