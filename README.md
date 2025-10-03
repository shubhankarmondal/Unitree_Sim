# Setup Instructions

This repo only contains research code.  
To run simulations you also need Unitree dependencies in `../Unitree_Support/`:

1. Clone Unitree SDKs and models into `Unitree_Support/`:
   git clone https://github.com/unitreerobotics/unitree_sdk2 ../Unitree_Support/unitree_sdk2
   git clone https://github.com/unitreerobotics/unitree_sdk2_python ../Unitree_Support/unitree_sdk2_python
   git clone https://github.com/unitreerobotics/unitree_mujoco ../Unitree_Support/unitree_mujoco

2. Install Python environment:
   ./setup_env.sh
   pip install -e ../Unitree_Support/unitree_sdk2_python
