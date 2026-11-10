Requirements: stable-baselines3, mujoco, unitree_sdk2_python (unitree_sdk2)

-------------------------------------------------------------------------------------------

1. ModuleNotFoundError: No module named 'stable_baselines3'
Solution:
pip install stable-baselines3

-------------------------------------------------------------------------------------------

2. ModuleNotFoundError: No module named 'mujoco'

Solution: pip install mujoco

-------------------------------------------------------------------------------------------

3. if any ERROR of GLFW:
libEGL warning: MESA-LOADER: failed to open swrast: /usr/lib/dri/swrast_dri.so: cannot open shared object file: No such file or directory (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suffix _dri)
/home/york/.local/lib/python3.10/site-packages/glfw/__init__.py:917: GLFWError: (65542) b'EGL: Failed to initialize EGL: EGL is not or could not be initialized'
  warnings.warn(message, GLFWError)
ERROR: could not create window

Solution: 
conda install -c conda-forge libstdcxx-ng

-------------------------------------------------------------------------------------------

4. Unitree_sdk2_python, need to install unitree_sdk2
For unitree_sdk2_python packages installation, more detail please refer to https://github.com/unitreerobotics/unitree_sdk2_python.

Dependencies of unitree_sdk2_python: 
 - Python >= 3.8
 - cyclonedds == 0.10.2
 - numpy
 - opencv-python

Installing from source
Execute the following commands in the terminal:


cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .


============================================================================================

5. HOW to USE?

 a) For single arm testing to verify the packages dependencies:
 python3 g1_arm_policy_controller.py 
 
 b) If dependencies are okay, Dual Arm Control:
 python3 g1_dual_arm_policy_controller.py
 

