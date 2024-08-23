# VeriFly - Fast Verification of LLM-Powered Robots using NVIDIA Isaac Sim

Author: Sebastian Orozco\
Mentors: Neiwen Ling, Guojun Chen

## What is VeriFly?
VeriFly is an end-to-end pipeline for fast verification of LLM-powered robots. It connects TypeFly with a digital twin operating within the NVIDIA Isaac Sim. Through the TypeFly UI, users can test the performance of their robot against various tasks in a pre-defined simulated environment.

## How to get started?
### Step 1: Setup your environment
First make sure you have both TypeFly and the NVIDIA Isaac Sim installed and configured on your computer. 
Then, cd into the **TypeFly** directory and update the `virtual_robot_wrapper.py` file to match the version in this repository.
Next, cd into the **isaac-sim-4.0.0** directory and download the `runtime-code-insertion` folder from this repository into this directory. 

*** ^^^ TODO: SPECIFY METHOD OF INSTALLING ISAAC SIM. CHANGE NAME OF VIRTUAL_ROBOT_WRAPPER.PY TO ISAAC_ROBOT_WRAPPER.PY. ***

### Step 2: Pre-define your scenario
In `typefly.py`, update the **BASE_SCENE_PATH** variable to be the path to the .usd file containing your pre-built scenario that you would like to test the robot in. 
By default, VeriFly uses the NVIDIA Kaya robot car. Future support will be added for other robots. Additionally, VeriFly's camera data is produced from the perspective of a camera mounted onto the Kaya robot. If you would like to instead simulate an eye-level perspective, you can adjust the y-value of the camera's position. Please note that the camera will still be affixed to the Kaya robot and continue to move with the robot, even if it is not physically adjacent. 

*** ^^^ TODO: ADD MORE DETAILS ABOUT SCENARIO BUILDING -> HOW TO LOAD SCENARIO (INCL. HOW TO SEARCH FOR ASSETS)? -> HOW TO CHANGE START LOCATION OF ROBOT? ***

### Step 3: Launch
In the terminal, run the command `(cd /path/to/isaac-sim-4.0.0 && ./python.sh typefly.py) & (cd /path/to/TypeFly && make typefly)`. If you now go to the local web server hosting the TypeFly UI, you will see that it is now connected to robot within your specified scenario.

*** ^^^ TODO: ADD MORE DETAILS ABOUT PROCEDURE, INCLUDING ABOUT THE TYPEFLY.PY FILE. CHANGE NAME OF TYPEFLY.PY FILE (TO AVOID CONFUSION), PERHAPS TYPE_SIMULATOR.PY. ADD INT_MAIN TO BOTTOM OF TYPEFLY.PY. CREATE OWN .SH FILE TO SIMPLIFY LAUNCHING PROCESS AND MAKE IT MORE ELEGANT. ***

*** TODO: INCLUDE DESCRIPTION FOR MEANING OF EACH FILE IN FOLDER AS NEW SECTION. ***

## Code Strcuture
```
 |-- SimFly                    // codes for xxxxx
    |-- virtual_robot_wrapper.py // code for xx
    |-- instructions.txt // file for xx

 |-- VRC                    // codes for xxxxx
*** TODO: Fill code structure here. ***
