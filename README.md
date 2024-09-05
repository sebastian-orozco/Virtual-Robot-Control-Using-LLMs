# SimFly
Author: Sebastian Orozco\
Mentors: Neiwen Ling, Guojun Chen, Lin Zhong

## Description
SimFly is an extension of [TypeFly](https://typefly.github.io/) for controlling simulated robots using large-language models. This project connects [TypeFly](https://github.com/typefly/TypeFly) with the [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim) to provide an opportunity to test the performance of LLM-controlled robots in simulated environments. This project allows users to interact with the [NVIDIA Kaya robot](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/assets/usd_assets_robots.html) through the TypeFly UI, where English instructions are interpreted by a LLM in order to control the movement of the Kaya robot in a user-defined simulated scene. This project was completed as the capstone for my summer internship with the Yale Efficient Computing Lab. 

## How to get started?
### Step 1: Setup your environment
1. First, make sure you have both TypeFly and the NVIDIA Isaac Sim installed and configured on your computer. To download and install TypeFly, you can follow the instructions [here](https://github.com/typefly/TypeFly). To download and install the Isaac Sim, first ensure that your computer meets the minimum specifications as listed [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html).Then, follow the steps for "Workstation Installation" as described [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html). 
2. After completing the install of TypeFly, cd into the **TypeFly** directory and update the `virtual_robot_wrapper.py` file to match the version in this repository.
Next, cd into the **isaac-sim-4.0.0** directory and download the `VRC` folder from this repository into this directory. After completing this action, the directory path to VRC should look like **isaac-sim-4.0.0/VRC**.

### Step 2: Define your scenario
1. In `parameters.py`, update the **BASE_SCENE_PATH** variable to be the path to the .usd file containing your pre-built scenario that you would like to test the robot in. 
By default, VeriFly uses the NVIDIA Kaya robot car with a holonomic controller, supporting movement in all directions. To build a scenario, you can first launch the Isaac Sim directly from the Omniverse App that you should already have on your computer following the installation guide linked above. After launching an instance of the Isaac Sim, you can search for assets from the NVIDIA Library (found in the bottom panel of the GUI) and drag and drop these assets into the viewport window. Once satisfied with your scenario, you can save the scenario as a .usd file to your computer, and update **BASE_SCENE_PATH** to be the path to this file. More detailed instructions on scenario building can be found [here](https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_intro_usd.html#isaac-sim-app-tutorial-intro-usd).
2. Then, you can further modify the variables in `parameters.py` as desired, such as by changing the **ROBOT_START_POSITION** variable to match the X,Y,Z coordinates of your desired initial location for the robot. A brief description of each variable's function is available in the file. 

### Step 3: Launch
1. In the terminal, ensure that the **OPENAI_API_KEY** and **VISION_SERVICE_IP** environment variables are set. Additionally, make sure to run `make SERVICE=yolo build` from the TypeFly directory on your local edge server. More detailed information can be found on [TypeFly](https://github.com/typefly/TypeFly)'s GitHub repository. 
2. Now, cd into the **isaac-sim-4.0.0/VRC** directory and run the command `./vrc.sh`. In the terminal will appear a link to a local web page hosting the TypeFly UI, which should now be connected to an instance of Issac Sim with a virtual Kaya robot running in your defined scenario. If the viewport appears black, please allow for a few minutes for the scenario to completely load. 
If you have opted for headless simulation (by setting this variable to "True" in the `parameters.py` file) then the simulation will automatically begin running without the Isaac Sim UI. If instead you have opted for non-headless simulation, then the Isaac Sim GUI will appear. From here you may continue to edit the scene and adjust the perspective camera until you press play. 
3. Once pressing play, you can now return to the TypeFly UI and converse with the LLM to pilot the robot. 

## Code Structure
```
 |-- isaac-sim-4.0.0             // Root directory containing all code relevant to NVIDIA Isaac Sim
    |-- VRC                      // Folder containing all code relevant to linking TypeFly with Isaac Sim
       |-- instructions.txt      // File to recieve MiniSpec instructions from TypeFly
       |-- parameters.py         // File to modify the simulation as desired. Please change the values of variables in this file to suit your specific needs.
       |-- video_generation.py   // File containing helper methods for converting synthetic data output into a video.
       |-- vrc.py                // Main file containing all code to run the Isaac Sim via Python API and contains code for deciphering and executing commands. 
       |-- vrc.sh                // File to setup and run Isaac Sim and TypeFly at the same time.

 |-- TypeFly                    
    |-- controller                  // Folder containing code relating to choosing a specific LLM controller. 
       |-- virtual_robot_wrapper.py // File to wrap low-level MiniSpec commands as movements in the simulator.
