# parameters.py
# Author: Sebastian Orozco
# Date: 08-23-2024
# Description: File to toggle parameters to Isaac Sim as desired.


# Toggle headless simulation
HEADLESS = False


# Toggle auto-play on startup (only if not headless)
# > This feature is relevant for if you would like to use the Isaac Sim GUI to modify the scene or
# > view the scene from a different perspective before beginning the simulation.
AUTOPLAY = True


# Define scene
BASE_SCENE_PATH = None
# Example: BASE_SCENE_PATH = "/home/sebastian/Documents/Virtual-Robot-Control-Using-LLMs/YECL-S24/VRC/demo.usd"


# Define intial position and rotation of the robot
ROBOT_START_POSITION = [2.1, 0.57, 0.0]
ROBOT_START_ORIENTATION = [2.1, 0.57, 0.0]


# Toggle whether to output video from synthetic data generated, and whether to set to real time
# > Please note that by default, videos will be generated according to simulated time (1 frame per 1 simulated second)
MAKE_VIDEO = False
REAL_TIME = False
