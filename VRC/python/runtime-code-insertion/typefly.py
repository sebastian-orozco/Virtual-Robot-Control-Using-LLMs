# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

# Initialize the simulation application with headless mode set to False
simulation_app = SimulationApp({"headless": False}) # add clarifying comment about purpose/ meaning of headless

import os
import time
import carb
import numpy as np
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core import World
from omni.isaac.sensor import Camera
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
import omni.replicator.core as rep
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.appwindow import get_default_app_window
from parameters import FPS, OUTPUT_DIR, SPEED, INSTRUCTIONS


# Set paths for rendering configuration, base scene, and prop
if False:
    BASE_SCENE_PATH = "/home/sebastian/Documents/Virtual-Robot-Control-Using-LLMs/YECL-S24/VRC/demo.usd" # <- demo factory scenario
else:
    BASE_SCENE_PATH = None


# Create the simulation world with the specified units and desired rendering frequency
my_world = World(stage_units_in_meters=1.0)
simulation_context = SimulationContext()
simulation_context.set_simulation_dt(physics_dt=1.0 / SPEED, rendering_dt=1.0 / FPS) # <- test changing speed at runtime once back in person


# Initialize starting location of robot
ROBOT_START_POSITION = np.array([0.0, 0.0, 0.0])
ROBOT_START_ORIENTATION = rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 180.0]), degrees=True)

# Define the path to the Kaya robot's USD file and add the robot to the world
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

kaya_asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"
my_kaya = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Kaya",
        name="my_kaya",
        wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
        create_robot=True,
        usd_path=kaya_asset_path,
        position=ROBOT_START_POSITION,
        orientation=ROBOT_START_ORIENTATION
    )
)


# Set up the camera and render product for the simulation
camera = rep.create.camera(position=(0.84, 0, 0.08), rotation=(360, 0, 0), parent="/World/Kaya/base_link")
render_product = rep.create.render_product(camera, (512, 512))


# Add the scene to the world or a default ground plane if no scene is defined
if not BASE_SCENE_PATH:
    my_world.scene.add_default_ground_plane() 
else:
    setting = add_reference_to_stage(usd_path=(BASE_SCENE_PATH), prim_path="/World")


# Set up the controller for the Kaya robot
kaya_setup = HolonomicRobotUsdSetup(
    robot_prim_path=my_kaya.prim_path, com_prim_path="/World/Kaya/base_link/control_offset"
)
(
    wheel_radius,
    wheel_positions,
    wheel_orientations,
    mecanum_angles,
    wheel_axis,
    up_axis,
) = kaya_setup.get_holonomic_controller_params()
my_controller = HolonomicController(
    name="holonomic_controller",
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
    max_angular_speed=0.01
)


# Set up a writer to capture the simulation output
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUTPUT_DIR, rgb=True, frame_padding=6, image_output_format="jpeg")
# writer.initialize(output_dir=OUTPUT_DIR, distance_to_camera=True, colorize_depth=True) # <- if depth data is desired over camera data
writer.attach([render_product])


# Initialize simulation variables
t = 0.0 # Simulation time

reset_needed = False
physics_dt = 0.0
rendering_dt = 0.0

meters_per_second = 0.13 
degrees_per_second = 16.0

#=========================================================================================
# RUNTIME CODE INSERTION
#=========================================================================================

# Define global state for actions
actions_state = {
    'move_forward': {'done': False, 'time_end': 0.0},
    'move_backward': {'done': False, 'time_end': 0.0},
    'move_left': {'done': False, 'time_end': 0.0},
    'move_right': {'done': False, 'time_end': 0.0},
    # 'move_up': {'done': False, 'time_end': 0.0}, <- not yet supported
    # 'move_down': {'done': False, 'time_end': 0.0}, <- not yet supported
    'turn_cw': {'done': False, 'time_end': 0.0},
    'turn_c_cw': {'done': False, 'time_end': 0.0},
    'delay': {'done': False, 'time_end': 0.0},

}


# Global flag to indicate whether new instructions have been added
new_instructions_flag = False

# Function to parse Minispec command instructions from a file
def parse_instructions(file_path):
    action_list = []
    with open(file_path, 'r') as file:
        line = file.readline().strip()
        commands = line.split(';')

        command_map = {
            'mf': 'move_forward',
            'mb': 'move_backward',
            'ml': 'move_left',
            'mr': 'move_right',
            'tc': 'turn_cw',
            'tu': 'turn_c_cw',
            'd': 'delay'
        }

        for command in commands:
            if command:
                command_parts = command.split('(')
                if len(command_parts) != 2:
                    # Handle improperly formatted commands
                    print(f"Ignoring improperly formatted command: {command}")
                    continue
                
                action_name = command_parts[0].strip()
                if action_name not in command_map:
                    # Handle unsupported commands
                    print(f"Ignoring unsupported command: {command}")
                    continue
                
                try:
                    param_str = command_parts[1][:-1].strip()
                    if action_name in ['ml', 'mr', 'mf', 'mb']:  # Convert cm to meters
                        param = float(param_str) / 100.0  # Convert cm to meters
                    elif action_name == 'tc' or action_name == 'tu':
                        param = float(param_str)
                    elif action_name == 'd':
                        param = float(param_str) / 1000.0  # Convert milliseconds to seconds

                    action_list.append((command_map[action_name], param))
                except ValueError:
                    print(f"Error parsing parameter in command: {command}")
                    continue

    return action_list


# Load and parse instructions from the specified file 
# TODO: update this to not be hardcoded
relative_path = f'standalone_examples/api/omni.isaac.kaya/runtime-code-insertion/instructions/{INSTRUCTIONS}'
instructions_file_path = os.path.join(os.getcwd(), relative_path)


# Get list of actions to perform
action_queue = parse_instructions(instructions_file_path)


# Define Minispec wrappers
def move_forward(distance):
    global current_action, t
    
    state = actions_state['move_forward']

    # Case (1): Action already performed 
    #       -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn 
    elif not state['done'] and state['time_end'] == 0.0: 
        # Set ending time of our action, defined by the time at which we 
        #       expect to have completed traveling the specified input distance
        state['time_end'] = (distance / meters_per_second) + t

    # Continuation of Case (3): Actually perform action
    if not state['done']: 
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.1, 0.0, 0.0]))
            return False  # Action not complete


def move_backward(distance):
    global current_action, t
    
    state = actions_state['move_backward']

    # Case (1): Action already performed 
    #       -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn 
    elif not state['done'] and state['time_end'] == 0.0: 
        # Set ending time of our action, defined by the time at which we 
        #       expect to have completed traveling the specified input distance
        state['time_end'] = (distance / meters_per_second) + t

    # Continuation of Case (3): Actually perform action
    if not state['done']: 
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[-0.1, 0.0, 0.0]))
            return False  # Action not complete


def move_left(distance):
    global current_action, t
    
    state = actions_state['move_left']

    # Case (1): Action already performed 
    #       -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn 
    elif not state['done'] and state['time_end'] == 0.0: 
        # Set ending time of our action, defined by the time at which we 
        #       expect to have completed traveling the specified input distance
        state['time_end'] = (distance / meters_per_second) + t

    # Continuation of Case (3): Actually perform action
    if not state['done']: 
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.1, 0.0]))
            return False  # Action not complete
        

def move_right(distance):
    global current_action, t
    
    state = actions_state['move_right']

    # Case (1): Action already performed 
    #       -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn 
    elif not state['done'] and state['time_end'] == 0.0: 
        # Set ending time of our action, defined by the time at which we 
        #       expect to have completed traveling the specified input distance
        state['time_end'] = (distance / meters_per_second) + t

    # Continuation of Case (3): Actually perform action
    if not state['done']: 
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, -0.1, 0.0]))
            return False  # Action not complete
        

def turn_cw(degrees):
    global current_action, t
    
    state = actions_state['turn_cw']

    # Case (1): Action already performed 
    #       -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn 
    elif not state['done'] and state['time_end'] == 0.0: 
        # Set ending time of our action, defined by the time at which we 
        #       expect to have completed traveling the specified input degrees
        state['time_end'] = (degrees / degrees_per_second) + t

    # Continuation of Case (3): Actually perform action
    if not state['done']: 
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.1]))
            return False  # Action not complete
        

def turn_c_cw(degrees):
    global current_action, t
    
    state = actions_state['turn_c_cw']

    # Case (1): Action already performed 
    #       -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn 
    elif not state['done'] and state['time_end'] == 0.0: 
        # Set ending time of our action, defined by the time at which we 
        #       expect to have completed traveling the specified input degrees
        state['time_end'] = (degrees / degrees_per_second) + t

    # Continuation of Case (3): Actually perform action
    if not state['done']: 
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, -0.1]))
            return False  # Action not complete
        

def delay(seconds):
    global current_action, t
    
    state = actions_state['delay']

    # Case (1): Action already performed 
    #       -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn 
    elif not state['done'] and state['time_end'] == 0.0: 
        # Set ending time of our action, defined by the time at which we 
        #       expect to have completed traveling the specified input distance
        state['time_end'] = seconds + t

    # Continuation of Case (3): Actually perform action
    if not state['done']: 
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.0]))
            return False  # Action not complete
        

# Reset the state of an action to its initial state
def reset_action_state(action):
    global actions_state
    actions_state[action]['done'] = False
    actions_state[action]['time_end'] = 0.0


# Process the next action in the queue
def process_next_action():
    global action_queue

    if not action_queue:
        return False # No more actions

    action, param = action_queue[0]

    if action == 'move_forward':
        if move_forward(param):
            action_queue.pop(0)
            reset_action_state('move_forward')
    elif action == 'move_backward':
        if move_backward(param):
            action_queue.pop(0)
            reset_action_state('move_backward')
    elif action == 'move_left':
        if move_left(param):
            action_queue.pop(0)
            reset_action_state('move_left')
    elif action == 'move_right':
        if move_right(param):
            action_queue.pop(0)
            reset_action_state('move_right')
    elif action == 'turn_cw':
        if turn_cw(param):
            action_queue.pop(0)
            reset_action_state('turn_cw')
    elif action == 'turn_c_cw':
        if turn_c_cw(param):
            action_queue.pop(0)
            reset_action_state('turn_c_cw')
    elif action == 'delay':
        if delay(param):
            action_queue.pop(0)
            reset_action_state('delay')

    return True


# Check for new instructions and update the action queue
def check_for_new_instructions():
    # open new instructions flag file
    # TODO: fix this to be in-line with expected behavior of minispec
    # overwrite file with F
    # if new instruction, we want to update flag here
    global new_instructions_flag, action_queue
    if new_instructions_flag:
        new_actions = parse_instructions(instructions_file_path)
        action_queue.extend(new_actions)
        new_instructions_flag = False

#=========================================================================================
  
# Define a function to handle keyboard events
def keyboard_event(event, *args, **kwargs):
    global new_instructions_flag
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.T:
            print(simulation_context.current_time)
        if event.input == carb.input.KeyboardInput.N:
            new_instructions_flag = True


# # Subscribe to keyboard event
appwindow = get_default_app_window()
input = carb.input.acquire_input_interface()
input.subscribe_to_keyboard_events(appwindow.get_keyboard(), keyboard_event)

my_world.reset()

# Main simulation loop
wall_time_start = 0.0
ready = False
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
        
    if my_world.is_playing():
        if wall_time_start == 0.0:
            wall_time_start = time.time()

        if reset_needed:
            my_world.reset()
            reset_needed = False
            t = 0.0
        

        t = simulation_context.current_time 

        check_for_new_instructions()

        if not process_next_action():
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.0])) # <- no movement by default

simulation_app.close()
