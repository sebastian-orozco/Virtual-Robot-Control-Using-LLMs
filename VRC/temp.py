# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# Modified by: Sebastian Orozco
# Date: 08-23-2024
# Description: Main program to launch simulator

from parameters import HEADLESS, AUTOPLAY, BASE_SCENE_PATH, ROBOT_START_POSITION, ROBOT_START_ORIENTATION

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

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


# CONFIGURE RENDERING AND PHYSICS
FPS = 1.0
OUTPUT_DIR = '_typefly_1fps'
SPEED = 60.0


# CREATE WORLD AND CONFIGURE SIMULATION
my_world = World(stage_units_in_meters=1.0)
simulation_context = SimulationContext()
simulation_context.set_simulation_dt(physics_dt=1.0 / SPEED, rendering_dt=1.0 / FPS)


# CREATE KAYA ROBOT
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
        position=np.array(ROBOT_START_POSITION),
        orientation=rot_utils.euler_angles_to_quats(np.array(ROBOT_START_ORIENTATION), degrees=True)
    )
)


# SETUP CAMERA, RENDER PRODUCT
camera = rep.create.camera(position=(0.84, 0, 0.08), rotation=(360, 0, 0), parent="/World/Kaya/base_link")
render_product = rep.create.render_product(camera, (512, 512))


# ADD SCENE TO WORLD
if not BASE_SCENE_PATH:
    my_world.scene.add_default_ground_plane() 
else:
    setting = add_reference_to_stage(usd_path=(BASE_SCENE_PATH), prim_path="/World")


# SETUP CONTROLLER FOR KAYA ROBOT
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
    max_angular_speed=0.005
)


# SETUP FOR WRITING SYNTHETIC DATA
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUTPUT_DIR, rgb=True, frame_padding=6, image_output_format="jpeg")
writer.attach([render_product])


# INITIALIZE BEFORE SIMULATION
t = 0.0 # Simulated time (in seconds)
reset_needed = False
meters_per_second = 0.13 
degrees_per_second = 8.0 


#=========================================================================================
# Note: The sole author of lines 118-330 is Sebastian Orozco.


# GLOBAL STATE FOR ACTIONS
actions_state = {
    'move_forward': {'done': False, 'time_end': 0.0},
    'move_backward': {'done': False, 'time_end': 0.0},
    'move_left': {'done': False, 'time_end': 0.0},
    'move_right': {'done': False, 'time_end': 0.0},
    # 'move_up': {'done': False, 'time_end': 0.0}, <- Not yet supported
    # 'move_down': {'done': False, 'time_end': 0.0}, <- Not yet supported
    'turn_cw': {'done': False, 'time_end': 0.0},
    'turn_c_cw': {'done': False, 'time_end': 0.0},
    'delay': {'done': False, 'time_end': 0.0},

}


# FLAG TO INDICATE WHETHER NEW INSTRUCTIONS HAVE BEEN ADDED
new_instructions_flag = False


# FUNCTION TO PARSE MINISPEC CODE
def parse_instruction(instruction):
    action_list = []

    commands = instruction.strip().split(';')

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


# DEFINE PATHS TO SIGNAL PIPES
B_to_A = '/tmp/input_pipe'
A_to_B = '/tmp/output_pipe'


# INITIALIZE ACTION QUEUE
action_queue = []


# DEFINE GENERAL HELPER TO CONVERT MINISPEC ACTION TO KAYA MOVEMENT USING HOLONOMIC CONTROLLER
def perform_movement(action_name, duration, command):
    global current_action, t
    state = actions_state[action_name]

    # Case (1): Action already performed -> do nothing
    if state['done']:
        return True  # Action complete
    
    # Case (3): Action has not yet been performed, AND it is our turn
    elif not state['done'] and state['time_end'] == 0.0:
        # Set ending time of our action, defined by the time at which we 
        # expect to have completed traveling the specified input distance
        state['time_end'] = duration + t

    # Continuation of Case (3): Actually perform action
    if not state['done']:
        # Check stopping condition (exceed max time)
        if t >= state['time_end']:
            state['done'] = True
            state['time_end'] = 0.0

            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()

            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=command))
            return False  # Action not complete
        

# DEFINE MINISPEC WRAPPERS
def move_forward(distance):
    return perform_movement('move_forward', (distance / meters_per_second), [0.1, 0.0, 0.0])

def move_backward(distance):
    return perform_movement('move_backward', (distance / meters_per_second), [-0.1, 0.0, 0.0])

def move_left(distance):
    return perform_movement('move_left', (distance / meters_per_second), [0.0, 0.1, 0.0])

def move_right(distance):
    return perform_movement('move_right', (distance / meters_per_second), [0.0, -0.1, 0.0])       

def turn_cw(degrees):
    return perform_movement('turn_cw', (degrees / degrees_per_second), [0.0, 0.0, 0.1])

def turn_c_cw(degrees):
    return perform_movement('turn_c_cw', (degrees / degrees_per_second), [0.0, 0.0, -0.1])
        
def delay(seconds):
    return perform_movement('delay', seconds, [0.0, 0.0, 0.0])
        

# HELPER FUNCTION TO RESET ACTIONS
def reset_action_state(action):
    global actions_state
    actions_state[action]['done'] = False
    actions_state[action]['time_end'] = 0.0


# HELPER TO PROCESS ACTIONS
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


# HELPER TO CHECK FOR NEW INSTRUCTIONS FROM TYPEFLY
def check_for_new_instructions():
    global action_queue

    print('checking for new instruction')

    with open(B_to_A, "r") as signal_pipe:
        signal = signal_pipe.readline().strip()
        if signal == 'STOP':
            print('STOP)')
            pass
        elif signal == 'READY':
            print('READY')
            pass
            # Read the instructions
        print(f"Received signal from B: {signal}")

    with open("standalone_examples/api/omni.isaac.kaya/IPC/instructions.txt", "r") as ins_pipe:
        instructions = ins_pipe.readline().strip()        
        new_actions = parse_instruction(instructions)  # Adjusted to pass input_data directly
        action_queue.extend(new_actions)
        print('just extended action queue')
        print(action_queue)


if not HEADLESS and AUTOPLAY:
    my_world.reset()


#=========================================================================================


# RUN SIMULATION
wall_time_start = 0.0
ready = False
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
        
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
            t = 0.0
        
        t = simulation_context.current_time 

        if t >= 10.0 and not process_next_action():
            check_for_new_instructions()

simulation_app.close()
