# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

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
from parameters import FPS, OUTPUT_DIR, SPEED
import fcntl

# SET PATHS FOR RENDERING CONFIG, BASE SCENE & PROP
# FPS = 20.0
# REFRESH_INTERVAL = 1
if True:
    BASE_SCENE_PATH = "/home/sebastian/Documents/Virtual-Robot-Control-Using-LLMs/YECL-S24/VRC/deprecated/demo.usd"
else:
    BASE_SCENE_PATH = None
# BASE_PROP_PATH = "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01"


# CREATE WORLD
my_world = World(stage_units_in_meters=1.0)

simulation_context = SimulationContext()

simulation_context.set_simulation_dt(physics_dt=1.0 / SPEED, rendering_dt=1.0 / FPS)


# CREATE KAYA ROBOT
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# ROBOT_START_POSITION = np.array([-25.001, 7.492, 0.02])
ROBOT_START_POSITION = np.array([0.0, 0.0, 0.0])

ROBOT_START_ORIENTATION = rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 180.0]), degrees=True)

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


# SETUP CAMERA, RENDER PRODUCT
camera = rep.create.camera(position=(0.84, 0, 0.08), rotation=(360, 0, 0), parent="/World/Kaya/base_link")
# camera = rep.create.camera(position=(-21.1, 11.87, 20.83), rotation=(0, -90, 90), parent="/World")

render_product = rep.create.render_product(camera, (512, 512))


# ADD SCENE TO WORLD
# (Not necessary if scene has been defined.)
if not BASE_SCENE_PATH:
    my_world.scene.add_default_ground_plane() 
else:
    # setting = add_reference_to_stage(usd_path=(assets_root_path + BASE_SCENE_PATH), prim_path="/World")
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
    max_angular_speed=0.01
)


# SETUP WRITER
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUTPUT_DIR, rgb=True, frame_padding=6, image_output_format="jpeg")
# writer.initialize(output_dir=OUTPUT_DIR, distance_to_camera=True, colorize_depth=True)
writer.attach([render_product])
# rep.orchestrator.preview()

# INITIALIZE BEFORE SIMULATION
t = 0.0 # sim time
i = 0   # sim steps

reset_needed = False
physics_dt = 0.0
rendering_dt = 0.0

meters_per_second = 0.13 
degrees_per_second = 16.0

#=========================================================================================
# RUNTIME CODE INSERTION
#=========================================================================================

# Global state for actions
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


# Global flag to indicate new instructions have been added
new_instructions_flag = False
# last_parsed_line = 0


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


# DEFINE MINISPEC WRAPPERS
B_to_A = '/tmp/input_pipe'
A_to_B = '/tmp/output_pipe'

action_queue = []
# init = parse_instruction('d(1000);')  
# action_queue.extend(init)

def move_forward(distance):
    global current_action, t
    # print('t = ', t)
    
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

            # with open(output_pipe_path, 'w') as output_pipe:
            #     output_pipe.write('True')
            #     output_pipe.flush()

            # with open(output_pipe_path, 'w') as file:
            #     file.write('True')

            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()

            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.1, 0.0, 0.0]))
            return False  # Action not complete


def move_backward(distance):
    global current_action, t
    # print('t = ', t)
    
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

            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()

            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[-0.1, 0.0, 0.0]))
            return False  # Action not complete


def move_left(distance):
    global current_action, t
    # print('t = ', t)
    
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

            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()

            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.1, 0.0]))
            return False  # Action not complete
        

def move_right(distance):
    global current_action, t
    # print('t = ', t)
    
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
            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, -0.1, 0.0]))
            return False  # Action not complete
        

def turn_cw(degrees):
    global current_action, t
    # print('t = ', t)
    
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
            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.1]))
            return False  # Action not complete
        

def turn_c_cw(degrees):
    global current_action, t
    # print('t = ', t)
    
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
            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, -0.1]))
            return False  # Action not complete
        

def delay(seconds):
    global current_action, t
    # print('t = ', t)
    
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
            with open(A_to_B, "w") as signal_pipe:
                signal_pipe.write("DONE\n")
                signal_pipe.flush()
            return True  # Action complete
        
        # Perform specified movement
        else:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.0]))
            return False  # Action not complete
        

def reset_action_state(action):
    global actions_state
    actions_state[action]['done'] = False
    actions_state[action]['time_end'] = 0.0



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


# def check_for_new_instructions():
#     global action_queue

#     with open(input_pipe_path, 'r') as input_pipe:
#         # Read input from program 2
#         input_data = input_pipe.readline().strip()
#         if input_data:
#             print(f"Program 1 received: {input_data}")
#             new_actions = parse_instructions(input_pipe)
#             action_queue.extend(new_actions)
#             print('just extended action queue')
#             print(action_queue)


# def check_for_new_instructions():
#     global action_queue

#     print('checking for new instruction')

#     try:
#         # # Use subprocess to run the command with shell=True
#         # subprocess.run(command, shell=True, check=True)
        
#         # Run 'cat' again to check for new input
#         result = subprocess.run(['cat', input_pipe_path], capture_output=True, text=True, check=True)
#         input_data = result.stdout.strip()

#         if input_data:
#             print(f"Program 1 received: {input_data}")
#             # new_actions = parse_instructions(input_pipe_path)  # Adjusted to pass input_data directly
#             new_actions = parse_instruction(input_data)  # Adjusted to pass input_data directly
#             action_queue.extend(new_actions)
#             print('just extended action queue')
#             print(action_queue)

#     except subprocess.CalledProcessError as e:
#         print(f"An error occurred while running the command: {e}")
#     except FileNotFoundError as e:
#         print(f"The file or command was not found: {e}")
#     except Exception as e:
#         print(f"An unexpected error occurred: {e}")


def check_for_new_instructions():
    global action_queue

    print('checking for new instruction')

    with open(B_to_A, "r") as signal_pipe:
        signal = signal_pipe.readline().strip()
        # if signal == 'STOP':
        #     pass
        # elif signal == 'READY':
        #     pass
        print(f"Received signal from B: {signal}")
   
    # Read the instructions
    with open("standalone_examples/api/omni.isaac.kaya/IPC/instructions.txt", "r") as ins_pipe:
        instructions = ins_pipe.readline().strip()        
        new_actions = parse_instruction(instructions)  # Adjusted to pass input_data directly
        action_queue.extend(new_actions)
        print('just extended action queue')
        print(action_queue)



#=========================================================================================

my_world.reset()

# RUN SIMULATION
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

        # check if world has loaded yet
        # if time.time() - wall_time_start < 60.0:
        #     print(time.time())

        if not process_next_action():
            # my_world.stop()
            print('no actions') 
            
            check_for_new_instructions()

            # my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.0]))

simulation_app.close()
