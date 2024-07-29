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
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.appwindow import get_default_app_window


# SET FRAMES PER SECOND, PATHS FOR BASE SCENE & PROP
FPS = 20.0
if False:
    BASE_SCENE_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
else:
    BASE_SCENE_PATH = None
BASE_PROP_PATH = "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01"

# CREATE WORLD
my_world = World(stage_units_in_meters=1.0)

simulation_context = SimulationContext()

simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / FPS)


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
        position=np.array([0, 0.0, 0.02]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
)


# SETUP CAMERA, RENDER PRODUCT
camera = rep.create.camera(position=(0.07, 0, 0.08), rotation=(360, 0, 0), parent="/World/Kaya/base_link")

render_product = rep.create.render_product(camera, (1024, 1024))


# ADD SCENE TO WORLD
# (Not necessary if scene has been defined.)
if not BASE_SCENE_PATH:
    my_world.scene.add_default_ground_plane() 
else:
    setting = add_reference_to_stage(usd_path=(assets_root_path + BASE_SCENE_PATH), prim_path="/World")


# ADD CUBE TO WORLD
SHELF_TRANSLATION = np.array([-8.96649, 0.0, 1.54662])
SHELF_ORIENTATION = rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, -39.755]), degrees=True)
FALL_TRANSLATION = np.array([-8.38833, 0.0, 1.54662])
FALL_ORIENTATION = rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, -39.755]), degrees=True)

fancy_cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube", # The prim path of the cube in the USD stage
        name="fancy_cube", # The unique name used to retrieve the object from the scene later on
        position=SHELF_TRANSLATION, # Using the current stage units which is in meters by default.
        orientation=SHELF_ORIENTATION,
        scale=np.array([0.5015, 0.5015, 0.5015]), # most arguments accept mainly numpy arrays.
        color=np.array([0.89, 0.59, 0.24]), # RGB channels, going from 0-1
    ))

# DEFINE FUNCTION TO TRIGGER CUBE EVENT
def keyboard_event(event, *args, **kwargs):
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.SPACE:
            fancy_cube.set_local_pose(translation=FALL_TRANSLATION, orientation=FALL_ORIENTATION)

# # Subscribe to keyboard event
appwindow = get_default_app_window()
input = carb.input.acquire_input_interface()
input.subscribe_to_keyboard_events(appwindow.get_keyboard(), keyboard_event)


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
)


# SETUP WRITER
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="_output_test", rgb=True)
writer.attach([render_product])
rep.orchestrator.preview()


# INITIALIZE BEFORE SIMULATION
t = 0.0 # sim time
i = 0   # sim steps
reset_needed = False
direction = 'forward'
physics_dt = 0.0
rendering_dt = 0.0

# RUN SIMULATION
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
        
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
            t = 0.0
            fancy_cube.set_local_pose(translation=SHELF_TRANSLATION, orientation=SHELF_ORIENTATION) # Move cube back to original position
        
        t = simulation_context.current_time

        # if t >= 2.0:
        #     print('Stopping...')
        #     my_world.stop()

        if t % 60.0 <= physics_dt:
            pass # clear images or something

        if t >= 125:
            direction = 'right'

        if t >= 145:
            direction == 'backward'

        # Move forward
        if i >= 0 and direction == 'forward':
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.1, 0.0, 0.0]))

        elif i >= 0 and direction == 'right':
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, -0.1, 0.0]))

        elif i >= 0 and direction == 'backward':
            my_kaya.apply_wheel_actions(my_controller.forward(command=[-0.1, 0.0, 0.0]))

        # # Move backward
        # elif i >= 500 and i < 1000:
        #     my_kaya.apply_wheel_actions(my_controller.forward(command=[-0.1, 0.0, 0.0]))

        # # Move left
        # elif i >= 1000 and i < 1500:
        #     my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.1, 0.0]))

        # # Move right
        # elif i >= 1500 and i < 2000:
        #     my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, -0.1, 0.0]))

        # # Reset
        # elif i == 2000:
        #     i = 0
        
        # Increment
        i += 1

        # Check if sim is slowing down
        if simulation_context.get_physics_dt() != physics_dt:
            physics_dt = simulation_context.get_physics_dt()
            print('New physics dt: ', physics_dt)

        if simulation_context.get_rendering_dt() != rendering_dt:
            rendering_dt = simulation_context.get_rendering_dt()
            print('New rendering dt: ', rendering_dt)


simulation_app.close()
