import sys
import signal
from typing import List

import asyncio
import numpy as np
import carb

# Create app simulator
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Enable browser extension
import omni.kit.app
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("omni.simready.explorer", True)
print(manager.is_extension_enabled("omni.simready.explorer"))

from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.bounds import compute_aabb, create_bbox_cache
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

import omni.simready.explorer as sre
import omni.usd
from pxr import Gf

'''Global variables'''
PATH_TO_BASE = "/Isaac/Environments/Simple_Room/simple_room.usd"
PATH_TO_DONUT = "/Isaac/Environments/Office/Props/SM_Donut.usd"

'''Helper functions'''
def deg_to_quat(degrees):
    """Convert degrees into quaternion"""
    angle_radians = np.radians(degrees)
    cos_half_angle = np.cos(angle_radians / 2)
    sin_half_angle = np.sin(angle_radians / 2)
    return [cos_half_angle, 0, 0, sin_half_angle]


async def browse():
    """Browse and add residential wooden chair assets"""
    browser = sre.get_instance()
    browser._show_window(visible=True)
    assets = await sre.find_assets(search_words=["chair", "dellwood"])
    print(f"Found {len(assets)} chairs")

    for asset in assets:
        res, prim_path = sre.add_asset_to_stage(
            asset.main_url, position=Gf.Vec3d(-1.2216, 0.13531, -0.7695))
        if res:
            print(f"Added '{prim_path}' from '{asset.main_url}'")


def signal_handler(sig, frame):
    """Handle termination signals for graceful exit"""
    print("Exiting gracefully")
    simulation_app.close()
    sys.exit(0)


'''Setup/configuration'''
# Find assets in nucleus server
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

print(f"Nucleus server path is: {assets_root_path}")


'''Build world and base setting'''
# Make a world
my_world = World(stage_units_in_meters=1.0)

# Add base setting to world
usd_path = assets_root_path + PATH_TO_BASE
setting = add_reference_to_stage(usd_path=usd_path, prim_path="/World")

# Add a donut
donut_path = assets_root_path + PATH_TO_DONUT
donut = add_reference_to_stage(usd_path=donut_path, prim_path="/World/Donut")

# Link it to an XForm prim
donut_system = my_world.scene.add(XFormPrim(prim_path="/World/Donut", position=[0, 0, 0.01040764], orientation=deg_to_quat(27)))


'''Getting the bounding box of a table'''
# Get the table
ground = XFormPrim(prim_path="/World/table_low_327")
cache = create_bbox_cache()
print(compute_aabb(bbox_cache=cache, prim_path="/World/table_low_327"))
print(f"{ground}")

# Get translation of table
values = prims.get_prim_attribute_names("/World/table_low_327")
print(values)

trans = prims.get_prim_attribute_value("/World/table_low_327", "xformOp:translate")
print(trans)


'''Asynchronous browsing of assets'''
asyncio.ensure_future(browse())

'''Signal handling for graceful exit'''
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

'''Keep the simulation running'''
while True:
    my_world.step(render=True)
