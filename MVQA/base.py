import sys
import signal
from typing import List
import carb
import asyncio

# Create app simulator
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Enable browser extension
import omni.kit.app
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("omni.simready.explorer", True)
print(manager.is_extension_enabled("omni.simready.explorer"))

import omni.simready.explorer as sre
import omni.usd
browser = sre.get_instance()
browser._show_window(visible=True)

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Helper functions
import scene
import props

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
setting = scene.add_base_scene(my_world, assets_root_path=assets_root_path)

hi = {"object":"chair",
 "synonyms":["chair","steat","stool","ottoman","bench"],
 "position":[1,1,0],
 "rotation": 27}

'''Query for add one prop at a time'''
asyncio.ensure_future(props.browse(my_world, hi))

'''Signal handling for graceful exit'''
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


'''Keep the simulation running'''
while True:
    my_world.step(render=True)
