# Authors: Guojun Chen, Sebastian Orozco
# Date: 08-23-2024
# Description: wrapper file to execute low-level commands in the Isaac Sim


import cv2
from typing import Tuple
from .abs.robot_wrapper import RobotWrapper
import os
import time


# Path to the instructions file
instructions_path = os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-4.0.0/VRC/instructions.txt")


# Define paths to signaling pipes
typefly_to_isaac_sim = '/tmp/input_pipe'
isaac_sim_to_typefly = '/tmp/output_pipe'


# Define helper to send instructions to the Isaac Sim
def send_instructions(instructions):
    with open(instructions_path, "w") as ins_pipe:
        ins_pipe.write(instructions)
   
    # Signal Isaac Sim
    with open(typefly_to_isaac_sim, "w") as signal_pipe:
        signal_pipe.write("READY\n")
        signal_pipe.flush()


# Define helper to recieve instructions from the Isaac Sim
def wait_for_signal_from_isaac_sim():
    with open(isaac_sim_to_typefly, "r") as signal_pipe:
        while True:
            signal = signal_pipe.readline().strip()
            if signal == "DONE":
                break


# Class to read synthetic data from Isaac Sim as frames
class FrameReader:
    def __init__(self, folder_path):
        # Initialize frame reading from a folder
        self.folder_path = folder_path
        self.frame_index = 0

    @property
    def frame(self):
        # Read the next frame from the folder
        filename = f"rgb_{self.frame_index + 1:06d}.jpeg"
        frame_path = os.path.join(self.folder_path, filename)
        
        while not os.path.isfile(frame_path):
            # print(f"Frame {filename} not found. Retrying...")
            time.sleep(1)  # Wait for 1 second before retrying
            filename = f"rgb_{self.frame_index + 1:06d}.jpeg"
            frame_path = os.path.join(self.folder_path, filename)

        frame = cv2.imread(frame_path, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"Could not read frame {filename}")
        
        # Attempt to delete the current frame and the previous frame
        try:
            pass
            # os.remove(frame_path)
            # print(f"Deleted frame: {filename}")
        except Exception as e:
            print(f"Failed to delete frame {filename}: {e}")
        
        prev_filename = f"rgb_{self.frame_index + 0:06d}.jpeg"
        prev_frame_path = os.path.join(self.folder_path, prev_filename)
        if os.path.isfile(prev_frame_path):
            try:
                pass
                # os.remove(prev_frame_path)
                # print(f"Deleted previous frame: {prev_filename}")
            except Exception as e:
                print(f"Failed to delete previous frame {prev_filename}: {e}")

        self.frame_index += 2
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    

# Class to wrap Isaac Sim functionality to high-level commands
class VirtualRobotWrapper(RobotWrapper):
    def __init__(self):
        self.stream_on = False

        #added by me
        self.stream_on = False
        self.cap = None
        self.frame_reader = None  # Initialize frame_reader attribute
        pass

    def keep_active(self):
        pass

    def connect(self):
        pass

    def takeoff(self) -> bool:
        return True

    def land(self):
        pass

    def start_stream(self):
        self.cap = cv2.VideoCapture(0)
        self.stream_on = True

    def stop_stream(self):
        self.cap.release()
        self.stream_on = False

    def get_frame_reader(self):
        if not self.stream_on:
            return None
        
        if self.frame_reader is None:
            self.frame_reader = FrameReader(os.path.expanduser("~/omni.replicator_out/_typefly_1fps"))
        
        return self.frame_reader

    def move_forward(self, distance: int) -> Tuple[bool, bool]:
        print(f"-> Moving forward {distance} cm")
        self.movement_x_accumulator += distance
        send_instructions(f'mf({distance});')
        wait_for_signal_from_isaac_sim()
        return True, False

    def move_backward(self, distance: int) -> Tuple[bool, bool]:
        print(f"-> Moving backward {distance} cm")
        self.movement_x_accumulator -= distance
        send_instructions(f'mb({distance});')
        wait_for_signal_from_isaac_sim()
        return True, False

    def move_left(self, distance: int) -> Tuple[bool, bool]:
        print(f"-> Moving left {distance} cm")
        self.movement_y_accumulator += distance
        send_instructions(f'ml({distance});')
        wait_for_signal_from_isaac_sim()
        return True, False

    def move_right(self, distance: int) -> Tuple[bool, bool]:
        print(f"-> Moving right {distance} cm")
        self.movement_y_accumulator -= distance
        send_instructions(f'mr({distance});')
        wait_for_signal_from_isaac_sim()
        return True, False

    def move_up(self, distance: int) -> Tuple[bool, bool]:
        print(f"-> Moving up {distance} cm")
        return True, False

    def move_down(self, distance: int) -> Tuple[bool, bool]:
        print(f"-> Moving down {distance} cm")
        return True, False

    def turn_ccw(self, degree: int) -> Tuple[bool, bool]:
        print(f"-> Turning CCW {degree} degrees")
        self.rotation_accumulator += degree
        if degree >= 90:
            print("-> Turning CCW over 90 degrees")
            send_instructions(f'tu({degree});')
            wait_for_signal_from_isaac_sim()
            return True, True
        send_instructions(f'tu({degree});')
        wait_for_signal_from_isaac_sim()
        return True, False

    def turn_cw(self, degree: int) -> Tuple[bool, bool]:
        print(f"-> Turning CW {degree} degrees")
        self.rotation_accumulator -= degree
        if degree >= 90:
            print("-> Turning CW over 90 degrees")
            send_instructions(f'tc({degree});')
            wait_for_signal_from_isaac_sim()
            return True, True
        send_instructions(f'tc({degree});')
        wait_for_signal_from_isaac_sim()
        return True, False
    
    def move_in_circle(self, cw) -> Tuple[bool, bool]:
        pass
