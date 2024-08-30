import time
import os
from PIL import Image 
from tqdm import tqdm
import ffmpeg
from parameters import REAL_TIME

# Set framerate
frame_rate = 1.0

if REAL_TIME:
    frame_rate = 60.0


# DEFINE HELPER TO CONVERT CAMERA DATA OUTPUT INTO VIDEO
def images_to_video(folder_path, output_path):
    images_pattern = os.path.join(folder_path, 'rgb_%06d.jpeg')  

    codec = 'libx264' 
    video_options = {
        'crf': '18',  
        'preset': 'slow',  
        'pix_fmt': 'yuv420p'  
    }

    # Run ffmpeg command to convert images to video
    (
        ffmpeg
        .input(images_pattern, format='image2', framerate=frame_rate) 
        .output(output_path, **video_options, vcodec=codec)
        .run()
    )

    print(f"Video saved: {output_path}")


# Set the path to your local folder containing images
folder_path = os.path.expanduser("~/omni.replicator_out/_typefly_1fps")
output_path = os.path.expanduser("~/omni.replicator_out/output_video.mp4")


# Create video from images
images_to_video(folder_path, output_path)