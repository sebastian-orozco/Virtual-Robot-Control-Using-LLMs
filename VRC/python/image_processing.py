import time
import os
from PIL import Image 
from tqdm import tqdm
import ffmpeg
from parameters import FPS, REFRESH_INTERVAL, OUTPUT_DIR


# DEFINE HELPER TO PROCESS IMAGE FILES
def process_and_delete_images(folder_path, batch_size=int(FPS * REFRESH_INTERVAL), retry_interval=5):
    processed_count = 0  # Initialize the processed images counter
    last_processed_index = -1  # Initialize the last processed index

    try:
        while True:
            try:
                # Step 1: Read all image files from the folder
                files = os.listdir(folder_path)
                image_files = sorted([f for f in files if f.lower().endswith('.png') and f.startswith('rgb_')],
                                     key=lambda x: int(x.split('_')[1].split('.')[0]))

                # Step 2: Process each image file starting from the last processed index + 1
                for image_file in image_files:
                    current_index = int(image_file.split('_')[1].split('.')[0])
                    if current_index <= last_processed_index:
                        continue

                    image_path = os.path.join(folder_path, image_file)
                    print(f"Processing image: {image_path}")
                    try:
                        # Open image (optional, only if you need to process them)
                        with Image.open(image_path) as img:
                            # Perform any processing on the image here
                            pass  # Replace with your image processing code

                        #yolo world ?

                        # Track the most recently processed index
                        last_processed_index = current_index
                        processed_count += 1

                        # Check if we have processed 'batch_size' images
                        if processed_count >= batch_size:
                            # Delete all processed image files
                            for file_index in range(last_processed_index - batch_size + 1, last_processed_index + 1):
                                file_to_delete = os.path.join(folder_path, f"rgb_{file_index:04d}.png")
                                try:
                                    os.remove(file_to_delete)
                                    print(f"Deleted image: {file_to_delete}")
                                except Exception as e:
                                    print(f"Failed to delete image {file_to_delete}: {e}")

                            # Reset counter
                            processed_count = 0

                    except Exception as e:
                        print(f"Failed to process image {image_path}: {e}")

            except FileNotFoundError as e:
                print(f"Error: {e}. Retrying in {retry_interval} seconds...")
                time.sleep(retry_interval)
                continue

            # Step 3: Short wait before next iteration
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Cleaning up...")

    finally:
        # Delete any remaining processed image files
        for file_index in range(last_processed_index - processed_count + 1, last_processed_index + 1):
            file_to_delete = os.path.join(folder_path, f"rgb_{file_index:04d}.png")
            try:
                os.remove(file_to_delete)
                print(f"Deleted image: {file_to_delete}")
            except Exception as e:
                print(f"Failed to delete image {file_to_delete}: {e}")
        print("Cleanup completed. Exiting...")


# DEFINE HELPER TO CONVERT CAMERA DATA OUTPUT INTO VIDEO
def images_to_video(folder_path, output_path):
    images_pattern = os.path.join(folder_path, 'rgb_%04d.png')  # Adjust the pattern as per your file naming

    # Output video codec and options
    codec = 'libx264'  # You can change this to another codec if needed
    video_options = {
        'crf': '18',  # Adjust the quality here if needed
        'preset': 'slow',  # Adjust the encoding speed vs compression ratio
        'pix_fmt': 'yuv420p'  # Required for compatibility
    }

    # Run ffmpeg command to convert images to video
    (
        ffmpeg
        .input(images_pattern, format='image2', framerate=FPS)  # Adjust framerate as needed
        .output(output_path, **video_options, vcodec=codec)
        .run()
    )

    print(f"Video saved: {output_path}")


# MAIN
# Set the path to your local folder containing images
folder_path = f'/home/sebastian/omni.replicator_out/{OUTPUT_DIR}'
output_path = f'/home/sebastian/omni.replicator_out/output_video_demo_final_top_{int(FPS)}fps.mp4'


# Start the process and delete loop
# process_and_delete_images(folder_path)

# Create video from images
images_to_video(folder_path, output_path)