import time
import os
from PIL import Image 

# Constants
FPS = 20  # Frames per second
refresh_interval = 1  # Number of minutes until refresh

# DEFINE FOLDER PATH TO PROCESS IMAGES
folder_path = '/home/sebastian/omni.replicator_out/_output_test'

def process_images(folder_path):
    processed_count = 0  # Initialize processed images counter
    processed_files = []  # List to keep track of processed files

    while True:
        # Step 1: Read all image files from folder
        files = os.listdir(folder_path)
        image_files = [f for f in files if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]

        # Step 2: Process each image file
        for image_file in image_files:
            image_path = os.path.join(folder_path, image_file)
            print(f"Processing image: {image_path}")
            try:
                # Open image
                with Image.open(image_path) as img:
                    pass  # Replace with image processing code

                # Track processed files
                processed_files.append(image_path)
                processed_count += 1

                # Check if we have processed 'batch_size' images
                if processed_count >= (FPS * refresh_interval * 60):
                    delete_processed_images(processed_files)
                    processed_count = 0
                    processed_files = []

            except Exception as e:
                print(f"Failed to process image {image_path}: {e}")

        # Step 3: Short wait before next iteration
        time.sleep(1)
        print(".")

def delete_processed_images(files_to_delete):
    for file_path in files_to_delete:
        try:
            os.remove(file_path)
            print(f"Deleted image: {file_path}")
        except Exception as e:
            print(f"Failed to delete image {file_path}: {e}")

while True:
    process_images(folder_path)
