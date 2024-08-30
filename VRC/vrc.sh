#!/bin/bash
# Author: Sebastian Orozco
# Date: 08-23-2024
# Description: Shell program to boot up Isaac Simulator and TypeFly.
# Provides options to quit entirely, restart TypeFly, or send a custom signal to TypeFly.

# Paths to the named pipes
input_pipe_path="/tmp/input_pipe"
output_pipe_path="/tmp/output_pipe"

# Path to the synthetic data output
output_dir="$HOME/omni.replicator_out/_typefly_1fps"

# Remove existing pipes if they exist
if [ -p "$input_pipe_path" ]; then
    rm "$input_pipe_path"
fi

if [ -p "$output_pipe_path" ]; then
    rm "$output_pipe_path"
fi

# Create pipes
mkfifo "$input_pipe_path"
mkfifo "$output_pipe_path"

# Path to parameters file
parameters_path="$HOME/.local/share/ov/pkg/isaac-sim-4.0.0/VRC/parameters.py"

# Extract whether user wants to make video or not
MAKE_VIDEO=$(python3 -c "
import sys
sys.path.append('$HOME/.local/share/ov/pkg/isaac-sim-4.0.0/VRC')
from parameters import MAKE_VIDEO
print(MAKE_VIDEO)
")

# Function to handle cleanup
cleanup() {
    echo "Cleaning up..."
    kill $isaac_sim_pid
    pkill -P $$  # Kill all child processes
    rm -f "$input_pipe_path" "$output_pipe_path"

    # Check the value of MAKE_VIDEO and take action
    if [ "$MAKE_VIDEO" = "True" ]; then
        echo "MAKE_VIDEO is True, generating video..."
        cd "$HOME/.local/share/ov/pkg/isaac-sim-4.0.0/VRC/"
        python3 video_generation.py 
    fi

    rm -rf "$output_dir"/*
    exit 0
}

# Function to restart TypeFly
restart_typefly() {
    echo "Restarting TypeFly..."
    pkill -f "make typefly" || true
    cd "$HOME/TypeFly"
    make typefly &
    typefly_pid=$!
}

# Trap Ctrl+C (SIGINT) and call the cleanup function if ^C is sent
trap cleanup SIGINT

# Clear out instructions
echo "" > instructions.txt

# Startup Isaac Sim
cd ..
./python.sh "$HOME/.local/share/ov/pkg/isaac-sim-4.0.0/VRC/vrc.py" &
isaac_sim_pid=$!

# Startup TypeFly
restart_typefly

# Main loop to continuously listen for user input
while true; do
    read -p "TypeFly is running. Choose an action: (r)estart TypeFly, or ^C to quit entirely." choice
    case $choice in
        [Rr]* )
            restart_typefly
            ;;
        * )
            echo "Please choose a valid option: r (restart), ^C (quit)."
            ;;
    esac
done
