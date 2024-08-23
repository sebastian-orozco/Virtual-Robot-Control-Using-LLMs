#!/bin/bash
# Author: Sebastian Orozco
# Date: 08-23-2024
# Description: Shell program to boot up Isaac Simulator and TypeFly.

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

# Function to handle cleanup on Ctrl+C
cleanup() {
    echo "Cleaning up..."
    pkill -P $$  # Kill all child processes
    rm -f "$input_pipe_path" "$output_pipe_path"
    rm -rf "$output_dir"/*
    exit 0
}

# Trap Ctrl+C (SIGINT) and call the cleanup function
trap cleanup SIGINT

cd "$PWD/standalone_examples/api/omni.isaac.kaya/"

echo "" > instructions.txt

cd "$HOME/.local/share/ov/pkg/isaac-sim-4.0.0/"
./python.sh "$HOME/.local/share/ov/pkg/isaac-sim-4.0.0/standalone_examples/api/omni.isaac.kaya/temp.py" &

cd "$HOME/TypeFly"
make typefly &

wait

cleanup
