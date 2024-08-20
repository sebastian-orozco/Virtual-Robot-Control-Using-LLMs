#!/bin/bash

# Paths to the named pipes
input_pipe_path="/tmp/input_pipe"
output_pipe_path="/tmp/output_pipe"

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


cd standalone_examples/api/omni.isaac.kaya/IPC/

echo "" > instructions.txt

# python3 A.py &

# python3 B.py &

cd /home/sebastian/.local/share/ov/pkg/isaac-sim-4.0.0/
./python.sh /home/sebastian/.local/share/ov/pkg/isaac-sim-4.0.0/standalone_examples/api/omni.isaac.kaya/IPC/temp.py &

# cd ~/TypeFly
# make typefly &

wait

rm /tmp/input_pipe /tmp/output_pipe 
