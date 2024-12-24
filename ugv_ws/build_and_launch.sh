#!/bin/bash

echo "Building workspace..."
colcon build --symlink-install

# Configure workspace
source install/setup.bash

# If compilation was successful, launch robot.launch.py
if [ $? -eq 0 ]; then
    echo "Compilation successful. Launching robot.launch.py..."
    ros2 launch robot_1 robot.launch.py
else
    echo "There was an error. Couldn't launch file."
    exit 1
fi
