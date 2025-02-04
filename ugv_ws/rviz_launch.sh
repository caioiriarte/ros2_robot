#!/bin/bash

# Kill existent Gazebo processes
killall gzserver
killall gzclient

echo "Building workspace..."
rm -rf build/ install/ log/
colcon build --symlink-install

# Configure workspace
source install/setup.bash
export SVGA_VGPU10=0

# If compilation was successful, launch robot.launch.py
if [ $? -eq 0 ]; then
    echo "Compilation successful. Launching robot.launch.py..."
    ros2 launch robot_1 rviz.launch.py
else
    echo "There was an error. Couldn't launch file."
    exit 1
fi
