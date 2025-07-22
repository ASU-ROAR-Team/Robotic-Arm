#!/bin/bash

# This script sets the GAZEBO_MODEL_PATH and then launches the Gazebo simulation
# for rover and rock models.

# --- Configuration ---
ROS_DISTRO="noetic"

# Path to the directory that DIRECTLY CONTAINS your rock1, rock2, rock3 folders.
CUSTOM_MODELS_DIR="/home/mohamed/Robotic-Arm/src/my_gazebo_models/sampling_rocks"

# Your ROS package name that contains the launch file
YOUR_ROS_PACKAGE_NAME="sampling_rover_with_arm_urdf"

# Your launch file name
YOUR_LAUNCH_FILE_NAME="rover_arm_urdf_rocks.launch" # Confirmed this is the file you want to launch

# --- Source ROS and Workspace Setup ---
# Source the main ROS setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Source your workspace setup (adjust if your workspace path or type is different)
# For catkin workspaces:
source /home/mohamed/Robotic-Arm/devel/setup.bash


# --- Set GAZEBO_MODEL_PATH ---
# Initialize GAZEBO_MODEL_PATH from scratch to avoid redundancy and ensure correct order.
# This sets it to your custom models directory, followed by the default Gazebo ROS models.
export GAZEBO_MODEL_PATH="$CUSTOM_MODELS_DIR"


# --- Launch Gazebo ---
echo "Launching Gazebo with rover and rock models..."
echo "GAZEBO_MODEL_PATH is set to: $GAZEBO_MODEL_PATH"

# Launch the specified ROS package and launch file
roslaunch $YOUR_ROS_PACKAGE_NAME $YOUR_LAUNCH_FILE_NAME 

echo "Script finished."