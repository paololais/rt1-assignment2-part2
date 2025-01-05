# RT1 Assignment 2 - Part 2

This ROS2 package implements a robot controller node that allows users to command a robot's linear and angular velocities via a simple user interface. The node publishes velocity commands to `/cmd_vel` and subscribes to `/odom` to retrieve the robot's position. The robot is moved for 1 second based on user inputs, after which it stops automatically.

## Features
- Command linear velocity (`x`) and angular velocity (`z`) via a user interface.
- Publishes velocity commands to the `/cmd_vel` topic.
- Subscribes to the `/odom` topic to get the robot's current position.
- Logs the robot's position and velocity commands.

## Prerequisites
- ROS2 Humble (or compatible version).
- `robot_urdf` package for the simulation environment.

## Setup Instructions

### Step 1: Launch the Simulation
```bash
ros2 launch robot_urdf gazebo.launch.py
```
### Step 2: Build the Package
Clone this repository into your ROS2 workspace (e.g., ~/ros2_ws/src) and build it:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```
### Step 3: Run the Controller Node
After building the package, run the controller node:
```bash
ros2 run assignment2 controller
```
