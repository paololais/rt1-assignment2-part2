# RT1 Assignment 2 - Part 2

This ROS2 package implements a robot controller node that allows users to command a robot's linear and angular velocities via a simple user interface. 

The node publishes velocity commands to `/cmd_vel` and subscribes to `/odom` to retrieve the robot's position. The robot is moved for 1 second based on user inputs, after which it stops automatically.

## Features
- Command linear velocity (`x`) and angular velocity (`z`) via a user interface.
- Publishes velocity commands to the `/cmd_vel` topic.
- Subscribes to the `/odom` topic to get the robot's current position.
- Logs the robot's position and velocity commands.

## Prerequisites
- ROS2 Humble (or compatible version).
- `robot_urdf` package for the simulation environment.

  To get it, run the following commands in your ROS2 workspace:
     ```
     git clone https://github.com/CarmineD8/robot_urdf
     git checkout ros2
     ```

## Code Explanation

### Robot Class
The `Robot` class is responsible for controlling the robot's movement and tracking its position. It contains the following key components:

1. **Initialization (`__init__` method)**:
   - A publisher (`self.publisher_`) is created to send velocity commands (`Twist` messages) to the `/cmd_vel` topic. This controls the robot's movement.
   - A subscriber (`self.subscriber_`) is set up to listen to odometry messages (`Odometry` messages) from the `/odom` topic. This helps track the robot's position.
   - The `velocity` attribute is a `Twist` message used to store the current linear and angular velocities.
   - The `current_position` attribute stores the robot's position, which is updated by the odometry data.

2. **Movement Control (`move` method)**:
   - The `move` method accepts two parameters: `x` (linear velocity) and `z` (angular velocity).
   - It updates the `Twist` message with the provided velocities and publishes it to the `/cmd_vel` topic, commanding the robot to move.
   - The method also logs the current linear and angular velocities for debugging purposes.

3. **Odometry Callback (`odom_callback` method)**:
   - The `odom_callback` method is called whenever an odometry message is received from the `/odom` topic.
   - It extracts the robot's current position (x, y) from the `msg.pose.pose.position` field of the odometry message.
   - The position is logged to the console for monitoring.

### Main Loop (`main` function)
The `main` function orchestrates the execution of the script and manages user interaction:

1. **ROS2 Node Initialization**:
   - The ROS2 system is initialized using `rclpy.init()`, and the `Robot` node is instantiated.
   
2. **User Input**:
   - The script enters a loop where it continuously prompts the user for linear (`x`) and angular (`z`) velocities.
   - The user inputs these values, and the robot moves accordingly.
   - The program checks for invalid input (non-numeric values) and prompts the user again if necessary.

3. **Movement and Stop**:
   - After receiving the velocity inputs, the robot is commanded to move for 1 second using the `move` method.
   - The robot is then stopped by setting both velocities to `0.0` (linear and angular velocities).

4. **Position Logging**:
   - After each movement, the robot's current position is displayed based on the odometry data. If the position is not available yet, the program waits for odometry updates before displaying the position.

5. **Handling exceptions**:
   - The script handles exceptions (e.g., invalid input or errors) and ensures that the robot node is destroyed cleanly when the script is terminated (e.g., by a `KeyboardInterrupt`).



## Setup Instructions
### Step 1: Build the package
Clone this repository into your ROS2 workspace (e.g., ~/ros2_ws/src) and build it:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Step 2: Launch the simulation
```bash
ros2 launch robot_urdf gazebo.launch.py
```
### Step 3: Run the controller node
```bash
ros2 run assignment2 controller
```
### Step 4: Control the robot
Follow the prompts displayed by the controller node to set the robot's velocities and interact with the simulation.

To exit press `Ctrl + c`.
