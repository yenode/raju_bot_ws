# Raju Bot Simulation Commands Guide (Updated for ros2_control)

This guide will help you launch, test, and control the robot in the Gazebo simulation environment using the modern ROS 2 Jazzy `ros2_control` stack.

## 1. Setup and Build
First, ensure that your workspace is built and sourced correctly. Whenever you make changes to your URDF or Python scripts, remember to rebuild.

```bash
# Navigate to your workspace
cd ~/air26_ros2_ws

# Build the required packages
colcon build --packages-select bot_description bot_controller

# Source the workspace setup file
source install/setup.bash
```

## 2. Launching the Simulation
You can launch the robot directly into the sample world. This single launch file will:
- Open Gazebo Harmonic.
- Spawn the Raju robot model.
- Automatically launch RViz 2 with pre-configured displays.
- Start the `controller_manager` and spawn the `joint_state_broadcaster` and `diff_cont` controllers.

```bash
ros2 launch bot_description gazebo.launch.py
```
> [!NOTE]
> Ensure that you do not have any zombie `gz sim` processes running before launching this, as it can cause `controller_manager` to hang.

## 3. Controlling the Robot

Because the simulation now uses `ros2_control` and `diff_drive_controller` in ROS 2 Jazzy, the controller exclusively accepts timestamped `TwistStamped` messages rather than standard `Twist` messages. 

### Option A: Using the Keyboard Teleop
You can use the standard `teleop_twist_keyboard` package by passing a special parameter to force it to publish `TwistStamped` messages, and remapping the topic to target our differential controller.

```bash
# Open a new terminal and source the workspace
cd ~/air26_ros2_ws
source install/setup.bash

# Run the teleop node with stamped messages enabled
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel -p stamped:=true
```
Use the `I`, `J`, `K`, `L`, `,` keys to move the robot.

### Option B: Using the Python `bot_controller`
You can also use the custom Python script we developed to automatically drive the robot forward in a circle using `TwistStamped`.

```bash
# In a new terminal
cd ~/air26_ros2_ws
source install/setup.bash
ros2 run bot_controller robot_controller
```

### Option C: Publishing Directly from CLI
If you want to manually publish a single twist message to make the robot move via the terminal:

```bash
ros2 topic pub /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_footprint'}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
```

## 4. Visualizing in RViz
RViz is already launched automatically by `gazebo.launch.py`.
If you want to view the camera stream:
1. Click **Add** -> **By topic** -> Select the **Camera / Image** topic.
2. The `camera_link` is perfectly aligned `8.7 cm` forward in the `base_footprint` frame.

## 5. Troubleshooting
- **Robot is floating or sinking in Gazebo**: The origin of the STL mesh (`raju.stl`) and wheels determines their placement relative to the `base_link`. The `base_footprint_joint` `Z` origin is strictly set to `0.072m` to ensure the wheels (`0.035m` radius + `0.037m` Z-offset) sit exactly on the `Z=0` ground plane.
- **Controllers fail to load**: If `spawner_diff_cont` or `spawner_joint_state_broadcaster` instantly fail or time out, it is highly likely a zombie Gazebo process is holding the controller manager. Run `killall -9 ruby && killall -9 gz` to clean it up before relaunching.
