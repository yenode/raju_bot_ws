# Raju Bot Simulation Commands Guide (Updated for ros2_control)

This guide will help you launch, test, and control the robot in the Gazebo simulation environment using the ROS 2 Humble `ros2_control` stack.

## 1. Dependencies and Installation
Before running the simulation, you must install the required ROS 2 packages. The easiest and recommended way is to install everything via `apt`:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control \
  ros-humble-cv-bridge \
  python3-pip

# Install Python dependencies required for AprilTag detection
# Note: numpy must be < 2.0.0 to prevent ABI crashes with cv_bridge
pip install 'numpy<2.0.0' opencv-python pupil-apriltags
```

## 2. Setup and Build
First, ensure that your workspace is built and sourced correctly. Whenever you make changes to your URDF or Python scripts, remember to rebuild.

```bash
# Navigate to your workspace
cd ~/raju_bot_ws

# Build the required packages
colcon build --packages-select bot_description bot_controller bot_detection

# Source the workspace setup file
source install/setup.bash
```

## 3. Launching the Simulation
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

## 4. Controlling the Robot

Because the simulation now uses `ros2_control` and `diff_drive_controller` in ROS 2 Humble with `use_stamped_vel: false` configured, the controller accepts standard `Twist` messages on the `/cmd_vel` topic. 

### Option A: Using the Keyboard Teleop
You can use the standard `teleop_twist_keyboard` package right out of the box because we've remapped the controller topics inside the URDF.

```bash
# Open a new terminal and source the workspace
cd ~/raju_bot_ws
source install/setup.bash

# Run the teleop node
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the `I`, `J`, `K`, `L`, `,` keys to move the robot.

### Option B: Using the Python `bot_controller`
You can also use the custom Python script we developed to automatically drive the robot forward in a circle.

```bash
# In a new terminal
cd ~/raju_bot_ws
source install/setup.bash
ros2 run bot_controller robot_controller --ros-args -p use_sim_time:=true
```

### Option C: Publishing Directly from CLI
If you want to manually publish a single twist message to make the robot move via the terminal:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

## 5. Visualizing in RViz
RViz is already launched automatically by `gazebo.launch.py`.
If you want to view the camera stream:
1. Click **Add** -> **By topic** -> Select the **Camera / Image** topic.
2. The `camera_link` is perfectly aligned `8.7 cm` forward in the `base_footprint` frame.

## 6. AprilTag Detection and Following
The robot is equipped with a camera and an AprilTag detector node that implements a full PID controller to smoothly track a `tag25h9` family tag and maintain a safe distance of 1.0 meter.

```bash
# In a new terminal, source the workspace
cd ~/raju_bot_ws
source install/setup.bash

# Run the Custom Odometry Node
ros2 run bot_detection odometry_node --ros-args -p use_sim_time:=true

# In a separate terminal, run the AprilTag Follower node
ros2 run bot_detection apriltag_detector --ros-args -p use_sim_time:=true
```

To view the live visualization feed with detection overlays:
```bash
ros2 run rqt_image_view rqt_image_view
# Select /camera/tag_visualization in the dropdown
```

## 7. Troubleshooting
- **Robot is floating or sinking in Gazebo**: The origin of the STL mesh (`raju.stl`) and wheels determines their placement relative to the `base_link`. The `base_footprint_joint` `Z` origin is strictly set to `0.072m` to ensure the wheels (`0.035m` radius + `0.037m` Z-offset) sit exactly on the `Z=0` ground plane.
- **Controllers fail to load**: If `spawner_diff_cont` or `spawner_joint_state_broadcaster` instantly fail or time out, it is highly likely a zombie Gazebo process is holding the controller manager. Run `killall -9 ruby && killall -9 gz` to clean it up before relaunching.
