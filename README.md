# Raju Bot - AprilTag Follower

**Developed for the AIR26 Summer Training Program**

This repository contains the full ROS 2 Humble workspace for **Raju Bot**, a custom differential-drive robot designed and simulated in Gazebo Ignition. The primary objective of this project is to implement an autonomous vision-based system where the robot detects, tracks, and precisely follows an AprilTag target using custom kinematics and PID control loops.

---

## Key Features

* **Autonomous AprilTag Tracking:** Utilizes the `pupil_apriltags` C-library to extract 3D pose (Translation and Rotation) from a `tag25h9` family tag.
* **Custom PID Controller:** Features a robust Proportional-Integral-Derivative control loop with Integral Anti-Windup. The robot smoothly steers to center the tag in its camera frame and dynamically accelerates/reverses to maintain a strict `1.0 meter` safe distance.
* **Custom Kinematics & Odometry:** Bypasses default ROS 2 odometry to calculate exact differential drive kinematics directly from raw `/joint_states`. It utilizes **Runge-Kutta 2nd Order (Midpoint) Integration** to accurately map curved driving arcs and broadcasts the `odom -> base_footprint` TF tree.
* **Modern ROS 2 Architecture:** Fully integrated with the latest `ros2_control` stack and Gazebo Harmonic (`ros_gz_bridge`).

---

## Package Architecture

The workspace is modularized into the following packages:

* `bot_description`: Contains the robot's physical URDF/Xacro models, `.stl` chassis meshes, Gazebo world files (configured with shadowless lighting for computer vision), and the main `gazebo.launch.py` script.
* `bot_controller`: Manages the `ros2_control` hardware interfaces and `diff_drive_controller` configurations.
* `bot_detection`: The "brain" of the robot. Contains the Python-based Computer Vision algorithms (`apriltag_detector`), the PID logic node (`apriltag_node`), and the high-precision dead-reckoning node (`odometry_node`).
* `bot_bringup`: Contains high-level launch configurations.

---

### Build Instructions
Clone this repository into your ROS 2 workspace `src/` directory and build:

```bash
cd ~/raju_bot_ws
colcon build --symlink-install
source install/setup.bash
```
*(The `--symlink-install` flag is highly recommended so you can live-edit the Python detection nodes without rebuilding).*

---

## Usage Guide

### 1. Launch the Simulation
This single command spins up Gazebo Harmonic, spawns the robot, loads the `ros2_control` hardware interfaces, and opens RViz2 with the correct TF displays:
```bash
ros2 launch bot_description gazebo.launch.py
```

### 2. Start the Custom Odometry
In a new terminal, launch the custom kinematics node. *Note: We pass `use_sim_time:=true` so the node synchronizes with the Gazebo clock, preventing TF tree timestamp conflicts.*
```bash
cd ~/raju_bot_ws
source install/setup.bash
ros2 run bot_detection odometry_node --ros-args -p use_sim_time:=true
```

### 3. Start the AprilTag Follower
In another terminal, launch the vision and PID controller node. Once started, the robot will lock onto the AprilTag and begin moving automatically.
```bash
cd ~/raju_bot_ws
source install/setup.bash
ros2 run bot_detection apriltag_detector --ros-args -p use_sim_time:=true
```

### 4. View the Camera Feed
To view what the robot's camera sees (including the green tracking bounding boxes and calculated metrics):
```bash
ros2 run rqt_image_view rqt_image_view
```
*Select `/camera/tag_visualization` from the dropdown menu.*

---

## 🛠 Troubleshooting

* **Robot jitters or TF errors in RViz:** Ensure you included `--ros-args -p use_sim_time:=true` when running your Python nodes. Mixing Wall Time and Sim Time will break the TF tree.
* **Controllers fail to load on launch:** You likely have a "zombie" Gazebo process running in the background. Kill it using `killall -9 ruby && killall -9 gz` and relaunch.
* **OpenCV / cv_bridge KeyError:** If the node crashes instantly upon seeing an image, ensure you are running `numpy<2`. The ROS 2 Jazzy `cv_bridge` package currently has a known bug with NumPy 2.0+.
