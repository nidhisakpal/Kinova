# Kinova Vision Teleoperation

Control a Kinova Gen3 or Gen3 Lite robotic arm using hand movements and gestures
captured by an RGB-D camera.

This ROS 2 project recreates the behavior of FrankaTeleop using
[`ros2_kortex`](https://github.com/Kinovarobotics/ros2_kortex), MoveIt Servo,
MediaPipe, and an Intel RealSense-compatible camera. It includes explicit
lost-tracking, workspace, and shutdown protections.

## Behaviors

| Gesture | Behavior |
|---|---|
| Thumb up | Hold the robot and recalibrate the hand/robot anchor |
| Release thumb up | Start or resume relative hand tracking |
| Thumb down | Latch teleoperation off until the teleop node is restarted |
| Closed fist | Close the gripper |
| Open palm | Open the gripper |

Hand translation commands end-effector translation. The end-effector
orientation is held at the orientation captured when tracking is armed.
Commands stop when tracking becomes stale or invalid, TF is unavailable, the
hand jumps unexpectedly, or thumb-down is detected. MoveIt Servo adds
collision, singularity, and joint-limit monitoring.

## Packages

- `kinova_teleop_interfaces`: tracked-hand ROS message.
- `kinova_hand_perception`: RGB-D/MediaPipe hand tracking and gesture node.
- `kinova_teleop`: safety state machine, Cartesian controller, gripper actions,
  RViz markers, MoveIt Servo configuration, and launch files.

# Complete simulation guide

The recommended first test uses **MoveIt fake hardware and RViz**. This runs the
real perception and teleoperation pipeline while replacing the physical Kinova
arm with a virtual arm. A physical RealSense camera is still used for hand
tracking; alternatively, you can replay a previously recorded ROS bag.

## Platform requirements

Use a native **Ubuntu 24.04 x86-64** installation with **ROS 2 Jazzy**.

Native Windows is not supported by the complete stack because the Kinova Kortex
driver uses a precompiled Linux x86-64 library. Windows remains suitable for
editing and managing the source code. WSL2 may be useful for experimentation,
but camera USB forwarding, RViz graphics, networking, and timing make it a less
reliable choice for this project.

Recommended computer:

- Ubuntu 24.04, x86-64
- 16 GB RAM or more
- 4 CPU cores or more
- An Intel RealSense D435/D435i or compatible aligned RGB-D source
- A GPU is optional for fake-hardware/RViz testing

## 1. Install ROS 2 Jazzy

Follow the official
[ROS 2 Jazzy Ubuntu installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

After installation, verify ROS:

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
```

The result should be:

```text
jazzy
```

## 2. Install build, MoveIt, camera, and Python dependencies

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep \
  python3-venv \
  python3-yaml \
  python3-opencv \
  python3-numpy \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-realsense2-camera \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rqt-image-view \
  ros-jazzy-tf-transformations
```

Initialize `rosdep` once:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that it has already been initialized, continue with
`rosdep update`.

## 3. Create the ROS workspace

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/kinova_ws/src
cd ~/kinova_ws/src
```

Clone the Kinova ROS 2 driver and this project:

```bash
git clone --branch jazzy https://github.com/Kinovarobotics/ros2_kortex.git
git clone https://github.com/nidhisakpal/Kinova.git
```

Import the additional Kortex source dependencies:

```bash
cd ~/kinova_ws
vcs import src --skip-existing \
  --input src/ros2_kortex/ros2_kortex.jazzy.repos
vcs import src --skip-existing \
  --input src/ros2_kortex/ros2_kortex-not-released.jazzy.repos
```

## 4. Install MediaPipe in a Python virtual environment

The virtual environment uses the system ROS Python packages while isolating the
MediaPipe installation:

```bash
cd ~/kinova_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install mediapipe
```

Verify the installation:

```bash
python -c "import mediapipe; print(mediapipe.__version__)"
```

## 5. Install remaining dependencies and build

Keep the virtual environment active while building:

```bash
cd ~/kinova_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --symlink-install \
  --parallel-workers 3 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Source the finished workspace:

```bash
source ~/kinova_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Verify that the project packages are available:

```bash
ros2 pkg list | grep -E "kinova_(teleop|hand)"
```

Expected packages:

```text
kinova_teleop
kinova_teleop_interfaces
kinova_hand_perception
```

## 6. Source every terminal

Run this block in every new terminal used below:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kinova_ws/.venv/bin/activate
source ~/kinova_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 7. Start the RealSense RGB-D camera

In **Terminal 1**:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true \
  align_depth.enable:=true
```

Depth alignment is required because the hand tracker samples depth at a pixel
detected in the color image.

Discover the actual camera topics:

```bash
ros2 topic list | grep -E "color/image_raw|aligned_depth|camera_info"
```

Recent RealSense ROS versions commonly publish:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

Leave this terminal running.

## 8. Start the virtual Kinova Gen3 arm

In **Terminal 2**, after sourcing the environment:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config \
  robot.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=true
```

The IP address is only a required placeholder in fake-hardware mode; no physical
robot connection is made. RViz should open with a virtual 7-DOF Gen3 and
Robotiq 2F-85 gripper.

Leave this terminal running.

## 9. Start hand teleoperation

In **Terminal 3**, after sourcing the environment:

```bash
ros2 launch kinova_teleop teleop.launch.py \
  moveit_config_package:=kinova_gen3_7dof_robotiq_2f_85_moveit_config \
  robot_name:=gen3 \
  move_group_name:=manipulator \
  base_frame:=base_link \
  command_frame:=base_link \
  ee_frame:=tool_frame \
  gripper_action:=/robotiq_gripper_controller/gripper_cmd \
  color_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

If your topics differ from those shown above, substitute the names returned by
`ros2 topic list`.

Some MoveIt Servo releases expose a start service. Check for it:

```bash
ros2 service list | grep servo
```

If `/servo_node/start_servo` is listed, call it once:

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```

## 10. Verify the simulation before moving

Confirm that hand tracking produces data:

```bash
ros2 topic echo /kinova_teleop/hand_tracking
```

The message should eventually contain:

```text
valid: true
gesture: <detected gesture>
```

Confirm that the trajectory controller is active:

```bash
ros2 control list_controllers
```

Look for:

```text
joint_trajectory_controller ... active
```

Confirm that MoveIt Servo is running:

```bash
ros2 node list | grep servo
ros2 topic echo /servo_node/status
```

View the processed camera stream:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/kinova_teleop/debug_image` in the image-view dropdown. The right hand
should be marked and the recognized gesture should appear beside it.

## 11. Operate the virtual arm

1. Place your right hand clearly in the camera view.
2. Hold a thumbs-up. The virtual arm should remain stationary while the system
   records a new hand and robot anchor.
3. Release the thumbs-up. Small hand movements should now move the virtual arm.
4. Make a closed fist to close the gripper.
5. Show an open palm to open the gripper.
6. Move your hand out of view. The arm should stop.
7. Show another thumbs-up to re-arm after tracking is lost.
8. Show thumbs-down to latch teleoperation off. Restart the teleop launch to use
   it again.

Begin with slow, small hand movements. The target pose is restricted to a
configurable Cartesian workspace box.

## 12. Test the gripper independently

Close the simulated gripper:

```bash
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.8, max_effort: 40.0}}"
```

Open it:

```bash
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 40.0}}"
```

## Testing without a camera

The repository does not currently contain a recorded camera dataset. You can
record one while a compatible camera is connected:

```bash
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/color/camera_info
```

Replay the recording later instead of launching the camera:

```bash
ros2 bag play <bag-directory> --loop
```

Then start the virtual robot and teleoperation as described above.

## Gen3 Lite simulation

For a Gen3 Lite, replace the robot-specific arguments with:

```text
MoveIt package: kinova_gen3_lite_moveit_config
Robot name: gen3_lite
Gripper action: /gen3_lite_2f_gripper_controller/gripper_cmd
```

Bring up fake hardware with:

```bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=true
```

Then launch teleoperation with the corresponding package, robot name, and
gripper action.

## Gazebo Harmonic

`ros2_kortex` supports Gazebo Harmonic, but this project's recommended and
documented test path is MoveIt fake hardware. Gazebo adds simulated-time,
controller, mimic-joint, and Protobuf considerations, while a physical or
recorded RealSense stream usually uses wall-clock time. Additional clock and
launch integration should be completed before treating the current teleop
launch as a supported Gazebo workflow.

Use fake hardware first to validate perception, gesture handling, MoveIt Servo,
controller output, and gripper behavior.

## Troubleshooting

### `Package 'kinova_teleop' not found`

Source the workspace in the current terminal:

```bash
source ~/kinova_ws/install/setup.bash
```

If the package is still missing, rebuild it:

```bash
cd ~/kinova_ws
colcon build --symlink-install --packages-up-to kinova_teleop
source install/setup.bash
```

### `ModuleNotFoundError: No module named 'mediapipe'`

Activate the same virtual environment used during the build:

```bash
source ~/kinova_ws/.venv/bin/activate
python -c "import mediapipe"
```

### No camera messages

Check the camera connection and topic names:

```bash
ros2 node list | grep camera
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw
```

Pass the discovered topic names to `teleop.launch.py`.

### Hand is visible but `valid` remains false

- Use the right hand.
- Keep the full palm inside the image.
- Improve lighting.
- Avoid reflective or missing depth regions.
- Remain within the configured maximum camera distance.
- Confirm that aligned depth is enabled.

### Virtual arm does not move

Check all three stages:

```bash
ros2 topic echo /kinova_teleop/hand_tracking
ros2 topic hz /servo_node/delta_twist_cmds
ros2 control list_controllers
```

Then verify that:

- Hand tracking reports `valid: true`.
- You armed the system with a thumbs-up and then released it.
- MoveIt Servo is running or its start service has been called.
- `joint_trajectory_controller` is active.
- The robot is not in a collision, joint-limit, or singularity stop state.

### Movement direction feels reversed

The `axis_map` launch argument controls camera-to-robot motion mapping. The
default is:

```text
z,-x,-y
```

Camera depth drives robot X, camera horizontal movement drives negative robot
Y, and camera vertical movement drives negative robot Z. Each entry can be
`x`, `y`, `z`, `-x`, `-y`, or `-z`.

Example override:

```bash
ros2 launch kinova_teleop teleop.launch.py axis_map:=z,x,-y <other-arguments>
```

### Build consumes too much memory

Reduce parallelism:

```bash
colcon build \
  --symlink-install \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Moving from simulation to the real arm

Only move to real hardware after fake-hardware testing passes. Keep the robot's
emergency stop accessible, use an empty workspace, verify the Cartesian limits,
and begin with reduced speeds.

The real 7-DOF Gen3 launch is:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config \
  robot.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=false
```

Before arming teleoperation, reduce the initial velocity limits:

```bash
ros2 param set /kinova_teleop_bridge max_linear_speed 0.03
ros2 param set /kinova_teleop_bridge max_angular_speed 0.15
```

# Safety

This is research software, not a certified safety system. The Cartesian box is
only a software target clamp. MoveIt collision checking protects only against
correctly modeled geometry. Always use the robot's native safety functions,
speed limits, and emergency stop.

## Useful references

- [Kinova ROS 2 Kortex](https://github.com/Kinovarobotics/ros2_kortex)
- [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [RealSense ROS](https://github.com/realsenseai/realsense-ros)
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
