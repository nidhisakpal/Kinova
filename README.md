# Kinova Vision Teleoperation

ROS 2 Jazzy hand-gesture teleoperation for Kinova Gen3 and Gen3 Lite arms. This
project recreates the behavior of FrankaTeleop using `ros2_kortex`, MoveIt
Servo, MediaPipe, and an RGB-D camera, while adding explicit lost-tracking and
shutdown safety behavior.

## Behaviors

| Gesture | Behavior |
|---|---|
| Thumb up | Hold the robot and recalibrate the hand/robot anchor |
| Release thumb up | Resume relative hand tracking |
| Thumb down | Latch teleoperation off until the node is restarted |
| Closed fist | Close the gripper |
| Open palm | Open the gripper |

Hand translation commands end-effector translation. Orientation is held at the
orientation captured when tracking is armed. Commands stop on stale or lost
tracking, outside the hand workspace, after thumb-down, or when TF is missing.
MoveIt Servo provides collision, singularity, and joint-limit checking.

## Packages

- `kinova_teleop_interfaces`: tracked-hand ROS message.
- `kinova_hand_perception`: RGB-D/MediaPipe hand tracking and gesture node.
- `kinova_teleop`: safety state machine, Cartesian controller, gripper actions,
  RViz markers, MoveIt Servo configuration, and launch files.

## Requirements

- Ubuntu 24.04 and ROS 2 Jazzy
- `ros2_kortex` built for the desired arm
- MoveIt 2 and MoveIt Servo
- An aligned RGB-D stream (defaults match Intel RealSense)
- Python packages: `mediapipe`, `opencv-python`, and `numpy`

```bash
sudo apt install ros-jazzy-moveit-servo ros-jazzy-cv-bridge \
  ros-jazzy-image-transport ros-jazzy-tf-transformations
python3 -m pip install mediapipe
```

Build this repository inside the same workspace as `ros2_kortex`:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

First bring up the robot and its MoveIt configuration. Example for a 7-DOF
Gen3 with a Robotiq 2F-85:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```

For initial testing, use the same launch with `use_fake_hardware:=true`.

Then start perception and teleoperation:

```bash
ros2 launch kinova_teleop teleop.launch.py \
  moveit_config_package:=kinova_gen3_7dof_robotiq_2f_85_moveit_config \
  base_frame:=base_link command_frame:=base_link ee_frame:=tool_frame \
  gripper_action:=/robotiq_gripper_controller/gripper_cmd
```

The default camera topics are:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

Override them as launch arguments when necessary. Keep the robot emergency stop
accessible and begin with reduced speed, fake hardware, and an empty workspace.

## Coordinate mapping

Camera-space relative movement maps to robot axes using the `axis_map` parameter.
The default is `z,-x,-y`: camera depth drives robot X, camera horizontal drives
negative robot Y, and camera vertical drives negative robot Z. Each entry may be
`x`, `y`, `z`, `-x`, `-y`, or `-z`.

## Safety notes

This is research software, not a certified safety system. The Cartesian box is
only a software target clamp. MoveIt collision checking only protects against
correctly modeled geometry. Always use the robot's native safety functions,
speed limits, and emergency stop.

