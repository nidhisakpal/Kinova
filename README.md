# Kinova Vision Teleoperation

Hand-gesture teleoperation of a **virtual Kinova Gen3 + Robotiq 2F-85 gripper** using a regular RGB laptop webcam, MediaPipe, ROS 2 Jazzy, MoveIt Servo, and MoveIt fake hardware.

This is the tested workflow. A RealSense/depth camera is **not required** for the virtual demo.

## Gestures

| Gesture | Behavior |
|---|---|
| Thumb up | Hold/re-anchor hand and robot pose |
| Release thumb up / neutral hand | Enter `TRACKING` |
| Closed fist | Close gripper |
| Open palm | Open gripper |
| Hand lost | Stop after tracking timeout |

The RGB webcam provides X/Y hand displacement; hand Z is fixed to zero.

## Requirements

Ubuntu 24.04 x86-64, ROS 2 Jazzy, MoveIt/MoveIt Servo, `ros2_kortex` Jazzy, MediaPipe 0.10.21, GStreamer/Python GI, CycloneDDS, and a webcam accessible through V4L2/GStreamer.

## Python environment

```bash
cd ~/kinova_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip uninstall mediapipe -y
python -m pip install "mediapipe==0.10.21" "numpy<2"
```

Verify:

```bash
python -c "import mediapipe as mp; print(mp.__version__); print(hasattr(mp, 'solutions'))"
python -c "import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst; print('GStreamer Python OK')"
```

## Build

```bash
cd ~/kinova_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python -m colcon build --symlink-install
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Source EVERY new terminal

```bash
cd ~/kinova_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Do not mix Fast DDS and CycloneDDS between terminals.

# Tested virtual demo: exact startup order

The order matters. **Move the Gen3 to the bent starting pose before starting Servo.** The default all-zero pose is close to a singularity and Servo can emergency-stop.

## Terminal 1 — virtual Gen3 + RViz

After sourcing:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config \
  robot.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=true
```

Leave it running.

## Terminal 2 — move to a non-singular starting pose

Do this **before Servo is running**:

```bash
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [{
      positions: [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }}"
```

Verify:

```bash
ros2 topic echo /joint_states --once
```

The first seven positions should be approximately:

```text
0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0
```

If they stay zero, check:

```bash
ros2 topic info /joint_trajectory_controller/joint_trajectory -v
```

Before setting the starting pose, its `Publisher count` should be `0`. If Servo is publishing there, stop the Servo/teleop launch first.

## Terminal 3 — RGB webcam tracker

After sourcing:

```bash
ros2 run kinova_hand_perception hand_tracker
```

Expected:

```text
GStreamer webcam pipeline started.
Kinova RGB webcam hand tracker started.
```

The tested webcam pipeline is `/dev/video0`, NV12, 1280x720 at 30 fps. Test it independently with:

```bash
gst-launch-1.0 v4l2src device=/dev/video0 \
  ! video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 \
  ! videoconvert \
  ! autovideosink
```

Check gestures:

```bash
ros2 topic echo /kinova_teleop/hand_tracking --field gesture
```

Expected: `Thumb_Up`, `Closed_Fist`, `Open_Palm`.

Optional debug image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/kinova_teleop/debug_image`.

## Terminal 4 — MoveIt Servo + teleop bridge

After sourcing:

```bash
ros2 launch kinova_teleop teleop.launch.py \
  launch_perception:=false \
  axis_map:="x,-y,z" \
  base_frame:=base_link \
  command_frame:=base_link \
  ee_frame:=bracelet_link
```

Important: the tested Gen3 end-effector TF frame is **`bracelet_link`**, not `tool_frame`. `launch_perception:=false` prevents a duplicate hand tracker.

Verify TF if needed:

```bash
ros2 run tf2_ros tf2_echo base_link bracelet_link
```

## Terminal 5 — enable Twist commands

After sourcing:

```bash
ros2 service call \
  /servo_node/switch_command_type \
  moveit_msgs/srv/ServoCommandType \
  "{command_type: 1}"
```

Expected: `success: true`.

## Operate

Optionally monitor:

```bash
ros2 topic echo /kinova_teleop/status_marker --field text
```

Then:

1. Hold thumbs up for about one second → `HOLD / THUMB UP`.
2. Open/relax the hand while keeping it visible → `TRACKING`.
3. Move the hand slowly; the virtual Gen3 should move in RViz.
4. Closed fist closes the gripper.
5. Open palm opens the gripper.
6. After tracking loss, use thumbs up again to re-anchor.

## Diagnostics

Nonzero hand-to-Servo commands while tracking:

```bash
ros2 topic echo /servo_node/delta_twist_cmds
```

Servo status:

```bash
ros2 topic echo /servo_node/status
```

Trajectory output:

```bash
ros2 topic hz /joint_trajectory_controller/joint_trajectory
```

Controllers:

```bash
ros2 control list_controllers
```

Fake-hardware joint feedback:

```bash
ros2 topic echo /joint_states --field position
```

## Troubleshooting

### `Very close to a singularity, emergency stop`

Stop the Servo/teleop launch. Confirm `/joint_trajectory_controller/joint_trajectory` has `Publisher count: 0`, then resend the bent starting pose from Terminal 2. Confirm `/joint_states` changes before restarting Servo.

### `Thumb-up detected, but robot end-effector pose is unavailable`

Use `bracelet_link`:

```bash
ros2 run tf2_ros tf2_echo base_link bracelet_link
```

### `Command type has not been set, cannot accept input`

Run the Terminal 5 `switch_command_type` service call with `command_type: 1`.

### Status says `TRACKING LOST`

Show thumbs up again, then transition to a neutral/open hand while keeping the hand visible.

### Webcam works with GStreamer but OpenCV reports no GStreamer

This project uses Python GStreamer bindings directly. `opencv-python` may report:

```bash
python -c "import cv2; print('GStreamer: YES' in cv2.getBuildInformation())"
```

as `False`; that is okay for this implementation.

### IPU6 laptops

On the tested Intel IPU6 laptop, `v4l2src device=/dev/video0` worked even when `icamerasrc` failed with CamHAL/PSYS errors.

### Gripper test

Close:

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.8, max_effort: 40.0}}"
```

Open:

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 40.0}}"
```

## Safety

This procedure is for **MoveIt fake hardware / virtual demonstration**. Before adapting it to a physical robot, independently review workspace limits, speeds, collision behavior, emergency-stop behavior, and RGB-only tracking limitations.
