"""Convert tracked-hand displacement into guarded MoveIt Servo commands."""

from enum import Enum
import math

import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, TwistStamped
from kinova_teleop_interfaces.msg import HandTracking
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class State(Enum):
    WAITING = "WAITING FOR HAND"
    HOLD = "HOLD / THUMB UP"
    TRACKING = "TRACKING"
    LOST = "TRACKING LOST"
    SHUTDOWN = "SHUTDOWN LATCHED"


class TeleopBridge(Node):
    """Dead-man state machine and Cartesian feedback controller."""

    def __init__(self):
        super().__init__("kinova_teleop_bridge")
        defaults = {
            "tracking_topic": "/kinova_teleop/hand_tracking",
            "servo_topic": "/servo_node/delta_twist_cmds",
            "base_frame": "base_link",
            "command_frame": "base_link",
            "ee_frame": "tool_frame",
            "gripper_action": "/robotiq_gripper_controller/gripper_cmd",
            "axis_map": "z,-x,-y",
            "hand_scale": 1.0,
            "linear_gain": 2.0,
            "angular_gain": 1.5,
            "max_linear_speed": 0.12,
            "max_angular_speed": 0.5,
            "tracking_timeout_s": 0.25,
            "hand_jump_limit_m": 0.12,
            "x_limits": "0.20,0.70",
            "y_limits": "-0.40,0.40",
            "z_limits": "0.10,0.75",
            "gripper_open_position": 0.0,
            "gripper_closed_position": 0.8,
            "gripper_max_effort": 40.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.command_frame = str(self.get_parameter("command_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.axis_map = [x.strip() for x in str(self.get_parameter("axis_map").value).split(",")]
        if len(self.axis_map) != 3 or any(
                x.lstrip("-") not in {"x", "y", "z"} for x in self.axis_map):
            raise ValueError("axis_map must contain three entries selected from x,y,z,-x,-y,-z")
        self.limits = np.array([
            self._pair("x_limits"), self._pair("y_limits"), self._pair("z_limits")])

        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("servo_topic").value), 10)
        self.marker_pub = self.create_publisher(Marker, "/kinova_teleop/status_marker", 2)
        self.box_pub = self.create_publisher(Marker, "/kinova_teleop/workspace_marker", 2)
        self.create_subscription(
            HandTracking, str(self.get_parameter("tracking_topic").value), self.on_tracking, 10)
        self.gripper = ActionClient(
            self, GripperCommand, str(self.get_parameter("gripper_action").value))

        self.state = State.WAITING
        self.tracking = None
        self.last_tracking_time = None
        self.previous_hand = None
        self.hand_anchor = None
        self.robot_anchor = None
        self.orientation_anchor = None
        self.previous_gesture = "None"
        self.last_gripper_gesture = "None"
        self.create_timer(0.02, self.control_tick)
        self.create_timer(0.1, self.marker_tick)

    def _pair(self, name):
        values = [float(v) for v in str(self.get_parameter(name).value).split(",")]
        if len(values) != 2 or values[0] >= values[1]:
            raise ValueError(f"{name} must be 'minimum,maximum'")
        return values

    def on_tracking(self, msg):
        self.tracking = msg
        self.last_tracking_time = self.get_clock().now()
        if self.state == State.SHUTDOWN:
            return
        if not msg.valid:
            self.state = State.LOST
            self.previous_hand = None
            self.previous_gesture = "None"
            return

        hand = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self.previous_hand is not None:
            jump = np.linalg.norm(hand - self.previous_hand)
            if jump > float(self.get_parameter("hand_jump_limit_m").value):
                self.state = State.LOST
                self.previous_hand = hand
                self.previous_gesture = "None"
                return
        self.previous_hand = hand

        gesture = msg.gesture
        if gesture == "Thumb_Down":
            self.state = State.SHUTDOWN
            self.hand_anchor = None
            self.robot_anchor = None
            self.get_logger().warning("Thumb down received: teleoperation shutdown latched")
            return
        if gesture == "Thumb_Up":
            self.state = State.HOLD
            self.hand_anchor = hand
            pose = self.current_pose()
            if pose is not None:
                self.robot_anchor, self.orientation_anchor = pose
        elif self.previous_gesture == "Thumb_Up" and gesture != "Thumb_Up":
            pose = self.current_pose()
            if pose is not None:
                self.hand_anchor = hand
                self.robot_anchor, self.orientation_anchor = pose
                self.state = State.TRACKING
        elif self.state == State.LOST:
            # Lost tracking never auto-resumes; require thumb-up re-arming.
            pass

        if gesture in ("Closed_Fist", "Open_Palm") and gesture != self.last_gripper_gesture:
            self.command_gripper(gesture == "Closed_Fist")
            self.last_gripper_gesture = gesture
        elif gesture not in ("Closed_Fist", "Open_Palm"):
            self.last_gripper_gesture = "None"
        self.previous_gesture = gesture

    def current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time(), timeout=Duration(seconds=0.03))
        except TransformException:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])

    def map_hand_delta(self, delta):
        source = {"x": delta[0], "y": delta[1], "z": delta[2]}
        mapped = []
        for entry in self.axis_map:
            sign = -1.0 if entry.startswith("-") else 1.0
            mapped.append(sign * source[entry.lstrip("-")])
        return np.array(mapped) * float(self.get_parameter("hand_scale").value)

    @staticmethod
    def quaternion_error(target, current):
        # q_error = target * conjugate(current), returned as an angle-axis vector.
        tx, ty, tz, tw = target
        cx, cy, cz, cw = -current[0], -current[1], -current[2], current[3]
        q = np.array([
            tw*cx + tx*cw + ty*cz - tz*cy,
            tw*cy - tx*cz + ty*cw + tz*cx,
            tw*cz + tx*cy - ty*cx + tz*cw,
            tw*cw - tx*cx - ty*cy - tz*cz])
        if q[3] < 0.0:
            q = -q
        norm = np.linalg.norm(q[:3])
        if norm < 1e-9:
            return np.zeros(3)
        angle = 2.0 * math.atan2(norm, max(1e-9, q[3]))
        return q[:3] / norm * angle

    @staticmethod
    def clamp_norm(vector, maximum):
        norm = np.linalg.norm(vector)
        return vector if norm <= maximum or norm == 0.0 else vector * maximum / norm

    def publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        self.twist_pub.publish(msg)

    def control_tick(self):
        now = self.get_clock().now()
        stale = self.last_tracking_time is None or (
            now - self.last_tracking_time).nanoseconds / 1e9 > float(
                self.get_parameter("tracking_timeout_s").value)
        if stale and self.state not in (State.SHUTDOWN, State.WAITING):
            self.state = State.LOST
        if self.state != State.TRACKING or self.tracking is None:
            self.publish_stop()
            return
        if self.hand_anchor is None or self.robot_anchor is None or self.orientation_anchor is None:
            self.state = State.HOLD
            self.publish_stop()
            return

        pose = self.current_pose()
        if pose is None:
            self.state = State.LOST
            self.publish_stop()
            return
        position, orientation = pose
        hand = np.array([self.tracking.point.x, self.tracking.point.y, self.tracking.point.z])
        desired = self.robot_anchor + self.map_hand_delta(hand - self.hand_anchor)
        desired = np.clip(desired, self.limits[:, 0], self.limits[:, 1])
        linear = (desired - position) * float(self.get_parameter("linear_gain").value)
        angular = self.quaternion_error(self.orientation_anchor, orientation) * float(
            self.get_parameter("angular_gain").value)
        linear = self.clamp_norm(linear, float(self.get_parameter("max_linear_speed").value))
        angular = self.clamp_norm(angular, float(self.get_parameter("max_angular_speed").value))

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear.tolist()
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular.tolist()
        self.twist_pub.publish(msg)

    def command_gripper(self, closed):
        if not self.gripper.server_is_ready():
            self.get_logger().warning("Gripper action server is unavailable")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(self.get_parameter(
            "gripper_closed_position" if closed else "gripper_open_position").value)
        goal.command.max_effort = float(self.get_parameter("gripper_max_effort").value)
        self.gripper.send_goal_async(goal)

    def marker_tick(self):
        status = Marker()
        status.header.frame_id = self.base_frame
        status.header.stamp = self.get_clock().now().to_msg()
        status.ns = "kinova_teleop"
        status.id = 0
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.z = self.limits[2, 1] + 0.1
        status.scale.z = 0.06
        status.color.a = 1.0
        status.color.r = 1.0 if self.state != State.TRACKING else 0.0
        status.color.g = 1.0 if self.state == State.TRACKING else 0.3
        status.text = self.state.value
        self.marker_pub.publish(status)

        box = Marker()
        box.header = status.header
        box.ns = "kinova_teleop"
        box.id = 1
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = float(np.mean(self.limits[0]))
        box.pose.position.y = float(np.mean(self.limits[1]))
        box.pose.position.z = float(np.mean(self.limits[2]))
        box.pose.orientation.w = 1.0
        box.scale.x, box.scale.y, box.scale.z = (self.limits[:, 1] - self.limits[:, 0]).tolist()
        box.color.a = 0.08
        box.color.b = 1.0
        self.box_pub.publish(box)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
