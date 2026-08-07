"""Publish metric right-hand position and MediaPipe gestures from RGB-D images."""

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from kinova_teleop_interfaces.msg import HandTracking
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


PALM_LANDMARKS = (0, 1, 2, 5, 9, 13, 17)


class HandTracker(Node):
    """Detect a right hand and deproject its palm center into camera coordinates."""

    def __init__(self):
        super().__init__("kinova_hand_tracker")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_topic", "/kinova_teleop/hand_tracking")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("minimum_confidence", 0.65)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("maximum_depth_m", 2.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("mirror_image", True)

        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.color_stamp = None

        # Infer the four required gestures geometrically from the tracked hand,
        # avoiding a separate gesture-model asset at deployment time.
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=1,
            min_detection_confidence=float(self.get_parameter("minimum_confidence").value),
            min_tracking_confidence=float(self.get_parameter("minimum_confidence").value),
        )

        self.pub = self.create_publisher(
            HandTracking, str(self.get_parameter("output_topic").value), 10)
        self.debug_pub = self.create_publisher(Image, "/kinova_teleop/debug_image", 2)
        self.create_subscription(Image, str(self.get_parameter("color_topic").value), self.on_color, 2)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self.on_depth, 2)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self.on_info, 2)
        self.create_timer(1.0 / 30.0, self.process)

    def on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if bool(self.get_parameter("mirror_image").value):
            self.color = cv2.flip(self.color, 1)
        self.color_stamp = msg.header.stamp

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        if bool(self.get_parameter("mirror_image").value):
            self.depth = cv2.flip(self.depth, 1)

    def on_info(self, msg):
        self.info = msg

    @staticmethod
    def _finger_up(points, tip, pip):
        return points[tip][1] < points[pip][1]

    def infer_gesture(self, points):
        index = self._finger_up(points, 8, 6)
        middle = self._finger_up(points, 12, 10)
        ring = self._finger_up(points, 16, 14)
        pinky = self._finger_up(points, 20, 18)
        thumb_vertical = points[4][1] - points[2][1]
        fingers = (index, middle, ring, pinky)
        if not any(fingers) and thumb_vertical < -0.06:
            return "Thumb_Up"
        if not any(fingers) and thumb_vertical > 0.06:
            return "Thumb_Down"
        if all(fingers):
            return "Open_Palm"
        if not any(fingers):
            return "Closed_Fist"
        return "None"

    def invalid_message(self):
        msg = HandTracking()
        msg.header.frame_id = str(self.get_parameter("camera_frame").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.valid = False
        msg.gesture = "None"
        return msg

    def process(self):
        if self.color is None or self.depth is None or self.info is None:
            self.pub.publish(self.invalid_message())
            return

        rgb = cv2.cvtColor(self.color, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        if not result.multi_hand_landmarks or not result.multi_handedness:
            self.pub.publish(self.invalid_message())
            return

        handedness = result.multi_handedness[0].classification[0]
        if handedness.label != "Right":
            self.pub.publish(self.invalid_message())
            return

        landmarks = result.multi_hand_landmarks[0].landmark
        normalized = [(p.x, p.y, p.z) for p in landmarks]
        u = int(np.mean([landmarks[i].x for i in PALM_LANDMARKS]) * self.color.shape[1])
        v = int(np.mean([landmarks[i].y for i in PALM_LANDMARKS]) * self.color.shape[0])
        if not (0 <= u < self.depth.shape[1] and 0 <= v < self.depth.shape[0]):
            self.pub.publish(self.invalid_message())
            return

        radius = 3
        patch = np.asarray(self.depth[max(0, v-radius):v+radius+1, max(0, u-radius):u+radius+1])
        valid_depth = patch[np.isfinite(patch) & (patch > 0)]
        if valid_depth.size == 0:
            self.pub.publish(self.invalid_message())
            return
        z = float(np.median(valid_depth)) * float(self.get_parameter("depth_scale").value)
        if z <= 0.0 or z > float(self.get_parameter("maximum_depth_m").value):
            self.pub.publish(self.invalid_message())
            return

        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        msg = HandTracking()
        msg.header.frame_id = str(self.get_parameter("camera_frame").value)
        msg.header.stamp = self.color_stamp
        msg.valid = True
        msg.gesture = self.infer_gesture(normalized)
        msg.point.x = (u - cx) * z / fx
        msg.point.y = (v - cy) * z / fy
        msg.point.z = z
        msg.confidence = float(handedness.score)
        self.pub.publish(msg)

        if bool(self.get_parameter("publish_debug_image").value):
            debug = self.color.copy()
            cv2.circle(debug, (u, v), 8, (0, 255, 0), -1)
            cv2.putText(debug, msg.gesture, (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
