"""Track a hand from a regular RGB webcam for Kinova teleoperation."""

import cv2
import gi
import mediapipe as mp
import numpy as np
import rclpy

gi.require_version("Gst", "1.0")
from gi.repository import Gst

from cv_bridge import CvBridge
from kinova_teleop_interfaces.msg import HandTracking
from rclpy.node import Node
from sensor_msgs.msg import Image

PALM_LANDMARKS = (0, 1, 2, 5, 9, 13, 17)


class HandTracker(Node):
    def __init__(self):
        super().__init__("kinova_hand_tracker")
        self.declare_parameter("output_topic", "/kinova_teleop/hand_tracking")
        self.declare_parameter("camera_frame", "webcam_frame")
        self.declare_parameter("minimum_confidence", 0.65)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("mirror_image", True)
        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("xy_scale_m", 0.50)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(HandTracking, str(self.get_parameter("output_topic").value), 10)
        self.debug_pub = self.create_publisher(Image, "/kinova_teleop/debug_image", 2)

        confidence = float(self.get_parameter("minimum_confidence").value)
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=confidence,
            min_tracking_confidence=confidence,
        )
        self.drawing = mp.solutions.drawing_utils

        Gst.init(None)
        device = str(self.get_parameter("video_device").value)
        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)
        fps = int(self.get_parameter("fps").value)
        pipeline_string = (
            f"v4l2src device={device} ! "
            f"video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=appsink drop=true max-buffers=1 sync=false"
        )
        self.get_logger().info(f"Opening webcam: {device}")
        self.get_logger().info(f"GStreamer pipeline: {pipeline_string}")
        self.pipeline = Gst.parse_launch(pipeline_string)
        self.appsink = self.pipeline.get_by_name("appsink")
        if self.appsink is None:
            raise RuntimeError("Could not create GStreamer appsink")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not start webcam GStreamer pipeline")
        self.get_logger().info("GStreamer webcam pipeline started.")
        self.create_timer(1.0 / fps, self.process)
        self.get_logger().info("Kinova RGB webcam hand tracker started.")

    @staticmethod
    def finger_up(points, tip, pip):
        return points[tip][1] < points[pip][1]

    def infer_gesture(self, points):
        index = self.finger_up(points, 8, 6)
        middle = self.finger_up(points, 12, 10)
        ring = self.finger_up(points, 16, 14)
        pinky = self.finger_up(points, 20, 18)
        fingers = (index, middle, ring, pinky)
        thumb_vertical = points[4][1] - points[2][1]
        if all(fingers):
            return "Open_Palm"
        if not any(fingers):
            if thumb_vertical < -0.10:
                return "Thumb_Up"
            return "Closed_Fist"
        return "None"

    def invalid_message(self):
        msg = HandTracking()
        msg.header.frame_id = str(self.get_parameter("camera_frame").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.valid = False
        msg.gesture = "None"
        msg.point.x = msg.point.y = msg.point.z = 0.0
        msg.confidence = 0.0
        return msg

    def get_frame(self):
        sample = self.appsink.emit("try-pull-sample", 100_000_000)
        if sample is None:
            return None
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        if caps is None:
            return None
        structure = caps.get_structure(0)
        width = int(structure.get_value("width"))
        height = int(structure.get_value("height"))
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return None
        try:
            data = np.frombuffer(map_info.data, dtype=np.uint8)
            expected = width * height * 3
            if data.size < expected:
                return None
            return data[:expected].reshape((height, width, 3)).copy()
        finally:
            buffer.unmap(map_info)

    def publish_debug(self, frame):
        if not bool(self.get_parameter("publish_debug_image").value):
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("camera_frame").value)
        self.debug_pub.publish(msg)

    def process(self):
        frame = self.get_frame()
        if frame is None:
            self.pub.publish(self.invalid_message())
            return
        if bool(self.get_parameter("mirror_image").value):
            frame = cv2.flip(frame, 1)
        result = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if not result.multi_hand_landmarks or not result.multi_handedness:
            self.pub.publish(self.invalid_message())
            self.publish_debug(frame)
            return

        hand_landmarks = result.multi_hand_landmarks[0]
        landmarks = hand_landmarks.landmark
        handedness = result.multi_handedness[0].classification[0]
        points = [(p.x, p.y, p.z) for p in landmarks]
        palm_x = float(np.mean([landmarks[i].x for i in PALM_LANDMARKS]))
        palm_y = float(np.mean([landmarks[i].y for i in PALM_LANDMARKS]))
        scale = float(self.get_parameter("xy_scale_m").value)
        hand_x = (palm_x - 0.5) * scale
        hand_y = -(palm_y - 0.5) * scale
        gesture = self.infer_gesture(points)

        msg = HandTracking()
        msg.header.frame_id = str(self.get_parameter("camera_frame").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.valid = True
        msg.gesture = gesture
        msg.point.x = hand_x
        msg.point.y = hand_y
        msg.point.z = 0.0
        msg.confidence = float(handedness.score)
        self.pub.publish(msg)

        debug = frame.copy()
        self.drawing.draw_landmarks(debug, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
        u, v = int(palm_x * frame.shape[1]), int(palm_y * frame.shape[0])
        cv2.putText(debug, gesture, (u + 15, v), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        self.publish_debug(debug)

    def destroy_node(self):
        if hasattr(self, "pipeline") and self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        if hasattr(self, "hands"):
            self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
