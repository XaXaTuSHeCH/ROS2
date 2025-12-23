import math
import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class ArucoMissionNode(Node):
    def __init__(self):
        super().__init__('aruco_mission_node')

        # ===== параметры =====
        self.declare_parameter('detect_start', 154.0)
        self.declare_parameter('detect_end', 164.0)
        self.detect_start = float(self.get_parameter('detect_start').value)
        self.detect_end = float(self.get_parameter('detect_end').value)

        # ===== состояние =====
        self.current_time = None
        self.race_start_time = None
        self.race_started = False
        self.published = False

        # ===== подписки =====
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        self.traffic_light_sub = self.create_subscription(
            Bool,
            '/traffic_light_go',
            self.traffic_light_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/color/image',
            self.image_callback,
            10
        )

        # ===== паблишер =====
        self.publisher = self.create_publisher(
            Float32,
            '/mission_aruco',
            10
        )

        self.bridge = CvBridge()

        # ===== ArUco =====
        aruco = cv2.aruco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

        self.get_logger().info("Aruco mission node started")

    # ---------- clock ----------
    def clock_callback(self, msg: Clock):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ---------- traffic light ----------
    def traffic_light_callback(self, msg: Bool):
        if msg.data and not self.race_started and self.current_time is not None:
            self.race_started = True
            self.race_start_time = self.current_time
            self.published = False
            self.get_logger().info(
                f"Race started at t={self.race_start_time:.3f}"
            )

    # ---------- image ----------
    def image_callback(self, msg: Image):
        # ещё не стартовали
        if not self.race_started:
            return
        if self.current_time is None or self.race_start_time is None:
            return

        elapsed = self.current_time - self.race_start_time

        # не в окне
        if not (self.detect_start <= elapsed <= self.detect_end):
            return

        # уже отправили
        if self.published:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None or len(ids) == 0:
            return

        aruco_id = int(ids.flatten()[0])
        value = round(math.sqrt(float(aruco_id)), 3)

        out = Float32()
        out.data = value
        self.publisher.publish(out)

        self.published = True
        self.get_logger().info(
            f"Aruco ID={aruco_id}, published √ID={value} at elapsed={elapsed:.3f}s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
