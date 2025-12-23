# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Bool
# from cv_bridge import CvBridge
# import cv2
# import numpy as np


# class TrafficLightWatcher(Node):
#     """
#     Простая BGR-детекция: ищет пиксели близкие к целевому цвету (B,G,R)
#     в небольшом допуске. Если найдено достаточно пикселей — публикует True
#     на `/traffic_light_go`. Также публикует маску на `/traffic_light/mask`
#     для визуальной отладки.
#     """

#     def __init__(self):
#         super().__init__('traffic_light_watcher')

#         # Целевой цвет и допуски (B, G, R) — можно менять через ros2 param set
#         self.declare_parameter('target_bgr', [0, 189, 0])
#         self.declare_parameter('bgr_tolerance', [40, 60, 40])  # tol for B, G, R

#         # Порог площади и дебаунс (в кадрах)
#         self.declare_parameter('min_area', 10)
#         self.declare_parameter('debounce_frames', 3)

#         tb = self.get_parameter('target_bgr').value
#         tol = self.get_parameter('bgr_tolerance').value
#         self.target_bgr = np.array(tb, dtype=np.int32)
#         self.bgr_tolerance = np.array(tol, dtype=np.int32)

#         self.min_area = int(self.get_parameter('min_area').value)
#         self.debounce_frames = int(self.get_parameter('debounce_frames').value)

#         self.bridge = CvBridge()
#         self.subscription = self.create_subscription(Image, '/color/image', self.image_cb, 10)
#         self.go_publisher = self.create_publisher(Bool, '/traffic_light_go', 10)
#         self.mask_publisher = self.create_publisher(Image, '/traffic_light/mask', 10)

#         self._green_count = 0
#         self._allowed = False

#         # начальное состояние — стоп
#         self.go_publisher.publish(Bool(data=False))
#         self.get_logger().info(f'TrafficLightWatcher started: target_bgr={self.target_bgr.tolist()}, tol={self.bgr_tolerance.tolist()}')

#     def image_cb(self, msg: Image):
#         if self._allowed:
#             return  # уже разрешено движение; latched

#         try:
#             img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         except Exception as e:
#             self.get_logger().error(f'CV bridge error: {e}')
#             return

#         # compute per-channel absolute difference
#         # convert to int to avoid uint8 underflow on subtraction
#         diff_b = np.abs(img[:, :, 0].astype(np.int32) - int(self.target_bgr[0]))
#         diff_g = np.abs(img[:, :, 1].astype(np.int32) - int(self.target_bgr[1]))
#         diff_r = np.abs(img[:, :, 2].astype(np.int32) - int(self.target_bgr[2]))

#         mask = (diff_b <= int(self.bgr_tolerance[0])) & \
#                (diff_g <= int(self.bgr_tolerance[1])) & \
#                (diff_r <= int(self.bgr_tolerance[2]))

#         mask_u8 = (mask.astype(np.uint8) * 255)

#         # Morphological clean-up to reduce noise
#         kernel = np.ones((3, 3), np.uint8)
#         mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
#         mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)

#         count = int(cv2.countNonZero(mask_u8))

#         # publish mask for debugging
#         try:
#             mask_msg = self.bridge.cv2_to_imgmsg(mask_u8, encoding='mono8')
#             self.mask_publisher.publish(mask_msg)
#         except Exception:
#             pass

#         self.get_logger().debug(f'BGR-match count={count}, min_area={self.min_area}')

#         if count >= self.min_area:
#             self._green_count += 1
#         else:
#             if self._green_count > 0:
#                 self._green_count -= 1

#         if self._green_count >= self.debounce_frames:
#             self._allowed = True
#             self.go_publisher.publish(Bool(data=True))
#             self.get_logger().info('Green DETECTED (BGR) — allowing motion (published True on /traffic_light_go)')


# def main(args=None):
#     rclpy.init(args=args)
#     node = TrafficLightWatcher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
import cv2
import numpy as np


class TrafficLightWatcher(Node):
    """
    Простая BGR-детекция: ищет пиксели близкие к целевому цвету (B,G,R)
    в небольшом допуске. Если найдено достаточно пикселей — публикует True
    на `/traffic_light_go`. Также публикует маску на `/traffic_light/mask`
    для визуальной отладки.
    """

    def __init__(self):
        super().__init__('traffic_light_watcher')

        # Целевой цвет и допуски (B, G, R) — можно менять через ros2 param set
        self.declare_parameter('target_bgr', [0, 189, 0])
        self.declare_parameter('bgr_tolerance', [40, 60, 40])  # tol for B, G, R

        # Порог площади и дебаунс (в кадрах)
        self.declare_parameter('min_area', 10)
        self.declare_parameter('debounce_frames', 3)

        tb = self.get_parameter('target_bgr').value
        tol = self.get_parameter('bgr_tolerance').value
        self.target_bgr = np.array(tb, dtype=np.int32)
        self.bgr_tolerance = np.array(tol, dtype=np.int32)

        self.min_area = int(self.get_parameter('min_area').value)
        self.debounce_frames = int(self.get_parameter('debounce_frames').value)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/color/image', self.image_cb, 10)
        self.go_publisher = self.create_publisher(Bool, '/traffic_light_go', 10)
        self.mask_publisher = self.create_publisher(Image, '/traffic_light/mask', 10)

        # clock subscription and race start publisher
        self.current_clock = None  # float seconds
        self.clock_sub = self.create_subscription(Time, '/clock', self.clock_cb, 10)
        self.race_start_pub = self.create_publisher(Float32, '/race_start', 10)
        self._race_started_published = False

        self._green_count = 0
        self._allowed = False

        # начальное состояние — стоп
        self.go_publisher.publish(Bool(data=False))
        self.get_logger().info(f'TrafficLightWatcher started: target_bgr={self.target_bgr.tolist()}, tol={self.bgr_tolerance.tolist()}')

    def clock_cb(self, msg: Time):
        # keep the latest simulated/ros clock time in seconds
        self.current_clock = float(msg.sec) + float(msg.nanosec) * 1e-9

    def image_cb(self, msg: Image):
        if self._allowed:
            return  # уже разрешено движение; latched

        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        # compute per-channel absolute difference
        # convert to int to avoid uint8 underflow on subtraction
        diff_b = np.abs(img[:, :, 0].astype(np.int32) - int(self.target_bgr[0]))
        diff_g = np.abs(img[:, :, 1].astype(np.int32) - int(self.target_bgr[1]))
        diff_r = np.abs(img[:, :, 2].astype(np.int32) - int(self.target_bgr[2]))

        mask = (diff_b <= int(self.bgr_tolerance[0])) & \
               (diff_g <= int(self.bgr_tolerance[1])) & \
               (diff_r <= int(self.bgr_tolerance[2]))

        mask_u8 = (mask.astype(np.uint8) * 255)

        # Morphological clean-up to reduce noise
        kernel = np.ones((3, 3), np.uint8)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)

        count = int(cv2.countNonZero(mask_u8))

        # publish mask for debugging
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_u8, encoding='mono8')
            self.mask_publisher.publish(mask_msg)
        except Exception:
            pass

        self.get_logger().debug(f'BGR-match count={count}, min_area={self.min_area}')

        if count >= self.min_area:
            self._green_count += 1
        else:
            if self._green_count > 0:
                self._green_count -= 1

        if self._green_count >= self.debounce_frames:
            self._allowed = True
            self.go_publisher.publish(Bool(data=True))
            # publish the race start time (once) using the ROS clock value
            if (not self._race_started_published) and (self.current_clock is not None):
                try:
                    tmsg = Float32()
                    tmsg.data = float(self.current_clock)
                    self.race_start_pub.publish(tmsg)
                    self._race_started_published = True
                    self.get_logger().info(f'Published race start time {tmsg.data} on /race_start')
                except Exception as e:
                    self.get_logger().error(f'Failed to publish race start: {e}')
            self.get_logger().info('Green DETECTED (BGR) — allowing motion (published True on /traffic_light_go)')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()