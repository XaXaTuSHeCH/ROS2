import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
import numpy as np
import time

class Overtaking(Node):
    def __init__(self):
        super().__init__('overtaking')
        self.debug = True

        # Параметры
        self.obstacle_threshold = 1.5  # m, если ближе — препятствие
        self.safe_side_distance = 0.5  # m, минимальное расстояние сбоку
        self.overtake_bias = 0.5  # сдвиг bias для обгона (вправо/влево, в зависимости от карты)
        self.crawl_speed = 0.08  # медленная скорость для маневра
        self.normal_speed = 0.1
        self.overtake_duration = 10.0  # макс время обгона, для таймаута (настрой по карте)

        # Состояние
        self.overtaking_active = False
        self.obstacle_detected = False
        self.overtake_start_time = None
        self.direction = "right"  # предполагаем обгон справа; измени по карте (left/right)

        # Подписки
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_line_error = self.create_subscription(Float64, '/line_error', self.line_error_cb, 10)
        self.sub_overtaking_active = self.create_subscription(Bool, '/overtaking_active', self.active_cb, 10)

        # Публикации
        self.pub_override_flag = self.create_publisher(Bool, '/control_override', 10)
        self.pub_override_twist = self.create_publisher(Twist, '/override_cmd_vel', 10)
        self.pub_speed = self.create_publisher(Float64, '/target_speed', 10)
        self.pub_overtaking_active = self.create_publisher(Bool, '/overtaking_active', 10)  # для сброса назад

        self.latest_error = 0.0
        self.get_logger().info("Overtaking node started: waiting for activation after cones...")

    def active_cb(self, msg: Bool):
        self.overtaking_active = msg.data
        if self.overtaking_active:
            self.get_logger().info("Overtaking mode activated")
        else:
            self.reset_state()
            self.get_logger().info("Overtaking mode deactivated externally")

    def line_error_cb(self, msg: Float64):
        self.latest_error = msg.data

    def scan_cb(self, msg: LaserScan):
        if not self.overtaking_active:
            return

        now = time.time()
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))  # более точный arange
        ranges = np.array(msg.ranges)

        # Фронт: -30° to 30° (впереди)
        front_mask = (angles > -np.pi/6) & (angles < np.pi/6)
        front_ranges = ranges[front_mask]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        min_front = np.min(front_ranges) if len(front_ranges) > 0 else float('inf')

        # Бок: для обгона (right: 30° to 90°, left: -30° to -90°)
        if self.direction == "right":
            side_mask = (angles > np.pi/6) & (angles < np.pi/2)
        else:
            side_mask = (angles < -np.pi/6) & (angles > -np.pi/2)
        side_ranges = ranges[side_mask]
        side_ranges = side_ranges[np.isfinite(side_ranges)]
        min_side = np.min(side_ranges) if len(side_ranges) > 0 else float('inf')

        self.obstacle_detected = min_front < self.obstacle_threshold

        if self.obstacle_detected:
            if self.overtake_start_time is None:
                self.overtake_start_time = now
                self.get_logger().info("SUV detected — starting overtake maneuver")

            # Логика маневра: замедлить, сдвинуться в сторону
            if min_side > self.safe_side_distance and abs(self.latest_error) < 0.8:  # безопасно сбоку и не выезд за разметку
                self._set_override(True)
                twist = Twist()
                twist.linear.x = self.crawl_speed
                twist.angular.z = -0.4 if self.direction == "right" else 0.4  # поворот в сторону обгона (отрицательный для right, если углы в ROS стандартные)
                self.pub_override_twist.publish(twist)
                self.pub_speed.publish(Float64(data=self.crawl_speed))
            else:
                # Экстренный стоп если не безопасно
                self._set_override(True)
                stop = Twist()
                self.pub_override_twist.publish(stop)
                self.get_logger().warn("Unsafe conditions (side too close or line error high) — emergency stop")

        else:
            # Нет препятствия впереди — предполагаем, что обгон завершён (SUV позади)
            if self.overtake_start_time and now - self.overtake_start_time > 3.0:  # debounce 3s
                self.pub_overtaking_active.publish(Bool(data=False))  # деактивируем режим
                self.reset_state()

        # Таймаут на весь обгон
        if self.overtake_start_time and now - self.overtake_start_time > self.overtake_duration:
            self.pub_overtaking_active.publish(Bool(data=False))
            self.reset_state()
            self.get_logger().warn("Overtake timeout — forcing reset")

    def reset_state(self):
        self._set_override(False)
        self.overtake_start_time = None
        self.obstacle_detected = False
        self.overtaking_active = False
        self.pub_speed.publish(Float64(data=self.normal_speed))
        self.get_logger().info("Overtake completed — returning to normal line following")

    def _set_override(self, enabled: bool):
        self.pub_override_flag.publish(Bool(data=enabled))

def main(args=None):
    rclpy.init(args=args)
    node = Overtaking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()