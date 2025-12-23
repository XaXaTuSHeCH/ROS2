import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        self.debug_mode = True

        # === Bird-eye view ===
        self.warped_width = 640
        self.warped_height = 480
        self.screen_mid_x = self.warped_width / 2.0

        self.source_points = np.float32([
            [195, 278],
            [645, 278],
            [805, 455],
            [35, 455]
        ])
        self.destination_points = np.float32([
            [0, 0],
            [self.warped_width, 0],
            [self.warped_width, self.warped_height],
            [0, self.warped_height]
        ])
        self.M = cv2.getPerspectiveTransform(self.source_points, self.destination_points)

        # === Детекция линий ===
        self.track_width_px = 517
        self.yellow_bias_normal = 0.42
        self.yellow_bias_left = 0.32     # ближе к жёлтой при left
        self.yellow_bias_right = 0.68    # ближе к белой при right
        self.yellow_bias = self.yellow_bias_normal
        self.min_area = 200

        self.lower_yellow = np.array([15, 70, 70])
        self.upper_yellow = np.array([40, 255, 255])
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 35, 255])

        # === Детекция знака ===
        self.lower_blue = np.array([90, 80, 80])
        self.upper_blue = np.array([150, 255, 255])
        self.sign_min_area = 200
        self.sign_max_area = 7000

        # === Параметры поворота на кольце ===
        self.turn_direction = None
        self.turn_start_time = None

        # Общее время bias_hold (для обоих направлений)
        self.bias_hold_duration = 1.8

        # Параметры фиксированного поворота — разные для left и right!
        self.fixed_linear_right = 0.28          # для right — едем вперёд
        self.fixed_angular_right = -0.65        # мягкий поворот направо

        self.fixed_linear_left = 0.08            # для left — почти на месте!
        self.fixed_angular_left = 1.1            # резко налево
        self.fixed_turn_duration_left = 2.2     # дольше, чтобы успеть >90° 2,4

        self.fixed_turn_duration_right = 0     # для right — короче, он и так проходит
        self.direction_locked = False # флаг того что направление уже зафиксировано
        self.crossroad_passed = False

        # ROS
        self.subscription = self.create_subscription(Image, '/color/image', self.process_image, 10)
        self.error_publisher = self.create_publisher(Float64, '/line_error', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.previous_error = 0.0
        self.last_direction = 0

    def process_image(self, msg):
        try:
            raw_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        # 1. Bird-eye и детекция линий (без изменений)
        warped = cv2.warpPerspective(raw_image, self.M, (self.warped_width, self.warped_height))
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

        my = cv2.moments(yellow_mask)
        mw = cv2.moments(white_mask)

        yellow_found = my['m00'] > self.min_area
        white_found = mw['m00'] > self.min_area

        cx_y = int(my['m10'] / my['m00']) if yellow_found else 0
        cx_w = int(mw['m10'] / mw['m00']) if white_found else 0

        target_x = self.screen_mid_x
        dist_from_yellow = self.track_width_px * self.yellow_bias

        if yellow_found and white_found:
            if cx_w > cx_y + (self.track_width_px / 3):
                target_x = cx_y + dist_from_yellow
                self.last_direction = 0
            else:
                target_x = cx_y + (self.track_width_px / 2)
        elif yellow_found:
            target_x = cx_y + dist_from_yellow
            self.last_direction = -1
        elif white_found:
            target_x = cx_w - (self.track_width_px * (1.0 - self.yellow_bias))
            self.last_direction = 1
        else:
            target_x = self.screen_mid_x + (self.previous_error * self.screen_mid_x * 0.6)

        current_error = (target_x - self.screen_mid_x) / self.screen_mid_x
        smoothed_error = 0.7 * current_error + 0.3 * self.previous_error
        smoothed_error = np.clip(smoothed_error, -1.0, 1.0)
        self.previous_error = smoothed_error

        self.error_publisher.publish(Float64(data=smoothed_error))

        # 2. Детекция знака
        if not self.crossroad_passed:
            raw_hsv = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)
            blue_mask = cv2.inRange(raw_hsv, self.lower_blue, self.upper_blue)
            kernel = np.ones((3,3), np.uint8)
            blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            sign_dbg = raw_image.copy()
            sign_detected = False

            for cnt in contours:
                area = cv2.contourArea(cnt)
                cv2.drawContours(sign_dbg, [cnt], -1, (0, 255, 0), 2)

                if self.sign_min_area < area < self.sign_max_area:
                    
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect = w / h if h > 0 else 0
                    if 0.7 < aspect < 1.3:
                        sign_region = raw_image[y:y+h, x:x+w]
                        sign_region_hsv = cv2.cvtColor(sign_region, cv2.COLOR_BGR2HSV)
                        arrow_mask = cv2.inRange(sign_region_hsv, self.lower_white, self.upper_white)
                        m_arrow = cv2.moments(arrow_mask)
                        if m_arrow['m00'] > 50:
                            arrow_cx = int(m_arrow['m10'] / m_arrow['m00'])
                            sign_center_x = w / 2.0

                            # ИСПРАВЛЕНО: стрелка левее центра → поворот налево
                            detected_direction = "left" if arrow_cx > sign_center_x else "right"
                            if self.turn_start_time is None and not self.direction_locked:
                                self.turn_direction = detected_direction
                                self.turn_start_time = time.time()
                                self.direction_locked = True   # Больше не меняем направление!

                                self.get_logger().info(
                                    f"ЗНАК ДЕТЕКТИРОВАН: поворот {self.turn_direction.upper()}! "
                                    f"arrow_cx={arrow_cx}, center={sign_center_x:.1f}"
                                )

                            # if self.turn_start_time is None:
                            #     self.turn_direction = detected_direction
                            #     self.turn_start_time = time.time()
                            #     self.get_logger().info(
                            #         f"ЗНАК ДЕТЕКТИРОВАН: поворот {self.turn_direction.upper()}! "
                            #         f"arrow_cx={arrow_cx}, center={sign_center_x:.1f}"
                            #     )

                            sign_detected = True
                            cv2.rectangle(sign_dbg, (x, y), (x+w, y+h), (0, 255, 255), 3)
                            cv2.circle(sign_dbg, (x + arrow_cx, y + h//2), 8, (0, 0, 255), -1)
                            break

            if not sign_detected:
                cv2.putText(sign_dbg, "No sign", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        else:
            sign_dbg = raw_image.copy()
            cv2.putText(sign_dbg, "Crossroad passed - ignoring signs", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 3. Управление поворотом — разные параметры для left/right
        current_time = time.time()
        if self.turn_direction is not None and self.turn_start_time is not None:
            elapsed = current_time - self.turn_start_time

            if self.turn_direction == "left":
                fixed_duration = self.fixed_turn_duration_left
                if elapsed < fixed_duration:
                    twist = Twist()
                    twist.linear.x = self.fixed_linear_left      # почти на месте
                    twist.angular.z = self.fixed_angular_left     # резко налево
                    self.cmd_vel_publisher.publish(twist)
                elif elapsed < fixed_duration + self.bias_hold_duration:
                    self.yellow_bias = self.yellow_bias_left
                else:
                    self.reset_turn_state()

            else:  # right
                # fixed_duration = self.fixed_turn_duration_right
                # if elapsed < fixed_duration:
                #     twist = Twist()
                #     twist.linear.x = self.fixed_linear_right
                #     twist.angular.z = self.fixed_angular_right
                #     self.cmd_vel_publisher.publish(twist)
                # elif elapsed < fixed_duration + self.bias_hold_duration:
                #     self.yellow_bias = self.yellow_bias_right
                # else:
                self.reset_turn_state()

        # 4. Debug
        if self.debug_mode:
            dbg = warped.copy()
            if yellow_found:
                cv2.circle(dbg, (cx_y, self.warped_height//2), 8, (0, 255, 255), -1)
            if white_found:
                cv2.circle(dbg, (cx_w, self.warped_height//2), 8, (255, 255, 255), -1)
            cv2.circle(dbg, (int(target_x), self.warped_height//2), 12, (0, 255, 0), -1)
            cv2.line(dbg, (int(self.screen_mid_x), 0), (int(self.screen_mid_x), self.warped_height), (255, 0, 0), 3)

            if self.turn_direction:
                status = f"Ring turn: {self.turn_direction.upper()} (elapsed: {elapsed:.1f}s)"
                cv2.putText(dbg, status, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow("Lane Detector [roseast]", dbg)
            cv2.imshow("Sign Debug [roseast]", sign_dbg)
            cv2.waitKey(1)

    def reset_turn_state(self):
        self.yellow_bias = self.yellow_bias_normal
        self.turn_direction = None
        self.turn_start_time = None
        self.crossroad_passed = True
        self.get_logger().info("Поворот на кольце завершён")
        


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()