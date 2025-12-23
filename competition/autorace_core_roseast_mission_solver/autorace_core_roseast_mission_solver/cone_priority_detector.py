import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class UnifiedDetector(Node):
    def __init__(self):
        super().__init__('cone_priority_detector')
        self.debug_mode = True

        # === Bird-eye view ===
        self.warped_width = 640
        self.warped_height = 480
        self.screen_mid_x = self.warped_width / 2.0
        self.source_points = np.float32([[195, 278], [645, 278], [805, 455], [35, 455]])
        self.destination_points = np.float32([[0, 0], [self.warped_width, 0],
                                              [self.warped_width, self.warped_height], [0, self.warped_height]])
        self.M = cv2.getPerspectiveTransform(self.source_points, self.destination_points)

        # === Линии ===
        self.track_width_px = 517
        self.yellow_bias_normal = 0.42
        self.yellow_bias_left = 0.32
        self.yellow_bias_right = 0.68
        self.yellow_bias = self.yellow_bias_normal

        self.min_area = 200
        self.lower_yellow = np.array([15, 70, 70])
        self.upper_yellow = np.array([40, 255, 255])
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 35, 255])

        # === Знак ===
        self.lower_blue = np.array([90, 80, 80])
        self.upper_blue = np.array([150, 255, 255])
        self.sign_min_area = 200
        self.sign_max_area = 7000

        self.turn_direction = None
        self.turn_start_time = None
        self.bias_hold_duration = 1.8

        self.fixed_linear_right = 0.28
        self.fixed_angular_right = -0.65
        self.fixed_linear_left = 0.08
        self.fixed_angular_left = 1.1
        self.fixed_turn_duration_left = 2.1 #8.2
        self.fixed_turn_duration_right = 0.0

        self.debounce_count = 0
        self.previous_detected_direction = None
        self.DEBOUNCE_REQUIRED = 3  # кадров подряд для подтверждения

        self.lower_white_bgr = np.array([100, 100, 0])
        self.upper_white_bgr = np.array([255, 255, 255])
        self.direction_locked = False
        self.crossroad_passed = False

        # === Конусы ===
        self.cone_lower_orange = np.array([0, 100, 80])
        self.cone_upper_orange = np.array([20, 255, 255])
        self.min_cone_area = 100

        self.PUSH_DURATION = 1.5
        self.COOLDOWN_DURATION = 5
        self.PUSH_ERROR = 1.2

        self.cone_push_until = 0.0
        self.cone_cooldown_until = 0.0
        self.cone_push_sign = 0.0

        # === Скорости ===
        self.NORMAL_SPEED = 0.1
        self.CRAWL_SPEED = 0.09

        # lidar
        self.pub_overtaking_active = self.create_publisher(Bool, '/overtaking_active', 10)
        self.overtaking_started = False  # флаг, чтобы не спамить True
        self.cones_finished_time = None  # время окончания конусов
        self.BUFFER_AFTER_CONES = 5.0  # секунды буфера после последнего конуса (настрой по тестам)

        # === ЛОГИКА ФИНИША И ТАЙМЕР ===
        self.race_start_time = None
        self.RACE_DURATION = 260.0   # изменил как у тебя
        self.finished = False

        # НОВОЕ: для логирования времени каждую секунду
        self.last_logged_second = -1

        # ROS
        self.bridge = CvBridge()
        self.previous_error = 0.0

        self.sub = self.create_subscription(Image, '/color/image', self.process_image, 10)
        self.sub_traffic_go = self.create_subscription(Bool, '/traffic_light_go', self.on_traffic_go, 10)

        self.pub_error = self.create_publisher(Float64, '/line_error', 10)
        self.pub_speed = self.create_publisher(Float64, '/target_speed', 10)
        self.pub_override_flag = self.create_publisher(Bool, '/control_override', 10)
        self.pub_override_twist = self.create_publisher(Twist, '/override_cmd_vel', 10)
        self.pub_finish = self.create_publisher(String, '/robot_finish', 10)

        self.pub_race_start = self.create_publisher(Float64, '/race_start_time', 10)
        self.get_logger().info("UNIFIED: line+sign+cone + timer finish + elapsed time logging")

    def on_traffic_go(self, msg: Bool):
        if msg.data and self.race_start_time is None:
            self.race_start_time = time.time()
            self.last_logged_second = -1
            self.get_logger().info(f"RACE STARTED! Target duration: {self.RACE_DURATION} seconds.")

            # Публикуем время старта (один раз)
            start_msg = Float64()
            start_msg.data = float(self.race_start_time)
            self.pub_race_start.publish(start_msg)
            self.get_logger().info(f"Published race start time: {self.race_start_time:.3f}")

    def process_image(self, msg):
        if self.finished:
            return

        try:
            raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return

        now = time.time()
        warped = cv2.warpPerspective(raw, self.M, (self.warped_width, self.warped_height))
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        # === ЛОГИРОВАНИЕ ПРОШЕДШЕГО ВРЕМЕНИ КАЖДУЮ СЕКУНДУ ===
        if self.race_start_time is not None:
            elapsed_race = now - self.race_start_time
            current_second = int(elapsed_race)

            if current_second != self.last_logged_second and current_second <= self.RACE_DURATION:
                self.last_logged_second = current_second
                remaining = max(0, int(self.RACE_DURATION - elapsed_race))
                self.get_logger().info(f"Race time: {current_second}s / {int(self.RACE_DURATION)}s (remaining: {remaining}s)")

            # === ПРОВЕРКА ФИНИША ПО ТАЙМЕРУ ===
            if elapsed_race >= self.RACE_DURATION:
                self.get_logger().info(f"RACE FINISHED! Total time: {elapsed_race:.1f}s")
                self.finished = True

                finish_msg = String()
                finish_msg.data = 'roseast'
                self.pub_finish.publish(finish_msg)
                self.get_logger().info("Published 'roseast' to /robot_finish")

                self._set_override(True)
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                self.pub_override_twist.publish(stop_twist)

                return

        # Остальная логика без изменений (повороты, конусы, линии, знаки)
        # ------------------------------------------------------------
        # 1) Обработка активного поворота по знаку
        # ------------------------------------------------------------
        if self.turn_direction is not None and self.turn_start_time is not None:
            elapsed = now - self.turn_start_time

            if self.turn_direction == "left":
                if elapsed < self.fixed_turn_duration_left:
                    self._set_override(True)
                    t = Twist()
                    t.linear.x = self.fixed_linear_left
                    t.angular.z = self.fixed_angular_left
                    self.pub_override_twist.publish(t)
                    return
                elif elapsed < self.fixed_turn_duration_left + self.bias_hold_duration:
                    self.yellow_bias = self.yellow_bias_left
                    self._set_override(False)
                else:
                    self.reset_turn_state()

            else:  # right
                if self.fixed_turn_duration_right > 0.0 and elapsed < self.fixed_turn_duration_right:
                    self._set_override(True)
                    t = Twist()
                    t.linear.x = self.fixed_linear_right
                    t.angular.z = self.fixed_angular_right
                    self.pub_override_twist.publish(t)
                    return
                elif elapsed < self.fixed_turn_duration_right + self.bias_hold_duration:
                    self.yellow_bias = self.yellow_bias_right
                    self._set_override(False)
                else:
                    self.reset_turn_state()

        self._set_override(False)

        # ------------------------------------------------------------
        # 2) Конусный “пинок”
        # ------------------------------------------------------------
        if now < self.cone_push_until:
            err = float(self.PUSH_ERROR * self.cone_push_sign)
            self.pub_error.publish(Float64(data=err))
            self.pub_speed.publish(Float64(data=self.CRAWL_SPEED))
            return

        can_detect_cones = now >= self.cone_cooldown_until
        if can_detect_cones:
            mask_o = cv2.inRange(hsv, self.cone_lower_orange, self.cone_upper_orange)
            mask_o[0:int(self.warped_height * 0.4), :] = 0

            half = int(self.screen_mid_x)
            mask_left = mask_o[:, :half]
            mask_right = mask_o[:, half:]

            m_l = cv2.moments(mask_left)
            m_r = cv2.moments(mask_right)
            l_found = m_l['m00'] > self.min_cone_area
            r_found = m_r['m00'] > self.min_cone_area

            if l_found or r_found:
                self.cone_push_sign = 1.0 if l_found else -1.0
                self.cone_push_until = now + self.PUSH_DURATION
                self.cone_cooldown_until = now + self.PUSH_DURATION + self.COOLDOWN_DURATION
                return
        # Проверка окончания конусов
        if now > self.cone_cooldown_until and self.cones_finished_time is None:
            self.cones_finished_time = now
            self.get_logger().info("Cones section finished — starting buffer")

        if self.cones_finished_time and now > self.cones_finished_time + self.BUFFER_AFTER_CONES and not self.overtaking_started:
            self.overtaking_started = True
            self.pub_overtaking_active.publish(Bool(data=True))
            self.get_logger().info("Buffer after cones expired — activating overtaking mode")

        # ------------------------------------------------------------
        # 3) Линии (BASE)
        # ------------------------------------------------------------
        mask_y = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_w = cv2.inRange(hsv, self.lower_white, self.upper_white)

        my = cv2.moments(mask_y)
        mw = cv2.moments(mask_w)

        yellow_found = my['m00'] > self.min_area
        white_found = mw['m00'] > self.min_area

        cx_y = int(my['m10'] / my['m00']) if yellow_found else 0
        cx_w = int(mw['m10'] / mw['m00']) if white_found else 0

        target_x = self.screen_mid_x
        dist_from_yellow = self.track_width_px * self.yellow_bias

        if yellow_found and white_found:
            if cx_w > cx_y + (self.track_width_px / 3):
                target_x = cx_y + dist_from_yellow
            else:
                target_x = cx_y + (self.track_width_px / 2)
        elif yellow_found:
            target_x = cx_y + dist_from_yellow
        elif white_found:
            target_x = cx_w - (self.track_width_px * (1.0 - self.yellow_bias))
        else:
            target_x = self.screen_mid_x + (self.previous_error * self.screen_mid_x * 0.6)

        err = (target_x - self.screen_mid_x) / self.screen_mid_x
        smoothed = 0.7 * err + 0.3 * self.previous_error
        smoothed = float(np.clip(smoothed, -1.0, 1.0))
        self.previous_error = smoothed

        self.pub_error.publish(Float64(data=smoothed))
        self.pub_speed.publish(Float64(data=self.NORMAL_SPEED))

        # ------------------------------------------------------------
        # 4) Детекция знака
        # ------------------------------------------------------------
        if not self.crossroad_passed:
            self._detect_sign(raw)

    def _detect_sign(self, raw_image):
        raw_hsv = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(raw_hsv, self.lower_blue, self.upper_blue)
        kernel = np.ones((3, 3), np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_raw = raw_image.copy()  # Для отладки всего изображения

        detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.sign_min_area < area < self.sign_max_area):
                continue

            # Фильтр на полноту: если площадь слишком мала (обрезан знак), пропускаем
            if area < self.sign_min_area * 2:  # Пример: минимум вдвое больше min_area для полноты
                self.get_logger().info("Sign too small/fragmented — skipping detection")
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect = (w / h) if h > 0 else 0.0
            if not (0.7 < aspect < 1.3):
                continue

            # Рисуем bounding box синий круг/знак
            cv2.rectangle(debug_raw, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Синий bounding box

            sign_region = raw_image[y:y+h, x:x+w]
            sign_hsv = cv2.cvtColor(sign_region, cv2.COLOR_BGR2HSV)

            arrow_mask = cv2.inRange(sign_hsv, self.lower_white, self.upper_white)
            arrow_contours, _ = cv2.findContours(arrow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            debug_sign_region = sign_region.copy()

            # Рисуем контуры белой стрелки
            cv2.drawContours(debug_sign_region, arrow_contours, -1, (0, 255, 0), 2)  # Зеленые контуры стрелки

            m_arrow = cv2.moments(arrow_mask)
            if m_arrow['m00'] <= 50:
                continue

            arrow_cx = int(m_arrow['m10'] / m_arrow['m00'])
            arrow_cy = int(m_arrow['m01'] / m_arrow['m00'])

            # Рисуем центр масс стрелки
            cv2.circle(debug_sign_region, (arrow_cx, arrow_cy), 5, (0, 0, 255), -1)  # Красный круг

            # Рисуем центр знака
            sign_center_x = int(w / 2.0)
            cv2.line(debug_sign_region, (sign_center_x, 0), (sign_center_x, h), (255, 0, 0), 2)  # Синяя линия

            sign_center_y = int(h / 2.0)
            cv2.circle(debug_sign_region, (sign_center_x, sign_center_y), 5, (255, 255, 0), -1)  # Желтый круг для центра знака

            # НОВАЯ ЛОГИКА: Анализ формы (линии и контуры)
            # Детекция линий с HoughLinesP
            edges = cv2.Canny(arrow_mask, 50, 150)
            lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=20, minLineLength=10, maxLineGap=5)

            vertical_lines_right = 0
            horizontal_lines = 0
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    # Рисуем линии на debug для визуализации
                    cv2.line(debug_sign_region, (x1, y1), (x2, y2), (255, 0, 255), 2)  # Фиолетовые линии

                    # Угол линии
                    angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                    if abs(angle) > 60:  # Вертикальная (~90°)
                        if (x1 + x2) / 2 > sign_center_x:  # В правой половине
                            vertical_lines_right += 1
                    elif abs(angle) < 30:  # Горизонтальная (~0°)
                        horizontal_lines += 1

            # Анализ формы контура (для треугольника/горизонтальной)
            is_triangle_like = False
            if arrow_contours:
                approx = cv2.approxPolyDP(arrow_contours[0], 0.02 * cv2.arcLength(arrow_contours[0], True), closed=True)
                if len(approx) == 3:  # Треугольник
                    is_triangle_like = True

            # Определение направления по новой логике
            if vertical_lines_right > 0:  # Вертикальная линия справа — левая стрелка
                detected_direction = "left"
            elif horizontal_lines > 1 or is_triangle_like:  # Горизонтальные или треугольник — правая стрелка
                detected_direction = "right"
            else:
                # Fallback на центр масс, если форма неоднозначна
                detected_direction = "left" if arrow_cx > sign_center_x else "right"

            # Debounce (как раньше)
            if detected_direction == self.previous_detected_direction:
                self.debounce_count += 1
            else:
                self.debounce_count = 1
                self.previous_detected_direction = detected_direction

            if self.debounce_count >= self.DEBOUNCE_REQUIRED:
                if self.turn_start_time is None and not self.direction_locked:
                    self.turn_direction = detected_direction
                    self.turn_start_time = time.time()
                    self.direction_locked = True
                    self.right_turn_attempts = 0  # Сброс попыток для right
                    self.get_logger().info(f"SIGN DETECTED: {self.turn_direction.upper()} (after debounce and shape analysis)")
                    detected = True

            # Публикуем debug изображения (даже если не подтверждено, для отладки)
            try:
                # Вставляем debug_sign_region обратно в debug_raw для полного вида
                debug_raw[y:y+h, x:x+w] = debug_sign_region

                debug_msg = self.bridge.cv2_to_imgmsg(debug_raw, encoding='bgr8')
                self.debug_pub_sign.publish(debug_msg)

                arrow_mask_msg = self.bridge.cv2_to_imgmsg(arrow_mask, encoding='mono8')
                self.debug_pub_arrow_mask.publish(arrow_mask_msg)
            except Exception as e:
                self.get_logger().error(f"Debug publish error: {str(e)}")

            # Локальное окно для дебага
            # if self.debug_mode:
            #     cv2.imshow("Sign Detection Debug", debug_raw)
            #     cv2.imshow("Arrow Mask", arrow_mask)
            #     cv2.waitKey(1)

            if detected:
                break  # Выходим после первого подтвержденного

    def reset_turn_state(self):
        self.yellow_bias = self.yellow_bias_normal
        self.turn_direction = None
        self.turn_start_time = None
        self.direction_locked = False
        self.crossroad_passed = True
        self.get_logger().info("Turn finished, back to normal")

    def _set_override(self, enabled: bool):
        self.pub_override_flag.publish(Bool(data=bool(enabled)))

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()