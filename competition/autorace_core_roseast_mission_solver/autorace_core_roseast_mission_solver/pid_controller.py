import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist

class PidLaneController(Node):
    """
    PID по /line_error.
    Движение разрешается /traffic_light_go (Bool).
    Если /control_override=True -> публикуем /override_cmd_vel и игнорируем PID.
    Скорость берём из параметра speed, но можно менять через /target_speed (Float64).
    """

    def __init__(self):
        super().__init__('pid_lane_controller')

        self.declare_parameter('kp', 0.55)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.06)
        self.declare_parameter('speed', 0.1)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.base_speed_param = float(self.get_parameter('speed').value)
        self.current_speed = self.base_speed_param

        self.integral_error = 0.0
        self.previous_error = 0.0
        self.max_integral = 1.0

        self.allowed = False

        self.override_active = False
        self.override_twist = Twist()

        self.latest_error = 0.0
        self.last_pid_twist = Twist()
        self.last_pid_twist.linear.x = 0.0
        self.last_pid_twist.angular.z = 0.0

        self.subscription_go = self.create_subscription(Bool, '/traffic_light_go', self.traffic_cb, 10)
        self.subscription_err = self.create_subscription(Float64, '/line_error', self.error_callback, 10)

        self.sub_speed = self.create_subscription(Float64, '/target_speed', self.speed_cb, 10)
        self.sub_override_flag = self.create_subscription(Bool, '/control_override', self.override_flag_cb, 10)
        self.sub_override_twist = self.create_subscription(Twist, '/override_cmd_vel', self.override_twist_cb, 10)

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Публикуем команду стабильно (а не только при приходе /line_error)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def speed_cb(self, msg: Float64):
        # Если детектор не публикует speed — будет оставаться параметр speed
        self.current_speed = float(msg.data)

    def override_flag_cb(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state != self.override_active:
            # при переключении сбрасываем интеграл, чтобы после override не было рывка
            self.integral_error = 0.0
            self.previous_error = 0.0
        self.override_active = new_state

    def override_twist_cb(self, msg: Twist):
        self.override_twist = msg

    def error_callback(self, msg: Float64):
        self.latest_error = float(msg.data)

        # Если движение запрещено или мы в override — PID не считает
        if (not self.allowed) or self.override_active:
            return

        # PID
        self.integral_error += self.latest_error
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)

        derivative_error = self.latest_error - self.previous_error
        steering = -(self.kp * self.latest_error + self.ki * self.integral_error + self.kd * derivative_error)
        steering = max(min(steering, 1.5), -1.5)

        t = Twist()
        t.linear.x = float(self.current_speed)
        t.angular.z = float(steering)

        self.last_pid_twist = t
        self.previous_error = self.latest_error

    def control_loop(self):
        if not self.allowed:
            zero = Twist()
            self.velocity_publisher.publish(zero)
            return

        if self.override_active:
            self.velocity_publisher.publish(self.override_twist)
        else:
            self.velocity_publisher.publish(self.last_pid_twist)

    def traffic_cb(self, msg: Bool):
        self.allowed = bool(msg.data)

        if not self.allowed:
            zero = Twist()
            self.velocity_publisher.publish(zero)
            self.integral_error = 0.0
            self.previous_error = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PidLaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
