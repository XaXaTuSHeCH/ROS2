import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import sys

class MoveToGoal(Node):
    def __init__(self, x_goal, y_goal, theta_goal):
        super().__init__('move_to_goal')

        self.goal_x = x_goal
        self.goal_y = y_goal
        self.goal_theta = theta_goal

        self.pose = None

        self.sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            return

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y

        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.pose.theta)

        cmd = Twist()

        if abs(angle_error) > 0.1:
            cmd.angular.z = 2.0 * angle_error
        elif distance > 0.1:
            cmd.linear.x = 2.0 * distance
        else:
            final_error = self.normalize_angle(self.goal_theta - self.pose.theta)
            if abs(final_error) > 0.1:
                cmd.angular.z = 2.0 * final_error
            else:
                self.pub.publish(Twist())
                rclpy.shutdown()
                return

        self.pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    rclpy.init()

    if len(sys.argv) != 4:
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3])

    node = MoveToGoal(x, y, theta)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
