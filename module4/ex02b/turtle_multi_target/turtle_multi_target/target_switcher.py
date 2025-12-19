import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
import math
from turtle_multi_target_msg.msg import TargetInfo
import sys
import select
import tty
import termios


class TargetSwitcher(Node):

    def __init__(self):
        super().__init__('target_switcher')
        
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        self.declare_parameter('switch_threshold', 1.0)
        self.declare_parameter('initial_target', 'carrot1')
        
        self.switch_threshold = self.get_parameter('switch_threshold').get_parameter_value().double_value
        self.current_target = self.get_parameter('initial_target').get_parameter_value().string_value
        
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.target_index = self.targets.index(self.current_target) if self.current_target in self.targets else 0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        qos_profile = QoSProfile(depth=10)
        self.target_pub = self.create_publisher(TargetInfo, 'current_target', qos_profile)
        
        self.timer = self.create_timer(0.5, self.check_distance)
        
        self.keyboard_timer = self.create_timer(0.1, self.check_keyboard)
        
        self.publish_target(0, 0, 0)
        
        self.get_logger().info(f'Target switcher started. Current target: {self.current_target}')
        self.get_logger().info('Press "n" to switch target manually')

    def check_keyboard(self):
        """Check for keyboard input"""
        if self.is_key_pressed():
            key = sys.stdin.read(1)
            if key.lower() == 'n':
                self.get_logger().info('Manual switch requested')
                self.switch_to_next_target()

    def is_key_pressed(self):
        """Check if a key is pressed without blocking"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def check_distance(self):
        from_frame_rel = self.current_target
        to_frame_rel = 'turtle2'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time()
            )
            
            distance = math.sqrt(
                t.transform.translation.x ** 2 +
                t.transform.translation.y ** 2
            )
            
            t_world = self.tf_buffer.lookup_transform(
                'world',
                from_frame_rel,
                rclpy.time.Time()
            )
            target_x = t_world.transform.translation.x
            target_y = t_world.transform.translation.y
            
            self.get_logger().debug(f'Distance to {self.current_target}: {distance:.2f}')
            self.publish_target(target_x, target_y, distance)
            
            if distance < self.switch_threshold:
                self.get_logger().info(f'Reached target, switching!')
                self.switch_to_next_target()
                
        except TransformException as ex:
            self.get_logger().debug(f'Transform not ready: {ex}')
            self.publish_target(0.0, 0.0, 0.0)
            return

    def publish_target(self, target_x, target_y, distance):
        msg = TargetInfo()
        msg.target_name = self.current_target
        msg.target_x = float(target_x)
        msg.target_y = float(target_y)
        msg.distance_to_target = float(distance)
        self.target_pub.publish(msg)

    def switch_to_next_target(self):
        old_target = self.current_target
        self.target_index = (self.target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.target_index]
        self.get_logger().info(f'Switched target: {old_target} -> {self.current_target}')

    def destroy_node(self):
        """Clean up terminal settings when node is destroyed"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main():
    rclpy.init()
    node = TargetSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()