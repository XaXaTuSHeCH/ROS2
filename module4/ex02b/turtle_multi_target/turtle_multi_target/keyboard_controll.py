import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSwitch(Node):
    def __init__(self):
        super().__init__('keyboard_reader')
        self.pub = self.create_publisher(String, 'keyboard_input', 10)
        
    def switch_target(self):
        msg = String()
        msg.data = 'n'
        self.pub.publish(msg)
        self.get_logger().info('Target switch requested')


def main():
    rclpy.init()
    node = SimpleSwitch()
    
    try:
        while rclpy.ok():
            print("\nPress ENTER to switch target, or 'q' to quit:")
            user_input = input()
            if user_input.lower() == 'q':
                break
            else:
                node.switch_target()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()