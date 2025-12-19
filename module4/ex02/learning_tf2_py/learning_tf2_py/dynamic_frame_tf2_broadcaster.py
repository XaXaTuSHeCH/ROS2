import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('direction_of_rotation', 1)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        
        self.angle = 0.0

    def broadcast_timer_callback(self):
        radius = self.get_parameter('radius').get_parameter_value().double_value
        direction = self.get_parameter('direction_of_rotation').get_parameter_value().integer_value
        
        angular_speed = 1.0
        self.angle += direction * angular_speed * 0.1
        
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi
        elif self.angle < -2 * math.pi:
            self.angle += 2 * math.pi

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        
        t.transform.translation.x = radius * math.cos(self.angle)
        t.transform.translation.y = radius * math.sin(self.angle)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()