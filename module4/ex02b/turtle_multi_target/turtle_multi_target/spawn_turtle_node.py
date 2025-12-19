import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class SimpleSpawn(Node):

    def __init__(self):
        super().__init__('simple_spawn_turtle3')
        self.spawn_turtle()

    def spawn_turtle(self):
        self.cli = self.create_client(Spawn, 'spawn')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = Spawn.Request()
        request.name = 'turtle3'
        request.x = 3.0
        request.y = 3.0
        request.theta = 0.0
        
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Spawned {response.name}')
        except Exception as e:
            self.get_logger().error(f'Failed: {e}')
        finally:
            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = SimpleSpawn()
    rclpy.spin(node)
