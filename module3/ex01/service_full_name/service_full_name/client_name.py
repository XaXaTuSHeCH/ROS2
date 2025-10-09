import sys
import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import FullNameSumService


class FullNameClient(Node):

    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(FullNameSumService, 'FullNameSumService')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FullNameSumService.Request()

    def send_request(self, last_name, name, first_name):
        self.req.last_name = last_name
        self.req.name = name
        self.req.first_name = first_name
        return self.cli.call_async(self.req) 


def main():
    rclpy.init()

    minimal_client = FullNameClient()
    future = minimal_client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        f'full name: {response.full_name}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()