from tutorial_interfaces.srv import FullNameSumService
import rclpy
from rclpy.node import Node

class FullNameService(Node):
    def __init__(self):
        super().__init__('full_name_service')
        self.srv = self.create_service(FullNameSumService, 'FullNameSumService', self.full_name_callback)

    def full_name_callback(self, request, response):
        response.full_name = request.last_name + ' ' + request.name + ' ' + request.first_name
        return response

def main():
    rclpy.init()
    full_name_service = FullNameService()
    rclpy.spin(full_name_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 