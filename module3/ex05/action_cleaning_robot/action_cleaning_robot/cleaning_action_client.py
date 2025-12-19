import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from cleaning_interface.action import CleaningTask


class CleaningRobotActionClient(Node):

    def __init__(self):
        super().__init__('cleaning_robot_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'cleaning_robot')
        self.declare_parameter('task_type', 'return_home') # clean_square
        self.declare_parameter('area_size', 3.0)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        
        self.task_type = self.get_parameter('task_type').get_parameter_value().string_value
        self.area_size = self.get_parameter('area_size').get_parameter_value().double_value
        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value

    def send_goal(self):
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = self.task_type
        goal_msg.area_size = self.area_size
        goal_msg.target_x = self.target_x
        goal_msg.target_y = self.target_y
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result:')
        self.get_logger().info(f'\tSuccess: {result.success}')
        self.get_logger().info(f'\tCleaned_points: {result.cleaned_points}')
        self.get_logger().info(f'\tTotal_distance: {result.total_distance}')
        rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback:')
        self.get_logger().info(f'\tProgress_percent: {feedback.progress_percent}')
        self.get_logger().info(f'\tCurrent_cleaned_points: {feedback.current_cleaned_points}')
        self.get_logger().info(f'\tCurrent_x: {feedback.current_x}')
        self.get_logger().info(f'\tCurrent_y: {feedback.current_y}')


def main(args=None):
    rclpy.init(args=args)
    action_client = CleaningRobotActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()