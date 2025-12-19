import math
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from cleaning_interface.action import CleaningTask
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class CleaningRobotActionServer(Node):

    def __init__(self, epsilon=0.1):
        super().__init__('cleaning_robot_action_server')
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'cleaning_robot',
            self.execute_callback
        )
        self.publisher_ = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10
        )
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10
        )
        self.pose = None
        self.epsilon = epsilon
        
        self.cleaned_points = 0.0
        
    def listener_callback(self, msg: Pose):
        self.pose = msg
        
    def update_cleaned_points(self, last_x, last_y):
        rclpy.spin_once(self, timeout_sec=0.01)
        dx = self.pose.x - last_x
        dy = self.pose.y - last_y
        dist = math.sqrt(dx**2 + dy**2)
        self.cleaned_points += dist

    def move_forward(self, goal_handle, speed, lenght):
        rclpy.spin_once(self, timeout_sec=0.01)
        msg = Twist()
        msg.linear.x = speed
        start_x = self.pose.x
        start_y = self.pose.y

        while True:
            dx = self.pose.x - start_x
            dy = self.pose.y - start_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= lenght:
                break
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.update_cleaned_points(start_x, start_y)

    def rotate(self, goal_handle, angular_speed, angle):
        rclpy.spin_once(self, timeout_sec=0.01)
        twist = Twist()
        goal_theta = self.pose.theta + angle
        goal_theta = math.atan2(math.sin(goal_theta), math.cos(goal_theta))

        while True:
            theta = self.pose.theta
            angle_error = goal_theta - theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            if abs(angle_error) < self.epsilon**2:
                break
            twist.angular.z = angular_speed * angle_error
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)

        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def move_to_goal(self, goal_handle, speed, goal_x, goal_y, goal_theta):
        total_distance = None
        while True:
            twist = Twist()
            rclpy.spin_once(self, timeout_sec=0.01)
            x = self.pose.x
            y = self.pose.y
            theta = self.pose.theta

            dx = goal_x - x
            dy = goal_y - y
            distance = math.sqrt(dx**2 + dy**2)
            if total_distance is None:
                total_distance = distance

            if distance < self.epsilon:
                angle_error = goal_theta - theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                if abs(angle_error) > self.epsilon:
                    twist.angular.z = speed * angle_error
                else:
                    self.publisher_.publish(twist)
                    return

            else:
                goal_theta = math.atan2(dy, dx)
                angle_error = goal_theta - theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                if abs(angle_error) > self.epsilon:
                    twist.angular.z = speed * angle_error
                else:
                    twist.linear.x = speed * distance
            self.cleaned_points = total_distance - distance
            feedback_msg = CleaningTask.Feedback()
            progress = (1 - (distance / (total_distance - self.epsilon))) * 100
            feedback_msg.progress_percent = min(int(max(progress, 0.0)), 100)
            feedback_msg.current_cleaned_points = int(self.cleaned_points)
            feedback_msg.current_x = self.pose.x
            feedback_msg.current_y = self.pose.y
            goal_handle.publish_feedback(feedback_msg)
                    
            self.publisher_.publish(twist)
        
    def execute_callback(self, goal_handle):
        rclpy.spin_once(self, timeout_sec=0.01)
        task_type = goal_handle.request.task_type
        self.cleaned_points = 0
        
        feedback_msg = CleaningTask.Feedback()
        feedback_msg.current_cleaned_points = self.cleaned_points
        
        if task_type == "clean_square":
            start_size = size = goal_handle.request.area_size
            final_size = 1
            step = 0.2
            total_sides = math.floor((start_size - final_size) / step)
            while size > final_size:
                for i in range(4):
                    self.move_forward(goal_handle=goal_handle, speed=1.5, lenght=size)
                    self.rotate(goal_handle=goal_handle, angular_speed=1.5, angle=1.57)
                    size -= step
                progress = (start_size - size) / (start_size - final_size) * 100
                feedback_msg.progress_percent =  min(int(progress), 100)
                feedback_msg.current_cleaned_points = int(self.cleaned_points)
                feedback_msg.current_x = self.pose.x
                feedback_msg.current_y = self.pose.y
                goal_handle.publish_feedback(feedback_msg)          
        elif task_type == "return_home":
            goal_x = goal_handle.request.target_x
            goal_y = goal_handle.request.target_y
            self.move_to_goal(goal_handle=goal_handle, speed=1.5, goal_x=goal_x, goal_y=goal_y, goal_theta=0.0)            
        else:
            result = CleaningTask.Result()
            result.success = False
            return result
            
        goal_handle.succeed()
        result = CleaningTask.Result()
        result.success = True
        result.total_distance = float(self.cleaned_points)
        result.cleaned_points = int(self.cleaned_points)
        return result


def main(args=None):
    rclpy.init(args=args)
    cleaning_robot_action_server = CleaningRobotActionServer()
    rclpy.spin(cleaning_robot_action_server)


if __name__ == '__main__':
    main()