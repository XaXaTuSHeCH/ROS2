#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CircleMovement : public rclcpp::Node
{
public:
  CircleMovement() : Node("circle_movement")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CircleMovement::publish_twist, this)
    );
  }

private:
  void publish_twist()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.5; 
    msg.angular.z = 0.5; 
    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleMovement>());
  rclcpp::shutdown();
  return 0;
}
