#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class SinusoidMovement : public rclcpp::Node
{
public:
  SinusoidMovement() : Node("sinusoid_movement"), time_(0.0)
  {
    this->declare_parameter("linear_speed", 0.4);
    this->declare_parameter("angular_amplitude", 1.0);
    this->declare_parameter("frequency", 0.3);
    this->declare_parameter("update_rate_hz", 50.0);

    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_amplitude_ = this->get_parameter("angular_amplitude").as_double();
    frequency_ = this->get_parameter("frequency").as_double();
    double update_rate = this->get_parameter("update_rate_hz").as_double();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate),
      std::bind(&SinusoidMovement::publish_twist, this)
    );

    RCLCPP_INFO(this->get_logger(), "Sinusoid movement started. Robot will move in a sine wave pattern.");
  }

private:
  void publish_twist()
  {
    auto msg = geometry_msgs::msg::Twist();

    msg.linear.x = linear_speed_;

    // A * sin(2π * f * t)
    double angular_z = angular_amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_);

    msg.angular.z = angular_z;

    publisher_->publish(msg);

    double dt = 1.0 / this->get_parameter("update_rate_hz").as_double();
    time_ += dt;
  }

  double linear_speed_;
  double angular_amplitude_;
  double frequency_;
  double time_; 

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SinusoidMovement>());
  rclcpp::shutdown();
  return 0;
}