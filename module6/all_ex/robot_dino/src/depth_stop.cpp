#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>

class DepthStop : public rclcpp::Node {
public:
    DepthStop() : Node("depth_stop") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/image",
            qos,
            std::bind(&DepthStop::image_callback, this, std::placeholders::_1)
        );
        
        start_time_ = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(this->get_logger(), "Waiting 3 seconds...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_log_time_;
    const double LOG_THROTTLE_DURATION = 1.0;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double>(current_time - start_time_).count();
        
        if (elapsed < 3.0) {
            auto throttle_elapsed = std::chrono::duration<double>(current_time - last_log_time_).count();
            if (throttle_elapsed >= 1.0) {
                RCLCPP_INFO(this->get_logger(), "Wait: %.1f s", 3.0 - elapsed);
                last_log_time_ = current_time;
            }
            return;
        }

        auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
        float min_distance = 10.0f;

        try {
            if (msg->encoding == "32FC1") {
                auto* data = reinterpret_cast<const float*>(msg->data.data());
                int total_pixels = msg->width * msg->height;
                
                int center_start = (msg->height / 2 - 10) * msg->width + (msg->width / 2 - 10);
                int center_end = center_start + 20 * msg->width;
                
                min_distance = 10.0f;
                for (int i = center_start; i < center_end && i < total_pixels; i += msg->width) {
                    for (int j = 0; j < 20; j++) {
                        if (i + j < total_pixels) {
                            float depth = data[i + j];
                            if (!std::isnan(depth) && !std::isinf(depth) && depth > 0) {
                                min_distance = std::min(min_distance, depth);
                            }
                        }
                    }
                }

                const float STOP_DIST = 0.5f;
                if (min_distance < STOP_DIST) {
                    cmd->linear.x = 0.0;
                    RCLCPP_WARN(this->get_logger(), "Obstacle at %.2f m", min_distance);
                } else {
                    cmd->linear.x = 0.3;
                    auto throttle_elapsed = std::chrono::duration<double>(current_time - last_log_time_).count();
                    if (throttle_elapsed >= LOG_THROTTLE_DURATION) {
                        RCLCPP_INFO(this->get_logger(), "Go. Depth: %.2f m", min_distance);
                        last_log_time_ = current_time;
                    }
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Wrong encoding: %s", msg->encoding.c_str());
                cmd->linear.x = 0.0;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
            cmd->linear.x = 0.0;
        }

        cmd_vel_pub_->publish(*cmd);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}