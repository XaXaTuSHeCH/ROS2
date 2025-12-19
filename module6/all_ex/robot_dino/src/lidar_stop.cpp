#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>


class LidarStop : public rclcpp::Node {
public:
    LidarStop() : Node("lidar_stop") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot/scan",
            qos,
            std::bind(&LidarStop::scan_callback, this, std::placeholders::_1)
        );
        
        start_time_ = std::chrono::high_resolution_clock::now();
        last_log_time_ = start_time_;
        RCLCPP_INFO(this->get_logger(), "Waiting 3 seconds...");
    }


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_log_time_;
    const double LOG_THROTTLE_DURATION = 1.0;
    bool moving = false;


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double>(current_time - start_time_).count();
        
        if (elapsed < 3.0) {
            return;
        }

        auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
        float min_distance = std::numeric_limits<float>::max();

        try {
            for (size_t i = 0; i < msg->ranges.size(); i++) {
                float range = msg->ranges[i];
                
                if (range >= msg->range_min && range <= msg->range_max && 
                    !std::isnan(range) && !std::isinf(range) && range > 0) {
                    min_distance = std::min(min_distance, range);
                }
            }

            const float STOP_DIST = 0.5f;
            
            if (min_distance == std::numeric_limits<float>::max()) {
                cmd->linear.x = 0.0;
            } else if (min_distance < STOP_DIST) {
                cmd->linear.x = 0.0;
                if (moving) {
                    RCLCPP_WARN(this->get_logger(), "Obstacle at %.2f m", min_distance);
                    moving = false;
                }
            } else {
                cmd->linear.x = 0.3;
                if (!moving) {
                    auto throttle_elapsed = std::chrono::duration<double>(current_time - last_log_time_).count();
                    if (throttle_elapsed >= LOG_THROTTLE_DURATION) {
                        RCLCPP_INFO(this->get_logger(), "Moving");
                        last_log_time_ = current_time;
                        moving = true;
                    }
                }
            }

        } catch (const std::exception& e) {
            cmd->linear.x = 0.0;
        }

        cmd_vel_pub_->publish(*cmd);
    }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}