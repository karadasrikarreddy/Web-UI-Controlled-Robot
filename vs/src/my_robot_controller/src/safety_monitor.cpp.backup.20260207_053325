/*
 * Project: ROS 2 Autonomous Robot
 * Author: Srikar Reddy
 * Date: February 2026
 * License: Apache 2.0
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

class SafetyMonitor : public rclcpp::Node {
public:
    SafetyMonitor() : Node("safety_monitor") {
        // 1. SAFETY INPUTS
        sub_lidar_ = this->create_subscription<std_msgs::msg::Bool>(
            "/safety/lidar", 10, std::bind(&SafetyMonitor::lidar_callback, this, std::placeholders::_1));

        sub_camera_ = this->create_subscription<std_msgs::msg::Bool>(
            "/safety/camera", 10, std::bind(&SafetyMonitor::camera_callback, this, std::placeholders::_1));

        // 2. DRIVE INPUT (From Local Planner)
        sub_drive_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_drive", 10, std::bind(&SafetyMonitor::drive_callback, this, std::placeholders::_1));

        // 3. FINAL OUTPUT (To Robot)
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "🛡️ SAFETY MONITOR READY. Gatekeeping /cmd_vel");
    }

private:
    void lidar_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        lidar_danger_ = msg->data;
        decide_and_publish();
    }

    void camera_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        camera_danger_ = msg->data;
        decide_and_publish();
    }

    void drive_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
        decide_and_publish();
    }

    void decide_and_publish() {
        geometry_msgs::msg::Twist final_cmd;

        if (lidar_danger_ || camera_danger_) {
            // EMERGENCY STOP
            final_cmd.linear.x = 0.0;
            final_cmd.angular.z = 0.0;
        } else {
            // ALL CLEAR - Pass through command
            final_cmd = latest_cmd_;
        }
        pub_cmd_->publish(final_cmd);
    }

    bool lidar_danger_ = false;
    bool camera_danger_ = false;
    geometry_msgs::msg::Twist latest_cmd_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lidar_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_camera_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_drive_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}
