/*
 * Project: ROS 2 Autonomous  Robot
 * Author: Srikar Reddy
 * Date: February 2026
 * License: Apache 2.0
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>

// Constants
const double LIN_VEL_STEP_SIZE = 0.05;
const double ANG_VEL_STEP_SIZE = 0.1;
const double MAX_LIN_VEL = 2.0;
const double MAX_ANG_VEL = 1.57;

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("custom_teleop") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        target_linear_ = 0.0;
        target_angular_ = 0.0;
        current_linear_ = 0.0;
        current_angular_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "--- Advanced Teleop Ready ---");
        RCLCPP_INFO(this->get_logger(), "W/X: Inc/Dec Linear | A/D: Inc/Dec Angular | S/Space: Stop");
    }

    // Capture raw keyboard input
    char get_key() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    // Smoothly ramp velocity
    double ramp_velocity(double current, double target, double step) {
        if (target > current) {
            return std::min(target, current + step);
        } else if (target < current) {
            return std::max(target, current - step);
        }
        return target;
    }

    void run() {
        while (rclcpp::ok()) {
            char c = get_key();

            if (c == 'w') {
                target_linear_ = std::min(MAX_LIN_VEL, target_linear_ + LIN_VEL_STEP_SIZE);
            } else if (c == 'x') {
                target_linear_ = std::max(-MAX_LIN_VEL, target_linear_ - LIN_VEL_STEP_SIZE);
            } else if (c == 'a') {
                target_angular_ = std::min(MAX_ANG_VEL, target_angular_ + ANG_VEL_STEP_SIZE);
            } else if (c == 'd') {
                target_angular_ = std::max(-MAX_ANG_VEL, target_angular_ - ANG_VEL_STEP_SIZE);
            } else if (c == ' ' || c == 's') {
                target_linear_ = 0.0;
                target_angular_ = 0.0;
            } else if (c == '\033') { // ESC
                break;
            }

            // Apply smooth ramping
            current_linear_ = ramp_velocity(current_linear_, target_linear_, LIN_VEL_STEP_SIZE / 2.0);
            current_angular_ = ramp_velocity(current_angular_, target_angular_, ANG_VEL_STEP_SIZE / 2.0);

            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = target_linear_ ;      //current_linear_;
            msg.angular.z = target_angular_ ;    //current_angular_;
            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Linear: %.2f | Angular: %.2f", current_linear_, current_angular_);
        }
        
        // Stop robot on exit
        auto stop_msg = geometry_msgs::msg::Twist();
        publisher_->publish(stop_msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double target_linear_, target_angular_;
    double current_linear_, current_angular_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
