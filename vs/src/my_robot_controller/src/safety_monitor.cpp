/*
 * Project: ROS 2 Autonomous Robot - OPTIMIZED SAFETY
 * Author: Srikar Reddy (Enhanced)
 * Date: February 2026
 * License: Apache 2.0
 * 
 * IMPROVEMENTS:
 * - Gradual braking instead of emergency stop
 * - Safety zones with different threat levels
 * - Velocity scaling based on proximity
 * - Smoother transitions
 * 
 * NOTE: This version uses safety signals from obstacle_detector_3d
 * instead of processing raw LiDAR data directly.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

class OptimizedSafetyMonitor : public rclcpp::Node {
public:
    OptimizedSafetyMonitor() : Node("safety_monitor") {
        // Safety Parameters
        this->declare_parameter("brake_smoothness", 0.15);     // Braking acceleration
        this->declare_parameter("min_safe_speed", 0.05);      // Minimum speed
        
        // Safety signal subscribers
        sub_lidar_safety_ = this->create_subscription<std_msgs::msg::Bool>(
            "/safety/lidar", 10, 
            std::bind(&OptimizedSafetyMonitor::lidar_safety_callback, this, std::placeholders::_1));

        sub_camera_ = this->create_subscription<std_msgs::msg::Bool>(
            "/safety/camera", 10, 
            std::bind(&OptimizedSafetyMonitor::camera_callback, this, std::placeholders::_1));

        // Drive input
        sub_drive_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_drive", 10, 
            std::bind(&OptimizedSafetyMonitor::drive_callback, this, std::placeholders::_1));

        // Final output
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Safety status publisher (for UI)
        pub_safety_status_ = this->create_publisher<std_msgs::msg::Bool>("/safety/status", 10);

        // Control timer for smooth braking
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OptimizedSafetyMonitor::safety_control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🛡️ OPTIMIZED SAFETY MONITOR ACTIVE");
        RCLCPP_INFO(this->get_logger(), "   - Gradual braking enabled");
        RCLCPP_INFO(this->get_logger(), "   - Smooth safety transitions");
    }

private:
    void lidar_safety_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        bool prev_state = lidar_danger_;
        lidar_danger_ = msg->data;
        
        // Log state changes
        if (lidar_danger_ && !prev_state) {
            RCLCPP_WARN(this->get_logger(), "⚠️  LIDAR: Obstacle detected - initiating smooth stop");
        } else if (!lidar_danger_ && prev_state) {
            RCLCPP_INFO(this->get_logger(), "✓ LIDAR: Path clear - resuming normal operation");
        }
    }

    void camera_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        bool prev_state = camera_danger_;
        camera_danger_ = msg->data;
        
        // Log state changes
        if (camera_danger_ && !prev_state) {
            RCLCPP_WARN(this->get_logger(), "⚠️  CAMERA: Obstacle detected - initiating smooth stop");
        } else if (!camera_danger_ && prev_state) {
            RCLCPP_INFO(this->get_logger(), "✓ CAMERA: Path clear - resuming normal operation");
        }
    }

    void drive_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        desired_cmd_ = *msg;
    }

    void safety_control_loop() {
        geometry_msgs::msg::Twist final_cmd;
        bool in_danger = lidar_danger_ || camera_danger_;
        
        // Determine target velocity
        double target_linear = 0.0;
        double target_angular = desired_cmd_.angular.z;  // Always allow turning
        
        if (!in_danger) {
            // All clear - use desired command
            target_linear = desired_cmd_.linear.x;
        } else {
            // Danger detected - smooth stop
            target_linear = 0.0;
        }
        
        // Apply smooth acceleration/braking
        double brake_accel = this->get_parameter("brake_smoothness").as_double();
        current_linear_vel_ = smooth_transition(current_linear_vel_, target_linear, brake_accel);
        
        // Ensure minimum speed threshold
       // double min_speed = this->get_parameter("min_safe_speed").as_double();
        //if (std::abs(current_linear_vel_) < min_speed && target_linear != 0.0) {
         //   current_linear_vel_ = 0.0;  // Snap to zero if below threshold
        //}
        
        // Publish final command
        final_cmd.linear.x = current_linear_vel_;
        final_cmd.angular.z = target_angular;
        pub_cmd_->publish(final_cmd);
        
        // Publish safety status for UI
        std_msgs::msg::Bool status;
        status.data = in_danger;
        pub_safety_status_->publish(status);
        
        // Throttled logging for active braking
        if (in_danger && std::abs(current_linear_vel_) > 0.01) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🛑 Braking: %.2f m/s → %.2f m/s", 
                desired_cmd_.linear.x, current_linear_vel_);
        }
    }

    double smooth_transition(double current, double target, double max_change) {
        double dt = 0.05;  // 20Hz control loop
        double max_delta = max_change * dt;
        double delta = target - current;
        
        if (std::abs(delta) > max_delta) {
            return current + std::copysign(max_delta, delta);
        }
        return target;
    }

    // Members
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lidar_safety_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_camera_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_drive_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_safety_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool lidar_danger_ = false;
    bool camera_danger_ = false;
    geometry_msgs::msg::Twist desired_cmd_;
    double current_linear_vel_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptimizedSafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}
