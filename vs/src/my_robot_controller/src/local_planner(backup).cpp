/*
 * Project: ROS 2 Autonomous Robot - Path Following Fix
 * Author: Srikar Reddy (Enhanced)
 * Date: February 2026
 * License: Apache 2.0
 * 
 * FIXES:
 * - Robot now moves forward while turning (no more rotate-in-place)
 * - Better lookahead logic
 * - Minimum forward speed during navigation
 * - Improved goal detection
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <algorithm>

class OptimizedLocalPlanner : public rclcpp::Node {
public:
    OptimizedLocalPlanner() : Node("local_planner") {
        // Enhanced Parameters
        this->declare_parameter("lookahead_dist", 0.8);        // Lookahead distance
        this->declare_parameter("min_lookahead", 0.5);         // Min lookahead
        this->declare_parameter("max_lookahead", 1.5);         // Max lookahead
        this->declare_parameter("obstacle_weight", 0.3);       
        this->declare_parameter("max_speed", 0.5);             
        this->declare_parameter("min_speed", 0.15);            // NEW: Minimum forward speed
        this->declare_parameter("max_angular_vel", 1.5);       
        this->declare_parameter("max_linear_accel", 0.4);      
        this->declare_parameter("max_angular_accel", 1.0);     
        this->declare_parameter("goal_tolerance", 0.2);        // Increased tolerance
        this->declare_parameter("rotation_threshold", 0.5);    // NEW: Max angle error for forward motion

        // Subscribers
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, 
            std::bind(&OptimizedLocalPlanner::path_callback, this, std::placeholders::_1));
        
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, 
            std::bind(&OptimizedLocalPlanner::lidar_callback, this, std::placeholders::_1));

        // Publisher
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_drive", 10);

        // TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Control loop at 20Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&OptimizedLocalPlanner::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "✅ Optimized Local Planner Ready (Path Following Mode)");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }
        
        current_path_ = *msg;
        current_goal_index_ = 0;
        path_received_ = true;
        goal_reached_logged_ = false;
        
        RCLCPP_INFO(this->get_logger(), "📍 New path: %lu waypoints, distance: %.2fm", 
            msg->poses.size(),
            calculate_path_length(msg->poses));
    }

    double calculate_path_length(const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
        double length = 0.0;
        for (size_t i = 1; i < poses.size(); i++) {
            double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
            double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
            length += std::hypot(dx, dy);
        }
        return length;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::fromROSMsg(*msg, latest_scan_);
    }

    void control_loop() {
        if (!path_received_ || current_path_.poses.empty()) {
            publish_velocity(0.0, 0.0);
            return;
        }

        // 1. Get Robot Pose
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) { 
            return; 
        }

        double rx = t.transform.translation.x;
        double ry = t.transform.translation.y;
        
        tf2::Quaternion q(
            t.transform.rotation.x, t.transform.rotation.y, 
            t.transform.rotation.z, t.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 2. Find Lookahead Point
        double lookahead = this->get_parameter("lookahead_dist").as_double();
        double gx, gy;
        bool end_reached = get_lookahead_point(rx, ry, lookahead, gx, gy);

        if (end_reached) {
            // Smooth stop at goal
            publish_velocity(0.0, 0.0);
            if (!goal_reached_logged_) {
                RCLCPP_INFO(this->get_logger(), "🎯 Goal Reached!");
                goal_reached_logged_ = true;
            }
            path_received_ = false;
            return;
        }

        // 3. Calculate desired heading to lookahead point
        double dx = gx - rx;
        double dy = gy - ry;
        double desired_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(desired_yaw - yaw);
        
        // Distance to lookahead point
        double distance_to_goal = std::hypot(dx, dy);

        // 4. Obstacle avoidance (optional - can reduce if causing issues)
        double rep_x = 0.0;
        double rep_y = 0.0;
        double obs_weight = this->get_parameter("obstacle_weight").as_double();

        if (!latest_scan_.empty() && obs_weight > 0.01) {
            calculate_repulsive_force(rep_x, rep_y, yaw);
            
            // Adjust desired heading with repulsion
            double total_x = dx + (rep_x * obs_weight);
            double total_y = dy + (rep_y * obs_weight);
            desired_yaw = std::atan2(total_y, total_x);
            yaw_error = normalize_angle(desired_yaw - yaw);
        }

        // 5. Velocity Control - KEEP MOVING FORWARD
        double max_speed = this->get_parameter("max_speed").as_double();
        double min_speed = this->get_parameter("min_speed").as_double();
        double max_angular = this->get_parameter("max_angular_vel").as_double();
        double rotation_threshold = this->get_parameter("rotation_threshold").as_double();
        
        // Angular velocity - proportional to error
        double angular_vel = std::clamp(2.0 * yaw_error, -max_angular, max_angular);
        
        // Linear velocity - ALWAYS MOVE FORWARD (unless turning sharply)
        double linear_vel;
        
        // ... inside control_loop ...
        
        if (std::abs(yaw_error) > rotation_threshold) {
            // FIX: Increase speed during turns to overcome friction
            linear_vel = min_speed * 1.5; // Was 0.5 (too slow)
        } else {
            // Normal operation
            double alignment_factor = 1.0 - (std::abs(yaw_error) / M_PI);
            linear_vel = min_speed + (max_speed - min_speed) * alignment_factor;
            
            if (distance_to_goal < 1.0) {
                linear_vel *= distance_to_goal;
            }
        }
        // Ensure minimum forward motion (prevents getting stuck rotating)
        if (linear_vel < min_speed * 0.3) {
            linear_vel = min_speed * 0.3;
        }

        // 6. Apply Acceleration Limits
        linear_vel = apply_acceleration_limit(
            last_linear_vel_, linear_vel, 
            this->get_parameter("max_linear_accel").as_double()
        );
        
        angular_vel = apply_acceleration_limit(
            last_angular_vel_, angular_vel,
            this->get_parameter("max_angular_accel").as_double()
        );

        // 7. Debug output
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Pos: (%.2f, %.2f) → Goal: (%.2f, %.2f) | Dist: %.2fm | Angle: %.1f° | Vel: %.2f m/s",
            rx, ry, gx, gy, distance_to_goal, yaw_error * 180.0 / M_PI, linear_vel);

        // 8. Publish
        publish_velocity(linear_vel, angular_vel);
    }

    void calculate_repulsive_force(double& rep_x, double& rep_y, double robot_yaw) {
        int obstacle_count = 0;
        double total_rep_x = 0.0;
        double total_rep_y = 0.0;
        
        for (const auto& pt : latest_scan_.points) {
            // Height filter
            if (pt.z < 0.05 || pt.z > 1.2) continue;
            
            double dist = std::hypot(pt.x, pt.y);
            
            // Self filter
            if (dist < 0.35) continue;
            
            // Only repel from nearby obstacles
            if (dist < 1.0) {
                double force = std::pow(1.0 / dist, 2) * 0.3;
                
                double local_rep_x = -pt.x / dist;
                double local_rep_y = -pt.y / dist;

                // Transform to global frame
                total_rep_x += (local_rep_x * cos(robot_yaw) - local_rep_y * sin(robot_yaw)) * force;
                total_rep_y += (local_rep_x * sin(robot_yaw) + local_rep_y * cos(robot_yaw)) * force;
                obstacle_count++;
            }
        }

        if (obstacle_count > 0) {
            rep_x = total_rep_x / obstacle_count;
            rep_y = total_rep_y / obstacle_count;
        }
    }

    bool get_lookahead_point(double rx, double ry, double dist, double& gx, double& gy) {
        double goal_tolerance = this->get_parameter("goal_tolerance").as_double();
        
        // Find point closest to lookahead distance
        int best_index = current_goal_index_;
        double best_diff = 1000.0;
        
        for (size_t i = current_goal_index_; i < current_path_.poses.size(); i++) {
            double px = current_path_.poses[i].pose.position.x;
            double py = current_path_.poses[i].pose.position.y;
            double distance = std::hypot(px - rx, py - ry);
            
            // Check if this is the final goal and we're close enough
            if (i == current_path_.poses.size() - 1 && distance < goal_tolerance) {
                return true;  // Goal reached
            }
            
            // Find point closest to desired lookahead
            double diff = std::abs(distance - dist);
            if (diff < best_diff) {
                best_diff = diff;
                best_index = i;
            }
            
            // Don't look too far ahead
            if (distance > dist * 2.0) break;
        }
        
        // Update current goal index
        current_goal_index_ = best_index;
        
        // Return the lookahead point
        gx = current_path_.poses[best_index].pose.position.x;
        gy = current_path_.poses[best_index].pose.position.y;
        
        return false;  // Not at goal yet
    }

    double apply_acceleration_limit(double current_vel, double target_vel, double max_accel) {
        double dt = 0.05;
        double max_delta = max_accel * dt;
        double delta = target_vel - current_vel;
        
        if (std::abs(delta) > max_delta) {
            return current_vel + std::copysign(max_delta, delta);
        }
        return target_vel;
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void publish_velocity(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;
        pub_vel_->publish(cmd);
        
        last_linear_vel_ = linear;
        last_angular_vel_ = angular;
    }

    // Members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    nav_msgs::msg::Path current_path_;
    pcl::PointCloud<pcl::PointXYZ> latest_scan_;
    size_t current_goal_index_ = 0;
    bool path_received_ = false;
    bool goal_reached_logged_ = false;
    
    double last_linear_vel_ = 0.0;
    double last_angular_vel_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptimizedLocalPlanner>());
    rclcpp::shutdown();
    return 0;
}
