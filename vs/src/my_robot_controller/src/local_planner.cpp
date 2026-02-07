/*
 * Project: ROS 2 Autonomous Robot
 * Author: Srikar Reddy
 * Date: February 2026
 * License: Apache 2.0
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

class LocalPlanner : public rclcpp::Node {
public:
    LocalPlanner() : Node("local_planner") {
        // Tunable Parameters
        this->declare_parameter("lookahead_dist", 0.7);  // Look 1 meter ahead on path
        this->declare_parameter("obstacle_weight", 1.0); // How much to fear obstacles
        this->declare_parameter("max_speed", 0.5);

        // Subscribers
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&LocalPlanner::path_callback, this, std::placeholders::_1));
        
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1));

        // Publisher (To Raw, so Safety Node still protects us)
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_drive", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&LocalPlanner::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "✅ Potential Field Local Planner Ready.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = *msg;
        current_goal_index_ = 0;
        path_received_ = true;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::fromROSMsg(*msg, latest_scan_);
    }

    void control_loop() {
        if (!path_received_ || current_path_.poses.empty()) return;

        // 1. Get Robot Pose
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) { return; }

        double rx = t.transform.translation.x;
        double ry = t.transform.translation.y;
        
        tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 2. Calculate ATTRACTIVE Force (Towards Path)
        double lookahead = this->get_parameter("lookahead_dist").as_double();
        double gx, gy;
        bool end_reached = get_lookahead_point(rx, ry, lookahead, gx, gy);

        if (end_reached) {
            geometry_msgs::msg::Twist stop;
            pub_vel_->publish(stop);
            if (path_received_) RCLCPP_INFO(this->get_logger(), "🎉 Goal Reached!");
            path_received_ = false;
            return;
        }

        double att_x = gx - rx;
        double att_y = gy - ry;

        // 3. Calculate REPULSIVE Force (Away from Obstacles)
        double rep_x = 0.0;
        double rep_y = 0.0;
        double obs_weight = this->get_parameter("obstacle_weight").as_double();

        if (!latest_scan_.empty()) {
            for (const auto& pt : latest_scan_.points) {
                if (pt.z < 0.05 || pt.z > 1.0) continue; // Floor/Ceiling filter
                
                double dist = std::hypot(pt.x, pt.y);
                if (dist < 0.4) continue; // Self filter
                
                // Repel only if close (within 1.0m)
                if (dist < 1.0) {
                    double force = (1.0 / dist) - (1.0 / 1.0); 
                    double local_rep_x = -pt.x / dist; 
                    double local_rep_y = -pt.y / dist;

                    // Rotate to Global Frame and add to repulsion accumulator
                    rep_x += (local_rep_x * cos(yaw) - local_rep_y * sin(yaw)) * force;
                    rep_y += (local_rep_x * sin(yaw) + local_rep_y * cos(yaw)) * force;
                }
            }
        }

        // 4. Combine Forces
        double total_x = att_x + (rep_x * obs_weight);
        double total_y = att_y + (rep_y * obs_weight);

        // 5. Steering Control (Pure Pursuit on the Resultant Vector)
        double desired_yaw = std::atan2(total_y, total_x);
        double yaw_error = desired_yaw - yaw;

        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;
        
        // --- SMOOTHING FIX 1: LOWER GAIN ---
        // Reduced from 2.0 to 1.0 to stop the "twitching"
        cmd.angular.z = 1.0 * yaw_error;

        double max_speed = this->get_parameter("max_speed").as_double();
        
        // --- SMOOTHING FIX 2: PROPORTIONAL SPEED ---
        // Instead of hard if/else brake, we scale speed linearly.
        // Sharp turn (1.0 rad error) -> Speed drops to ~0.0 (stopped turn)
        // Straight (0.0 rad error) -> Speed is max
        double speed_factor = 1.0 - std::abs(yaw_error);
        if (speed_factor < 0.1) speed_factor = 0.1; // Never stall completely
        
        cmd.linear.x = max_speed * speed_factor;

        pub_vel_->publish(cmd);
    }
    bool get_lookahead_point(double rx, double ry, double dist, double& gx, double& gy) {
        for (; current_goal_index_ < current_path_.poses.size(); current_goal_index_++) {
            gx = current_path_.poses[current_goal_index_].pose.position.x;
            gy = current_path_.poses[current_goal_index_].pose.position.y;
            if (std::hypot(gx - rx, gy - ry) > dist) return false;
        }
        return true; // End of path
    }

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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlanner>());
    rclcpp::shutdown();
    return 0;
}
