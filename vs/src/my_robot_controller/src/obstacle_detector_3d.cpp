/*
 * Project: ROS 2 Autonomous Robot
 * Author: Srikar Reddy
 * Date: February 2026
 * License: Apache 2.0
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/bool.hpp> // Publishing Safety State
#include <atomic>

class ObstacleDetector3D : public rclcpp::Node {
public:
    ObstacleDetector3D() : Node("obstacle_detector_3d") {
        // --- PARAMS ---
        this->declare_parameter("robot_width_half", 0.30); 
        this->declare_parameter("robot_length_front", 0.35);
        this->declare_parameter("stop_distance_buffer", 0.5); // Static buffer for now

        // Subscriber: Lidar
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, std::bind(&ObstacleDetector3D::lidar_callback, this, std::placeholders::_1));

        // Publisher: Safety State (True = Danger, False = Safe)
        pub_safety_ = this->create_publisher<std_msgs::msg::Bool>("/safety/lidar", 10);

        // Timer: Check safety 10 times a second
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ObstacleDetector3D::check_safety_loop, this));

        RCLCPP_INFO(this->get_logger(), "🛡️ LIDAR MONITOR ACTIVE. Publishing to /safety/lidar");
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        double width_limit = this->get_parameter("robot_width_half").as_double();
        double front_base = this->get_parameter("robot_length_front").as_double();

        float min_x = 100.0;

        for (const auto& pt : cloud.points) {
            // 1. Z-Filter (Floor/Ceiling)
            if (pt.z < 0.05 || pt.z > 1.0) continue;

            // 2. Y-Filter (Corridor Width)
            if (std::abs(pt.y) > width_limit) continue;

            // 3. X-Filter (Ignore self)
            if (pt.x < front_base) continue;

            // 4. Find closest object in front
            if (pt.x < min_x) min_x = pt.x;
        }
        min_forward_dist_ = min_x;
    }

    void check_safety_loop() {
        double front_base = this->get_parameter("robot_length_front").as_double();
        double buffer = this->get_parameter("stop_distance_buffer").as_double();
        
        // If object is closer than (Front of robot + 0.5m buffer) -> DANGER
        double limit = front_base + buffer;
        float obs_dist = min_forward_dist_.load();

        std_msgs::msg::Bool safety_msg;
        if (obs_dist < limit) {
            safety_msg.data = true; // DANGER!
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "🛑 LIDAR TRIGGER! Obj: %.2fm", obs_dist);
        } else {
            safety_msg.data = false; // SAFE
        }
        
        pub_safety_->publish(safety_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_safety_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<float> min_forward_dist_{100.0f}; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector3D>());
    rclcpp::shutdown();
    return 0;
}
