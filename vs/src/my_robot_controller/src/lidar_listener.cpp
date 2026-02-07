/*
 * Project: ROS 2 Autonomous Robot
 * Author: Srikar Reddy
 * Date: February 2026
 * License: Apache 2.0
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSubscriber : public rclcpp::Node {
public:
    LidarSubscriber() : Node("lidar_subscriber") {
        // Subscribe to the "/scan" topic
        // We use 'sensor_msgs::msg::LaserScan' because that's what Lidars output
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
        // The Lidar returns an array (ranges). 
        // Index 0 is usually directly in front or behind depending on orientation.
        // Let's find the distance to the object directly in front (middle of the array).
        int middle_index = msg->ranges.size() / 2;
        float dist_front = msg->ranges[middle_index];

        RCLCPP_INFO(this->get_logger(), "Distance to obstacle in front: '%.2f' meters", dist_front);
        
        if (dist_front < 0.5) {
            RCLCPP_WARN(this->get_logger(), "TOO CLOSE! STOP THE ROBOT!");
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSubscriber>());
    rclcpp::shutdown();
    return 0;
}
