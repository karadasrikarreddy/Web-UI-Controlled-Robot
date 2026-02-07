/*
 * Project: ROS 2 Autonomous Robot
 * Author: Srikar Reddy
 * Date: February 2026
 * License: Apache 2.0
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class MapLoaderNode : public rclcpp::Node {
public:
    MapLoaderNode() : Node("map_loader_node") {
        // Declare parameters for dynamic paths
        this->declare_parameter<std::string>("pcd_path", "");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<double>("publish_rate", 1.0);

        std::string pcd_path = this->get_parameter("pcd_path").as_string();
        std::string frame_id = this->get_parameter("frame_id").as_string();
        double rate = this->get_parameter("publish_rate").as_double();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/static_map_cloud", 10);

        // Load the PCD file
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to load map at: %s", pcd_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded 3D map with %lu points.", cloud->size());

        // Convert to ROS Message once
        pcl::toROSMsg(*cloud, map_msg_);
        map_msg_.header.frame_id = frame_id;

        // Timer to keep the map "alive" in the network
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&MapLoaderNode::publish_map, this));
    }

private:
    void publish_map() {
        map_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(map_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2 map_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLoaderNode>());
    rclcpp::shutdown();
    return 0;
}
