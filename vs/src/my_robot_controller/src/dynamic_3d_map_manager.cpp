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
#include <pcl/filters/passthrough.h>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class Dynamic3DMapManager : public rclcpp::Node {
public:
    Dynamic3DMapManager() : Node("dynamic_3d_map_manager") {
        // Declare the parameter
        this->declare_parameter<std::string>("pcd_file_path", "");

        // QoS: Transient Local is still good practice
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        qos_profile.reliable();
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map_3d", qos_profile);

        // Service to force reload
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "load_map", std::bind(&Dynamic3DMapManager::load_map_service, this, std::placeholders::_1, std::placeholders::_2));

        // TIMER: Run this logic every 5 seconds
        // This fixes the "Startup Race Condition" and the "Disappearing Map" issue.
        timer_ = this->create_wall_timer(
            5s, std::bind(&Dynamic3DMapManager::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "⏳ Map Manager Started. Checking for map every 5s...");
    }

private:
    void timer_callback() {
        // If we already have a map loaded, just re-publish it!
        // This ensures RViz always finds it.
        if (!map_msg_.data.empty()) {
            map_msg_.header.stamp = this->get_clock()->now();
            publisher_->publish(map_msg_);
            return;
        }

        // If no map yet, try to load it
        load_map_from_file();
    }

    bool load_map_from_file() {
        std::string pcd_path;
        this->get_parameter("pcd_file_path", pcd_path);

        if (pcd_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Waiting for 'pcd_file_path' parameter...");
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "❌ FAILED to open file: %s", pcd_path.c_str());
            return false;
        }

        // --- FILTERING (Ground Removal) ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.05, 5.0); 
        pass.filter(*cloud_filtered);
        // ----------------------------------

        // Save to member variable so we can re-publish later
        pcl::toROSMsg(*cloud_filtered, map_msg_);
        map_msg_.header.frame_id = "map";
        
       // RCLCPP_INFO(this->get_logger(), "✅ Map Loaded Successfully! (%lu points)", cloud_filtered->size());
        return true;
    }

    // Service just clears the current map forcing the timer to reload it next cycle
    void load_map_service(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        // Force reload by clearing current data
        map_msg_.data.clear(); 
        
        // Try immediately (optional, or just wait for timer)
        if (load_map_from_file()) {
            response->success = true;
            response->message = "Map reloaded.";
        } else {
            response->success = false;
            response->message = "Failed to load.";
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2 map_msg_; // Store map in memory
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dynamic3DMapManager>());
    rclcpp::shutdown();
    return 0;
}
