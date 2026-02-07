/*
 * Project: ROS 2 Autonomous  Robot
 * Author: Srikar Reddy
 * Date: February 2026
 * License: Apache 2.0
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CameraSafety : public rclcpp::Node {
public:
    CameraSafety() : Node("camera_safety") {
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, 
            std::bind(&CameraSafety::depth_callback, this, std::placeholders::_1));

        pub_safety_ = this->create_publisher<std_msgs::msg::Bool>("/safety/camera", 10);

        RCLCPP_INFO(this->get_logger(), "👁️ Camera Safety Monitor Active.");
    }

private:
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) { return; }

        cv::Mat depth_img = cv_ptr->image;
        float min_dist = 10.0;
        
        // Check center area (200x100)
        for (int y = 190; y < 290; y++) {
            for (int x = 220; x < 420; x++) {
                float dist = depth_img.at<float>(y, x);
                if (dist > 0.1 && dist < min_dist) min_dist = dist;
            }
        }

        std_msgs::msg::Bool safety_msg;
        safety_msg.data = (min_dist < 0.7); // True if danger
        pub_safety_->publish(safety_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_safety_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSafety>());
    rclcpp::shutdown();
    return 0;
}
