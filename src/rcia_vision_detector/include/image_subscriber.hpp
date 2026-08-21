#ifndef IMAGE_SUBSCRIBER_HPP
#define IMAGE_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

#include "Types.hpp"
#include "debug.hpp"

// ROS2 msg
#include "rcia_vision_interfaces/msg/serial_receive_data.hpp"
#include "rcia_vision_interfaces/msg/serial_transmit_data.hpp"

namespace rcia::vision_identify
{
    class ArmorIdentifier;
}

class ImageSubscriber : public rclcpp::Node
{
public:
    explicit ImageSubscriber(const rclcpp::NodeOptions &options);

    ~ImageSubscriber() {
        cv::destroyWindow("img");
    }

protected:

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img);

    void init_identifier();

private:
    // ROS2 Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<rcia_vision_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;
    // ROS2 Publisher
    rclcpp::Publisher<rcia_vision_interfaces::msg::SerialTransmitData>::SharedPtr vision_pub_;

    // ROS2
    // std::string odom_frame_;
    // Eigen::Matrix3d imu_to_camera_;

    // time
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    int frame_count_;
    double fps_;

    // 原子指针指向当前读取的缓冲区
    std::atomic<rcia::SerialProtocol::SerialDataStruct *> current_read_buffer_;
    rcia::SerialProtocol::SerialDataStruct buffer1_, buffer2_; // 双缓冲实例

    // armor_identifier
    std::unique_ptr<rcia::vision_identify::ArmorIdentifier> armor_identifier_;
    bool is_init_identifier = false;

};


#endif // IMAGE_SUBSCRIBER_HPP