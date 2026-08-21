#ifndef DEBUG_HPP
#define DEBUG_HPP

// Standard
#include <chrono>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rcia_vision_interfaces/msg/heart_beat.hpp" 
#include "rcia_vision_interfaces/msg/serial_transmit_data.hpp"

#include "Type.hpp"

using namespace chrono;


class HeartBeatPublisher{
public:

    explicit HeartBeatPublisher(std::weak_ptr<rclcpp::Node> node);
    
    ~HeartBeatPublisher() = default;

    void publish_heartbeat();

    void refresh(rcia_vision_interfaces::msg::SerialTransmitData vision_control_msg_) ;

private:

    int fps_count_ = 0;
    double fps = 0.0;
    double now_time_;
    double last_time_ = 0.0;
    
    std::weak_ptr<rclcpp::Node> node_;
    
    rclcpp::Publisher<rcia_vision_interfaces::msg::HeartBeat>::SharedPtr heartbeat_pub_;

};

#endif