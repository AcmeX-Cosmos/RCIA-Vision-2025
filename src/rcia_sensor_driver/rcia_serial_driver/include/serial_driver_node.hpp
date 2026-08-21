// serial_driver_node.hpp

#ifndef SERIAL_DRIVER_NODE_HPP
#define SERIAL_DRIVER_NODE_HPP

#include "../include/uart_transporter.hpp"
#include "../include/protocol.hpp"
#include "../include/protocol/infantry_protocol.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 msg
#include "rcia_vision_interfaces/msg/serial_receive_data.hpp"
#include "rcia_vision_interfaces/msg/serial_transmit_data.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace rcia {
namespace serial_driver {

class SerialDriverNode : public rclcpp::Node {
public:
    explicit SerialDriverNode(const rclcpp::NodeOptions & options);

private:
    void init_uart();
    void read_serial_data();
    void write_serial_data(const rcia_vision_interfaces::msg::SerialTransmitData::SharedPtr msg);
    void publish_data(const rcia_vision_interfaces::msg::SerialReceiveData& data);
    void broadcast_tf(double pitch_rad, double yaw_rad);

    bool simulate_mode_;

    std::unique_ptr<rcia::serial_driver::UartTransporter> serial_;
    std::unique_ptr<protocol::InfantryProtocol> protocol_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rcia_vision_interfaces::msg::SerialReceiveData>::SharedPtr electrl_pub_;
    rclcpp::Subscription<rcia_vision_interfaces::msg::SerialTransmitData>::SharedPtr vision_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rcia_vision_interfaces::msg::SerialReceiveData serial_msg_;
};

} // namespace serial_driver
} // namespace rcia


#endif // SERIAL_DRIVER_NODE_HPP