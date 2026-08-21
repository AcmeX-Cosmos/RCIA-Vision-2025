// serial_driver_node.cpp
#include "serial_driver_node.hpp"

namespace rcia {
namespace serial_driver {

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions & options)
    : Node("serial_driver_node", options),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {

    // 参数声明
    declare_parameter<bool>("simulate_mode", false);
    declare_parameter<std::string>("device", "/dev/ttyACM0");

    declare_parameter<int>("baudrate", 115200);
    declare_parameter<int>("flow_control", 0);
    declare_parameter<int>("data_bits", 8);
    declare_parameter<int>("stop_bits", 1);
    declare_parameter<int>("parity", 0);

    RCLCPP_DEBUG(this->get_logger(), "init"); 
    simulate_mode_ = get_parameter("simulate_mode").as_bool();

    // 初始化串口
    if (!simulate_mode_) {
        init_uart();
    }

    // 创建定时器（1ms周期）
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&SerialDriverNode::read_serial_data, this));

    // 创建发布者
    electrl_pub_ = create_publisher<rcia_vision_interfaces::msg::SerialReceiveData>("electrl_data", 1);

    // 创建订阅者
    vision_sub_ = create_subscription<rcia_vision_interfaces::msg::SerialTransmitData>(
        "vision_data", 1, std::bind(&SerialDriverNode::write_serial_data, this, std::placeholders::_1));
}

void SerialDriverNode::init_uart() {
    if (simulate_mode_) {
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode, skip UART init");
        return;
    }

    serial_ = std::make_unique<rcia::serial_driver::UartTransporter>();
    
    // 定义多个串口路径
    std::vector<std::string> ports_list = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};
    
    // 尝试打开串口
    if (!serial_->openPorts(ports_list)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize any serial port");
        rclcpp::shutdown();
    }

    // 获取参数
    const auto baudrate = get_parameter("baudrate").as_int();
    const auto flow_control = get_parameter("flow_control").as_int();
    const auto data_bits = get_parameter("data_bits").as_int();
    const auto stop_bits = get_parameter("stop_bits").as_int();
    const auto parity = get_parameter("parity").as_int();

    // 配置串口参数
    if (!serial_->setParam(baudrate, flow_control, data_bits, stop_bits, parity)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to set serial port parameters");
        // rclcpp::shutdown();
    }

    // 初始化协议层
    protocol_ = std::make_unique<protocol::InfantryProtocol>(*serial_);
}


void SerialDriverNode::write_serial_data(const rcia_vision_interfaces::msg::SerialTransmitData::SharedPtr msg) {
    try {

        if (!simulate_mode_) {
            auto serial_transmit_data = rcia::serial_driver::protocol::SerialTransmitData();
            auto* serial_send_data_ptr = &serial_transmit_data;

            serial_send_data_ptr->pitch_angle = msg->pitch_angle;
            serial_send_data_ptr->yaw_angle = msg->yaw_angle;
            serial_send_data_ptr->find_flag = msg->find_flag;
            serial_send_data_ptr->fire_flag = msg->fire_flag;
            serial_send_data_ptr->time_stamp = msg->time_stamp;
            serial_send_data_ptr->operator_ui_x = 0;
            serial_send_data_ptr->operator_ui_y = 0;

            // 调用 protocol_->transmit() 方法
            protocol_->transmit(serial_send_data_ptr);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transmit data: %s", e.what());
    }
}

void SerialDriverNode::read_serial_data() {
    try {
        // 此处可优化 将protocol_->receive()改成msg
        // 接着直接转发, 省去赋值操作
        rcia::serial_driver::protocol::SerialReceiveData serial_data;
        if (simulate_mode_) {
            // 模拟数据
            serial_data.pitch_angle = 100.0;
            serial_data.yaw_angle = 100.0;
            serial_data.bullet_speed = 23.0;
            serial_data.our_color = "red";
            serial_data.vision_mode = 0;
            serial_data.time_stamp = 0;
        } else {
            // 从协议层接收数据
            serial_data = protocol_->receive();
        }
        
        // 填充消息对象
        serial_msg_.pitch_angle = serial_data.pitch_angle;
        serial_msg_.yaw_angle = serial_data.yaw_angle;
        serial_msg_.bullet_speed = serial_data.bullet_speed;
        serial_msg_.our_color = serial_data.our_color;
        serial_msg_.vision_mode = serial_data.vision_mode;
        serial_msg_.time_stamp = serial_data.time_stamp;

        // 发布消息
        publish_data(serial_msg_);

        // 广播TF
        auto pitch_rad = serial_data.pitch_angle / 100.0 * M_PI / 180.0;
        auto yaw_rad = serial_data.yaw_angle / 100.0 * M_PI / 180.0;
        broadcast_tf(pitch_rad, yaw_rad);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
    }
}

void SerialDriverNode::publish_data(const rcia_vision_interfaces::msg::SerialReceiveData& data) {
    auto serial_msg_ = std::make_unique<rcia_vision_interfaces::msg::SerialReceiveData>(data);
    electrl_pub_->publish(std::move(serial_msg_));
    RCLCPP_DEBUG(this->get_logger(), "Published data"); 
}

void SerialDriverNode::broadcast_tf(double pitch_rad, double yaw_rad) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "gimbal_link";

    tf2::Quaternion q;
    q.setRPY(0, pitch_rad, yaw_rad); // RPY顺序为roll, pitch, yaw
    t.transform.rotation = tf2::toMsg(q);
    // t.transform.translation.z = 0.35;
    
    tf_broadcaster_->sendTransform(t);
}

} // namespace serial_driver
} // namespace rcia

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rcia::serial_driver::SerialDriverNode)