// protocol.hpp
#pragma once
#include "../include/uart_transporter.hpp"

// #include "rcia_vision_interfaces/msg/serial_receive_data.hpp"
#include <tuple>
#include <string>


namespace rcia::serial_driver{
namespace protocol {
struct UniversalFrame{
    // 通用帧定义
    static constexpr uint8_t HEADER = 0x39;
    static constexpr uint8_t END = 0xFF;
    static constexpr size_t HEADER_SIZE = 2;  // 双帧头
    static constexpr size_t DATA_SIZE = 16;   // 默认数据大小
};

struct SerialReceiveData {
    int32_t pitch_angle;
    int32_t yaw_angle;
    int bullet_speed;
    std::string our_color;
    uint8_t vision_mode;
    int32_t time_stamp;
};

struct SerialTransmitData {
    int32_t pitch_angle;
    int32_t yaw_angle;
    uint8_t find_flag;
    uint8_t fire_flag;
    uint32_t time_stamp;
    uint8_t operator_ui_x;
    uint8_t operator_ui_y;
};

class Protocol {
public:
    virtual ~Protocol() = default;

    explicit Protocol(rcia::serial_driver::UartTransporter& uart);

    // 接收解析
    virtual struct SerialReceiveData receive() = 0;

    // 发送打包
    virtual void transmit(struct SerialTransmitData* serial_send_data_ptr) = 0;

protected:
    rcia::serial_driver::UartTransporter& uart_;
    // // 数据打包私有方法
    // void pack_header(uint8_t* buffer);

    // void pack_data(uint8_t* buffer, int32_t pitch, int32_t yaw, uint8_t armor_flag, uint8_t fire_flag, uint32_t time_stamp, uint8_t ui_x, uint8_t ui_y);
    
    // struct SerialReceiveData parse_data(const uint8_t* data);
};
} // namespace protocol
} // namespace rcia::serial_driver
