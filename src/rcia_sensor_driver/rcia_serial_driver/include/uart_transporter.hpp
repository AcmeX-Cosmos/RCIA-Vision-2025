#ifndef UART_TRANSPORTER_HPP
#define UART_TRANSPORTER_HPP

#include <string>
#include <vector>
#include <iostream>

// ROS2
#include "rclcpp/rclcpp.hpp"

// using namespace
using namespace std;

namespace rcia::serial_driver {
class UartTransporter {
public:
    explicit UartTransporter(const string &device_path = "/dev/ttyACM0",
                    int speed = 115200, // 波特率：115200
                    int flow_ctrl = 0, // 流控制：不使用
                    int databits = 8, // 数据位：8
                    int stopbits = 1, // 停止位：1
                    int parity = 'N') // 校验位：无校验
        : device_path_(device_path)
        , speed_(speed)
        , flow_ctrl_(flow_ctrl)
        , databits_(databits)
        , stopbits_(stopbits)
        , parity_(parity) {}

    bool setParam(
        int speed = 115200, int flow_ctrl = 0, int databits = 8, int stopbits = 1, int parity = 'N');
        // 波特率：115200        流控制：不使用           数据位：8           停止位：1     校验位：无校验


    bool open();

    void close();

    bool isOpen();

    int read(void *buffer, size_t len);

    int write(const void *buffer, size_t len);
    string errorMessage() { return error_message_;}

    // 尝试打开多个串口设备
    bool openPorts(const vector<string>& ports_list) {
        for (const auto& port : ports_list) {
            if (open(port)) {
                RCLCPP_INFO(rclcpp::get_logger("UartTransporter"), "Successfully opened port: %s", port.c_str());
                return true; // 成功打开一个串口后返回
            } else {
                RCLCPP_WARN(rclcpp::get_logger("UartTransporter"), "Failed to open port: %s", port.c_str());
            }
        }

        // 如果所有串口都打开失败，关闭当前的串口
        close();
        return false;
    }

    // 重载 open 方法以接受设备路径
    bool open(const string& device_path) {
        device_path_ = device_path;
        return open();
    }

private:
    // 设备文件描述符
    int fd_{-1};

    // 设备状态
    bool is_open_{false};
    string error_message_;

    // 设备路径
    string device_path_;
    int speed_;
    int flow_ctrl_;
    int databits_;
    int stopbits_;
    int parity_;
};

    void Transmit_Serial_value(UartTransporter &serial, int32_t pitch_target_angle1, int32_t yaw_target_angle1, uint8_t armor_flag1, uint8_t fire_flag1, uint32_t message_time_stamp, uint8_t operator_ui_armorx, uint8_t operator_ui_armory);

} // namespace rcia::serial_driver


#endif // UART_TRANSPORTER_HPP