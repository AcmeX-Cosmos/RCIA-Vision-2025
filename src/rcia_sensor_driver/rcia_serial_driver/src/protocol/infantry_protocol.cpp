// infantry_protocol.cpp
#include "protocol/infantry_protocol.hpp"
#include <cstring>
#include <iostream>

namespace rcia::serial_driver::protocol {
InfantryProtocol::InfantryProtocol(rcia::serial_driver::UartTransporter& uart)
    : Protocol(uart) {
    }

struct SerialReceiveData InfantryProtocol::receive() {
    uint8_t header[UniversalFrame::HEADER_SIZE] = {0};
    while (true) {
        if (uart_.read(header, UniversalFrame::HEADER_SIZE) != UniversalFrame::HEADER_SIZE) {
            throw std::runtime_error("Failed to read header");
        }
        if (header[0] == UniversalFrame::HEADER && header[1] == UniversalFrame::HEADER) {
            uint8_t data[UniversalFrame::DATA_SIZE] = {0};
            if (uart_.read(data, UniversalFrame::DATA_SIZE) != UniversalFrame::DATA_SIZE) {
                throw std::runtime_error("Failed to read data");
            }
            return parse_data(data);
        }
    }
}

void InfantryProtocol::transmit(struct SerialTransmitData* serial_send_data_ptr) {
    uint8_t buffer[UniversalFrame::HEADER_SIZE + UniversalFrame::DATA_SIZE + 1] = {0};
    pack_header(buffer);
    pack_data(buffer + UniversalFrame::HEADER_SIZE, serial_send_data_ptr);
    uart_.write(buffer, sizeof(buffer));
}

void InfantryProtocol::pack_header(uint8_t* buffer) {
    buffer[0] = UniversalFrame::HEADER;
    buffer[1] = UniversalFrame::HEADER;
}


// 跟电控通讯有问题
// 可以让电控修改成不用DMA
// 让电控修改优先级
void InfantryProtocol::pack_data(uint8_t* buffer, const struct SerialTransmitData* serial_send_data_ptr) {
    buffer[0] = (serial_send_data_ptr->pitch_angle >> 24) & 0xFF;
    buffer[1] = (serial_send_data_ptr->pitch_angle  >> 16) & 0xFF;
    buffer[2] = (serial_send_data_ptr->pitch_angle  >> 8) & 0xFF;
    buffer[3] = serial_send_data_ptr->pitch_angle  & 0xFF;

    buffer[4] = (serial_send_data_ptr->yaw_angle >> 24) & 0xFF;
    buffer[5] = (serial_send_data_ptr->yaw_angle >> 16) & 0xFF;
    buffer[6] = (serial_send_data_ptr->yaw_angle >> 8) & 0xFF;
    buffer[7] = serial_send_data_ptr->yaw_angle & 0xFF;

    buffer[8] = serial_send_data_ptr->find_flag;
    buffer[9] = serial_send_data_ptr->fire_flag;

    buffer[10] = (serial_send_data_ptr->time_stamp >> 24) & 0xFF;
    buffer[11] = (serial_send_data_ptr->time_stamp >> 16) & 0xFF;
    buffer[12] = (serial_send_data_ptr->time_stamp >> 8) & 0xFF;
    buffer[13] = serial_send_data_ptr->time_stamp & 0xFF;

    buffer[14] = serial_send_data_ptr->operator_ui_x;
    buffer[15] = serial_send_data_ptr->operator_ui_y;
}


struct SerialReceiveData InfantryProtocol::parse_data(const uint8_t* data) {
    struct SerialReceiveData serial_data;

    serial_data.pitch_angle = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];

    serial_data.yaw_angle = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];

    serial_data.bullet_speed = data[8];

    serial_data.our_color = (data[9] == 0x42) ? "red" : "blue";

    serial_data.vision_mode = data[10];

    serial_data.time_stamp = (data[12] << 24) | (data[13] << 16) | (data[14] << 8) | data[15];

    return serial_data;
}

} // namespace rcia::serial_driver::protocol