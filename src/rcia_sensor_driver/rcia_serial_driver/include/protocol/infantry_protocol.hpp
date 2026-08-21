// protocol.hpp
#pragma once

#include "../include/protocol.hpp"

#include <tuple>
#include <string>

namespace rcia::serial_driver::protocol {
class InfantryProtocol : public Protocol {
public:
    explicit InfantryProtocol(rcia::serial_driver::UartTransporter& uart);
    
    ~InfantryProtocol() = default;

    // 接收解析
    struct SerialReceiveData receive() override;
    
    // 发送打包
    void transmit(struct SerialTransmitData* serial_send_data_ptr) override;

private:
    // 数据打包
    void pack_data(uint8_t* buffer, const struct SerialTransmitData* serial_send_data_ptr);
    
    void pack_header(uint8_t* buffer);

    struct SerialReceiveData parse_data(const uint8_t* data);
};

} // namespace rcia::serial_driver::protocol