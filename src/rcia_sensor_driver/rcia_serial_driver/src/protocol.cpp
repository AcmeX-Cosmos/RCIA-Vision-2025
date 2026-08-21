// protocol.cpp
#include "protocol.hpp"

namespace rcia::serial_driver::protocol {

Protocol::Protocol(rcia::serial_driver::UartTransporter& uart)
    : uart_(uart) {}

} // namespace rcia::serial_driver::protocol