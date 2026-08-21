

// System
#include <errno.h>  /* 错误码 */
#include <fcntl.h>  /* 文件控制 */
#include <stdio.h>  /* 标准输入输出 */
#include <stdlib.h> /* 标准库 */
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h> /* POSIX 串口 */
#include <unistd.h>  /* Unix 标准 */

#include <iostream>
#include <tuple>

#include "../include/uart_transporter.hpp"

namespace rcia::serial_driver {

bool UartTransporter::setParam(int speed, int flow_ctrl, int databits, int stopbits, int parity) {
    // 设置波特率
    int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};
    struct termios options;
    // 获取当前串口设置
    if (tcgetattr(fd_, &options) != 0) {
        error_message_ = "Setup Serial err";
        return false;
    }
    // 设置波特率
    for (size_t i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
        if (speed == name_arr[i]) {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }
    // 设置本地模式
    options.c_cflag |= CLOCAL; // CRTSCTS; // 
    // 设置读取模式
    options.c_cflag |= CREAD;
    // 设置流控制
    switch (flow_ctrl) {
        case 0: // 不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1: // 使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2: // 使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    // 设置数据位
    options.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            error_message_ = "Unsupported data size";
            return false;
    }
    // 设置校验位
    switch (parity) {
        case 'n':
        case 'N': // 无校验
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O': // 奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E': // 偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': // 空格校验
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            error_message_ = "Unsupported parity";
            return false;
    }
    // 设置停止位
    switch (stopbits) {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            error_message_ = "Unsupported stop bits";
            return false;
    }

    // 设置输出模式
    options.c_oflag &= ~OPOST;
    // 设置本地模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // 设置输入模式
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    // 设置超时和最小字符数
    options.c_cc[VTIME] = 1; // 超时 1*(1/10)s
    options.c_cc[VMIN] = 1;  // 最小字符数 1
    tcflush(fd_, TCIFLUSH);

    // 应用设置
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        error_message_ = "com set error";
        return false;
    }
    return true;
}

bool UartTransporter::open() {
    if (is_open_) {
        return true;
    }
    fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fd_) {
        error_message_ = "can't open uart device: " + device_path_;
        return false;
    }
    // 设置文件描述符标志
    if (fcntl(fd_, F_SETFL, 0) < 0) {
        error_message_ = "fcntl failed";
        return false;
    }

    // 设置串口参数
    if (!setParam(speed_, flow_ctrl_, databits_, stopbits_, parity_)) {
        return false;
    }
    is_open_ = true;
    return true;
}

void UartTransporter::close() {
    if (!is_open_) {
        return;
    }
    ::close(fd_);
    fd_ = -1;
    is_open_ = false;
}

bool UartTransporter::isOpen() { return is_open_; }

int UartTransporter::read(void *buffer, size_t len) {
    int ret = ::read(fd_, buffer, len);
    return ret;
}

int UartTransporter::write(const void *buffer, size_t len) {
    int ret = ::write(fd_, buffer, len);
    return ret;
}


}