#include "hardware/comms.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <iostream>

bool Comms::init(const std::string &device_path)
{
    i2c_fd_ = open(device_path.c_str(), O_RDWR);
    return i2c_fd_ >= 0;
}

bool Comms::read_encoder(uint8_t encoder_addr, double &position)
{
    if (ioctl(i2c_fd_, I2C_SLAVE, encoder_addr) < 0)
        return false;

    uint8_t reg = 0x0C; // Start register for raw angle
    if (write(i2c_fd_, &reg, 1) != 1)
        return false;

    uint8_t buf[2];
    if (read(i2c_fd_, buf, 2) != 2)
        return false;

    uint16_t raw = ((buf[0] << 8) | buf[1]) & 0x0FFF;  // 12-bit value
    position = static_cast<double>(raw) * (360.0 / 4096.0);  // Scale to degrees
    return true;
}


bool Comms::write_esp32(uint8_t esp32_addr, const std::vector<uint8_t> &data)
{
    if (ioctl(i2c_fd_, I2C_SLAVE, esp32_addr) < 0)
        return false;

    return write(i2c_fd_, data.data(), data.size()) == static_cast<ssize_t>(data.size());
}

bool Comms::read_servo_positions_from_esp32(uint8_t esp32_addr, std::vector<double> &positions)
{
    if (ioctl(i2c_fd_, I2C_SLAVE, esp32_addr) < 0)
        return false;

    uint8_t cmd = 0x01;  // Command code for requesting servo positions
    if (write(i2c_fd_, &cmd, 1) != 1)
        return false;

    uint8_t buf[6];  // 3 servos, 2 bytes each
    if (read(i2c_fd_, buf, 6) != 6)
        return false;

    positions.clear();
    for (int i = 0; i < 3; ++i)
    {
        uint16_t raw = (buf[i * 2] << 8) | buf[i * 2 + 1];
        positions.push_back(static_cast<double>(raw) * 0.01);  // Example scaling
    }
    return true;
}

bool Comms::select_mux_channel(uint8_t channel)
{
    if (channel > 7)
        return false;

    const uint8_t mux_address = 0x70; // Default TCA9548A I2C address
    uint8_t data = 1 << channel;

    if (ioctl(i2c_fd_, I2C_SLAVE, mux_address) < 0)
        return false;

    return write(i2c_fd_, &data, 1) == 1;
}