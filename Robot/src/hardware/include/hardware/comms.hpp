#pragma once

#include <string>
#include <vector>
#include <cstdint>

class Comms
{
public:
    bool init(const std::string &device_path);

    // Encoder read
    bool read_encoder(uint8_t encoder_addr, double &position);

    // ESP32 write
    bool write_esp32(uint8_t esp32_addr, const std::vector<uint8_t> &data);

    // ESP32 read (for internal servo positions)
    bool read_servo_positions_from_esp32(uint8_t esp32_addr, std::vector<double> &positions);
    bool select_mux_channel(uint8_t channel);

private:
    int i2c_fd_ = -1;
};