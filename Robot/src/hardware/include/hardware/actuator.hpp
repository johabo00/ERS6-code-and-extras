#pragma once

#include <string>
#include <vector>
#include <cstdint>

class Actuator
{
public:
    Actuator(const std::string &name, uint8_t encoder_addr, uint8_t esp32_addr, uint8_t esp32_channel, bool uses_external_encoder, int mux_channel);

    const std::string & get_name() const;
    uint8_t get_encoder_address() const;
    uint8_t get_esp32_address() const;
    uint8_t get_esp32_channel() const;
    bool uses_external_encoder() const;
    int get_mux_channel() const;
    bool uses_mux() const;

    void set_position(double pos);
    void set_command(double cmd);
    double get_command() const;
    double get_position() const;

    std::vector<uint8_t> encode_command() const;

private:
    std::string name_;
    uint8_t encoder_addr_;
    uint8_t esp32_addr_;
    uint8_t esp32_channel_;
    bool has_external_encoder_ = true;
    int mux_channel_ = -1;

    double position_ = 0.0;
    double command_ = 0.0;
};