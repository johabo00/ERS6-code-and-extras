#include "hardware/actuator.hpp"

Actuator::Actuator(const std::string &name, uint8_t encoder_addr, uint8_t esp32_addr,
                   uint8_t esp32_channel, bool uses_external_encoder, int mux_channel)
  : name_(name), encoder_addr_(encoder_addr), esp32_addr_(esp32_addr),
    esp32_channel_(esp32_channel), has_external_encoder_(uses_external_encoder),
    mux_channel_(mux_channel) {}

const std::string & Actuator::get_name() const { return name_; }
uint8_t Actuator::get_encoder_address() const { return encoder_addr_; }
uint8_t Actuator::get_esp32_address() const { return esp32_addr_; }
uint8_t Actuator::get_esp32_channel() const { return esp32_channel_; }
bool Actuator::uses_external_encoder() const { return has_external_encoder_; }
bool Actuator::uses_mux() const { return mux_channel_ >= 0; }
int Actuator::get_mux_channel() const { return mux_channel_; }

void Actuator::set_position(double pos) { position_ = pos; }
void Actuator::set_command(double cmd) { command_ = cmd; }
double Actuator::get_command() const { return command_; }
double Actuator::get_position() const { return position_; }

std::vector<uint8_t> Actuator::encode_command() const
{
  int16_t scaled = static_cast<int16_t>(command_ * 1000);  // Convert to fixed-point
  return { esp32_channel_, static_cast<uint8_t>(scaled >> 8), static_cast<uint8_t>(scaled & 0xFF) };
}