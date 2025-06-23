#include "hardware/hardware_components.hpp"
#include <iostream>

// ======================= Esp32Interface =======================
Esp32Interface::Esp32Interface(const std::string &id, int i2c_address)
    : id_(id), i2c_address_(i2c_address)
{
    is_connected_ = false;
}

bool Esp32Interface::connect()
{
    std::cout << "[Esp32Interface] Connecting to ESP32 at I2C address " << i2c_address_ << "\n";
    is_connected_ = true;
    return true;
}

void Esp32Interface::disconnect()
{
    std::cout << "[Esp32Interface] Disconnected\n";
    is_connected_ = false;
}

bool Esp32Interface::read_positions(std::vector<int> &encoder_values)
{
    if (!is_connected_) return false;

    // Simulate 3 encoder values
    encoder_values = {100, 200, 300};  // Replace with actual I2C reads
    return true;
}

bool Esp32Interface::send_motor_commands(const std::vector<double> &commands)
{
    if (!is_connected_) return false;

    std::cout << "[Esp32Interface] Sending commands: ";
    for (double cmd : commands)
        std::cout << cmd << " ";
    std::cout << "\n";

    // TODO: Implement actual I2C transmission
    return true;
}

// ======================= AS5600Interface =======================

AS5600Interface::AS5600Interface(const std::string &id, int i2c_address)
    : id_(id), i2c_address_(i2c_address)
{
    is_connected_ = false;
}

bool AS5600Interface::connect()
{
    std::cout << "[AS5600Interface] Connected to AS5600 at I2C address " << i2c_address_ << "\n";
    is_connected_ = true;
    return true;
}

void AS5600Interface::disconnect()
{
    std::cout << "[AS5600Interface] Disconnected\n";
    is_connected_ = false;
}

bool AS5600Interface::read_positions(int &encoder_values)
{
    if (!is_connected_) return false;

    // Simulate a single encoder value
    encoder_values = 400;  // Replace with real I2C read from AS5600
    return true;
}