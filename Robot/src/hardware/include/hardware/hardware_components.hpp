#pragma once

#include <string>
#include <vector>

class Esp32Interface {
    public:
        Esp32Interface(const std::string &id, int i2c_address);
    
        bool connect();
        void disconnect();
    
        bool read_positions(std::vector<int> &encoder_values);
        bool send_motor_commands(const std::vector<double> &commands);
    
    private:
        std::string id_;
        int i2c_address_;
    
        bool is_connected_ = false;
    
        // Optionally: I2C handle, file descriptor, etc.
    };

class AS5600Interface {
    public:
        AS5600Interface(const std::string &id, int i2c_address);

        bool connect();
        void disconnect();
    
        bool read_positions(int &encoder_values);
    
    private:
        std::string id_;
        int i2c_address_;
        int enc_counts_per_rev_ = 0;

        bool is_connected_ = false;
    };