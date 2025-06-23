// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware/ers6_system.hpp"
#include <rclcpp/logging.hpp>
#include <chrono>
#include <thread>
#include <map>

namespace ros2_control_ers6
{

CallbackReturn RobotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  actuator_configs_ = {
    {"base_link_to_a5", 0x36, 0x21, 0, true, 0},
    {"a5_to_a4",        0x36, 0x21, 1, true, 1},
    {"a4_to_a3",        0x36, 0x21, 2, true, 2},
    {"a3_to_a2",        0x00, 0x20, 0, false, -1},
    {"a2_to_a1",        0x00, 0x20, 1, false, -1},
    {"a1_to_a0",        0x00, 0x20, 2, false, -1}
  };

  for (const auto & cfg : actuator_configs_)
  {
    actuators_.emplace_back(cfg.name, cfg.encoder_address, cfg.esp32_address,
                            cfg.esp32_channel, cfg.has_external_encoder, cfg.mux_channel);
    pos_[cfg.name] = 0.0;
    vel_[cfg.name] = 0.0;
    prev_pos_[cfg.name] = 0.0;
    cmd_[cfg.name] = 0.0;
  }

  if (comms_.init("/dev/i2c-1")) {
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "I2C initialized successfully on /dev/i2c-1");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hardware"), "Failed to initialize I2C on /dev/i2c-1");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

return_type RobotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "READ called");
  std::map<uint8_t, std::vector<Actuator*>> servo_feedback_groups;

  for (auto & actuator : actuators_)
  {
    if (actuator.uses_external_encoder())
    {
      if (actuator.uses_mux()) {
        if (!comms_.select_mux_channel(actuator.get_mux_channel())) {
          RCLCPP_ERROR(rclcpp::get_logger("hardware"),
                       "Failed to select mux channel %d for %s",
                       actuator.get_mux_channel(), actuator.get_name().c_str());
          continue;
        }
      }

      double position = 0.0;
      if (comms_.read_encoder(actuator.get_encoder_address(), position))
      {
        RCLCPP_INFO(rclcpp::get_logger("hardware"), "Encoder read success for %s: %.3f",
                    actuator.get_name().c_str(), position);
        actuator.set_position(position);
        pos_[actuator.get_name()] = position;

        double dt = period.seconds();
        double prev = prev_pos_[actuator.get_name()];
        vel_[actuator.get_name()] = (position - prev) / dt;
        prev_pos_[actuator.get_name()] = position;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("hardware"), "Failed to read encoder for %s", actuator.get_name().c_str());
      }
    }
    else
    {
      servo_feedback_groups[actuator.get_esp32_address()].push_back(&actuator);
    }
  }

  for (auto & [esp_addr, group] : servo_feedback_groups)
  {
    std::vector<double> positions;
    if (comms_.read_servo_positions_from_esp32(esp_addr, positions) && positions.size() == group.size())
    {
      for (size_t i = 0; i < group.size(); ++i)
      {
        group[i]->set_position(positions[i]);
        pos_[group[i]->get_name()] = positions[i];

        double dt = period.seconds();
        double prev = prev_pos_[group[i]->get_name()];
        vel_[group[i]->get_name()] = (positions[i] - prev) / dt;
        prev_pos_[group[i]->get_name()] = positions[i];

        RCLCPP_INFO(rclcpp::get_logger("hardware"), "Servo feedback %s: %.3f",
                    group[i]->get_name().c_str(), positions[i]);
      }
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("hardware"), "Failed to read servo positions from ESP32 at address 0x%X", esp_addr);
    }
  }

  return return_type::OK;
}

return_type RobotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  RCLCPP_INFO(rclcpp::get_logger("hardware"), "WRITE called");
  std::map<uint8_t, std::vector<uint8_t>> esp32_commands;

  for (auto & actuator : actuators_)
  {
    actuator.set_command(cmd_[actuator.get_name()]);
    RCLCPP_INFO(rclcpp::get_logger("hardware"), "CMD %s = %.3f", actuator.get_name().c_str(), cmd_[actuator.get_name()]);
    auto cmd_bytes = actuator.encode_command();
    esp32_commands[actuator.get_esp32_address()].insert(
      esp32_commands[actuator.get_esp32_address()].end(),
      cmd_bytes.begin(), cmd_bytes.end());
  }

  for (auto & [esp_addr, data] : esp32_commands)
  {
    bool success = comms_.write_esp32(esp_addr, data);
    if (success) {
      RCLCPP_INFO(rclcpp::get_logger("hardware"), "Wrote command to ESP32 at 0x%X", esp_addr);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("hardware"), "Failed to write to ESP32 at 0x%X", esp_addr);
    }
  }

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> RobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & cfg : actuator_configs_)
  {
    state_interfaces.emplace_back(cfg.name, hardware_interface::HW_IF_POSITION, &pos_[cfg.name]);
    state_interfaces.emplace_back(cfg.name, hardware_interface::HW_IF_VELOCITY, &vel_[cfg.name]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & cfg : actuator_configs_)
  {
    command_interfaces.emplace_back(cfg.name, hardware_interface::HW_IF_POSITION, &cmd_[cfg.name]);
  }
  return command_interfaces;
}

} // namespace ros2_control_ers6

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_ers6::RobotSystemHardware, hardware_interface::SystemInterface)