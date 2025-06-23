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

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "hardware/comms.hpp"
#include "hardware/actuator.hpp"

using hardware_interface::return_type;

namespace ros2_control_ers6
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystemHardware)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  struct ActuatorConfig
  {
    std::string name;
    uint8_t encoder_address;
    uint8_t esp32_address;
    uint8_t esp32_channel;
    bool has_external_encoder;
    int mux_channel;

    ActuatorConfig(const std::string &n, uint8_t enc, uint8_t esp, uint8_t chan, bool ext, int muxc)
      : name(n), encoder_address(enc), esp32_address(esp), esp32_channel(chan), has_external_encoder(ext), mux_channel(muxc) {}
  };

  std::vector<Actuator> actuators_;
  std::vector<ActuatorConfig> actuator_configs_;

  Comms comms_;

  std::unordered_map<std::string, double> pos_;
  std::unordered_map<std::string, double> vel_;
  std::unordered_map<std::string, double> cmd_;
  std::unordered_map<std::string, double> prev_pos_;

};

}  // namespace ros2_control_ers6
