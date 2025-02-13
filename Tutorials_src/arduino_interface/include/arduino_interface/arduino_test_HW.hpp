#ifndef ARDUINO_INTERFACE__ARDUINO_TEST_HW_HPP_
#define ARDUINO_INTERFACE__ARDUINO_TEST_HW_HPP_
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arduino_interface/visibility_control.h"

namespace arduino_interface
{

  class ArduinoTestHw : public hardware_interface::SystemInterface
  {
  public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    std::vector<double> hw_commnds_;
    std::vector<double> last_hw_commands_;

    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
  };

} // namespace arduino_interface

#endif // ARDUINO_INTERFACE__ARDUINO_TEST_HW_HPP_
