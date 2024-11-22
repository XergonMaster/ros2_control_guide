#ifndef YB_EB__YB_EB_HW_HPP_
#define YB_EB__YB_EB_HW_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "yb_eb/visibility_control.h"

namespace yb_eb
{

  class YbEbHw : public hardware_interface::SystemInterface
  {
    struct Config
    {
      bool debug = false;
      std::string com_port = "/dev/myserial";
      std::string motor_name = "motor_joint";
      std::string steering_name = "steering_joint";
      double max_forward_motor = 98.0;
      double min_forward_motor = 91.0;
      double max_reverse_motor = 89.0;
      double min_reverse_motor = 76.0;
      double max_ang_steering = 115.0;
      double min_ang_steering = 65.0;
      double center_ang_steering = 90.0;
      double zero_ang_motor = 90.0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(YbEbHw)

    YB_EB_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    YB_EB_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    YB_EB_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    YB_EB_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    YB_EB_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    YB_EB_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    YB_EB_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    YB_EB_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    // YbEbHw();

    // virtual ~YbEbHw();

  private:
    int state_machine(int current_state, double cmd_motor);

    std::vector<double> hw_commnds_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
    int state = 0;
    Config cfg_;
  };

} // namespace yb_eb

#endif // YB_EB__YB_EB_HW_HPP_
