#ifndef ACKERMANN_HARDWARE_INTERFACE_HPP
#define ACKERMANN_HARDWARE_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "yb_board/Rosmaster.hpp"
#include <memory>
#include <vector>

namespace ackermann_robot
{
    class AckermannHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    private:
        std::shared_ptr<Rosmaster> board_;
        double steering_angle_;
        double motor_speed_;
        double steering_angle_command_;
        double motor_speed_command_;
    };
} // namespace ackermann_robot

#endif // ACKERMANN_HARDWARE_INTERFACE_HPP
