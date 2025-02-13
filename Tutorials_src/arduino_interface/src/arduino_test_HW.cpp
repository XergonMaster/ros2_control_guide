#include "arduino_interface/arduino_test_HW.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_interface
{

    hardware_interface::CallbackReturn ArduinoTestHw::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change On init State");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_commnds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        last_hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // Validate joints
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("ArduinoTestHw"), "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("ArduinoTestHw"), "Joint '%s' has %ld state interfaces found. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArduinoTestHw::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change on_configure State");

        (void)previous_state; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            hw_commnds_[i] = 0.0;
            hw_commnds_[i] = 0.0;
            last_hw_commands_[i] = 0.0;
            hw_states_position_[i] = 0.0;
            hw_states_velocity_[i] = 0.0;
        }
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "ArduinoTestHw Succesfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArduinoTestHw::export_state_interfaces()
    {

        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArduinoTestHw::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commnds_[0]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commnds_[0]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_commnds_[1]));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ArduinoTestHw::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change on_activate State");

        (void)previous_state; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArduinoTestHw::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change on_deactivate State");

        (void)previous_state; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArduinoTestHw::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;   // Para evitar la advertencia de parámetro no utilizado
        (void)period; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArduinoTestHw::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;   // Para evitar la advertencia de parámetro no utilizado
        (void)period; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::return_type::OK;
    }

} // namespace arduino_interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arduino_interface::ArduinoTestHw, hardware_interface::SystemInterface)
