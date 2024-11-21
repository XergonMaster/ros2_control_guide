#include "yb_eb/yb_eb_HW.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
namespace yb_eb
{

    hardware_interface::CallbackReturn YbEbHw::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_commnds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // Validate joints
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("YbEbHw"), "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("YbEbHw"), "Joint '%s' has %ld state interfaces found. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn YbEbHw::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        // use previus state to check if the hardware is already configured
        if (hardware_interface::SystemInterface::on_configure(previous_state) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (uint i = 0; i < info_.joints.size(); i++)
        {
            hw_commnds_[i] = 0.0;
            hw_states_position_[i] = 0.0;
            hw_states_velocity_[i] = 0.0;
        }
        RCLCPP_INFO(rclcpp::get_logger("YbEbHw"), "YbEbHw Succesfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn YbEbHw::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        if (hardware_interface::SystemInterface::on_activate(previous_state) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("YbEbHw"), "YbEbHw Succesfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn YbEbHw::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        if (hardware_interface::SystemInterface::on_deactivate(previous_state) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("YbEbHw"), "YbEbHw Succesfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> YbEbHw::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> YbEbHw::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commnds_[0]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_commnds_[1]));

        return command_interfaces;
    }
    hardware_interface::return_type YbEbHw::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // use mockup time and period to print the message
        RCLCPP_DEBUG(rclcpp::get_logger("YbEbHw"), "Reading from hardware interface at time %f", time.seconds());
        RCLCPP_DEBUG(rclcpp::get_logger("YbEbHw"), "Reading from hardware interface at period %f", period.seconds());
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type YbEbHw::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("YbEbHw"), "Reading from hardware interface at time %f", time.seconds());
        RCLCPP_DEBUG(rclcpp::get_logger("YbEbHw"), "Reading from hardware interface at period %f", period.seconds());
        return hardware_interface::return_type::OK;
    }
} // namespace yb_eb

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yb_eb::YbEbHw, hardware_interface::SystemInterface)
