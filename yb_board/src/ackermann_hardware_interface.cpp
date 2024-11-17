#include "yb_board/ackermann_hardware_interface.hpp"

namespace ackermann_robot
{
    hardware_interface::CallbackReturn AckermannHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Inicializamos el objeto Rosmaster
        board_ = std::make_shared<Rosmaster>(1, "/dev/myserial", true);

        // Inicializamos las variables internas
        steering_angle_ = 0.0;
        motor_speed_ = 0.0;

        RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareInterface"), "Hardware interface initialized successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> AckermannHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface("steering", "position", &steering_angle_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("motor", "velocity", &motor_speed_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> AckermannHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface("steering", "position", &steering_angle_command_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("motor", "velocity", &motor_speed_command_));
        return command_interfaces;
    }

    hardware_interface::return_type AckermannHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Aquí podrías leer el estado actual del hardware si es necesario
        // Por ejemplo, podrías obtener la posición real del servo o la velocidad del motor
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type AckermannHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Limitar los valores de los comandos si es necesario
        if (steering_angle_command_ < 0.0)
            steering_angle_command_ = 0.0;
        if (steering_angle_command_ > 180.0)
            steering_angle_command_ = 180.0;

        // Enviar los comandos al hardware usando la librería Rosmaster
        board_->set_pwm_servo(1, steering_angle_command_);
        board_->set_pwm_servo(2, 180 - steering_angle_command_);

        // Enviar la velocidad al motor (esto puede ser adaptado según el protocolo del motor)
        board_->set_pwm_servo(3, motor_speed_command_); // Esto es un ejemplo; revisa si el canal y método son correctos

        RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareInterface"), "Commands sent - Steering: %.2f, Motor: %.2f", steering_angle_command_, motor_speed_command_);
        return hardware_interface::return_type::OK;
    }

} // namespace ackermann_robot

PLUGINLIB_EXPORT_CLASS(ackermann_robot::AckermannHardwareInterface, hardware_interface::SystemInterface)
