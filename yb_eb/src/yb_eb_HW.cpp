#include "yb_eb/yb_eb_HW.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm> // Para std::min y std::max
#include <cmath>     // Para M_PI si usas PI

namespace yb_eb
{

    hardware_interface::CallbackReturn YbEbHw::on_init(const hardware_interface::HardwareInfo &info)
    {
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
                RCLCPP_FATAL(rclcpp::get_logger("YbEbHw"), "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("YbEbHw"), "Joint '%s' has %ld state interfaces found. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        cfg_.motor_name = (info_.hardware_parameters["motor_name"]);
        cfg_.steering_name = (info_.hardware_parameters["steering_name"]);
        std::istringstream(info_.hardware_parameters["debug"]) >> std::boolalpha >> cfg_.debug;
        cfg_.max_ang_steering = std::stod(info_.hardware_parameters["max_ang_steering"]);
        cfg_.min_ang_steering = std::stod(info_.hardware_parameters["min_ang_steering"]);
        cfg_.max_forward_motor = std::stod(info_.hardware_parameters["max_forward_motor"]);
        cfg_.min_forward_motor = std::stod(info_.hardware_parameters["min_forward_motor"]);
        cfg_.max_reverse_motor = std::stod(info_.hardware_parameters["max_reverse_motor"]);
        cfg_.min_reverse_motor = std::stod(info_.hardware_parameters["min_reverse_motor"]);
        std::istringstream(info_.hardware_parameters["create_receive_threading"]) >> std::boolalpha >> cfg_.create_receive_threading;

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
            hw_commnds_[i] = 0.0;
            last_hw_commands_[i] = 0.0;
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
        if (!board_)
        {
            board_ = std::make_shared<EBSerial>(1, "/dev/myserial", cfg_.debug);
        }
        if (cfg_.create_receive_threading)
        {
            board_->create_receive_threading();
        }

        board_->set_beep(0.1);
        RCLCPP_INFO(rclcpp::get_logger("YbEbHw"), "YbEbHw Succesfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn YbEbHw::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        if (hardware_interface::SystemInterface::on_deactivate(previous_state) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Detenemos el hilo de recepción de datos y reseteamos el puntero para liberar la memoria
        if (board_)
        {
            board_->set_beep(1);
            board_->stop_receiving(); // Asegúrate de que `stop_receiving()` detenga correctamente el hilo de recepción
            board_.reset();           // Libera el recurso gestionado por `std::shared_ptr`
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
        // Mensajes de depuración para el tiempo y periodo
        RCLCPP_DEBUG(rclcpp::get_logger("YbEbHw"), "Reading from hardware interface at time %f", time.seconds());
        RCLCPP_DEBUG(rclcpp::get_logger("YbEbHw"), "Reading from hardware interface at period %f", period.seconds());

        // Imprimir comandos de las joints
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("YbEbHw"),
                "Command for %s is: %f",
                info_.joints[i].name.c_str(),
                hw_commnds_[i]);
        }

        // Scaling de los comandos a ángulos en grados
        double cmd_motor = hw_commnds_[1] * 180 / M_PI + cfg_.center_ang_steering;
        double cmd_steering = hw_commnds_[0] * 180 / M_PI + cfg_.zero_ang_motor;

        cmd_steering = std::max(cfg_.min_ang_steering, std::min(cmd_steering, cfg_.max_ang_steering));
        // Llamada a la máquina de estados para manejar el estado actual
        state = state_machine(state, cmd_motor);

        // Limitar los valores según el estado actual
        switch (state)
        {
        case 1: // Transición a avanzar
            cmd_motor = cfg_.min_forward_motor;
            break;
        case 2: // Transición a reversa
            cmd_motor = cfg_.max_reverse_motor;
            break;
        case 3: // Avanzar
            cmd_motor = std::min(cmd_motor, cfg_.max_forward_motor);
            break;
        case 4: // Reversa
            cmd_motor = std::max(cmd_motor, cfg_.min_reverse_motor);
            break;
        default: // Parado
            cmd_motor = cfg_.zero_ang_motor;
            break;
        }
        if (last_hw_commands_[0] != hw_commnds_[0])
        {
            board_->set_pwm_servo(3, static_cast<float>(cmd_steering)); // Convierte a float antes de enviar
        }
        if (last_hw_commands_[1] != hw_commnds_[1])
        {
            board_->set_pwm_servo(2, static_cast<float>(cmd_motor)); // Convierte a float antes de enviar
        }
        last_hw_commands_ = hw_commnds_; // Guarda los comandos actuales
        // Mensaje de depuración del estado
        RCLCPP_DEBUG(rclcpp::get_logger("YbEbHw"), "State: %d, Command Motor: %f, Command Steering: %f", state, cmd_motor, cmd_steering);
        return hardware_interface::return_type::OK;
    }

    // Nueva función: Máquina de Estados
    int YbEbHw::state_machine(int current_state, double cmd_motor)
    {
        // Máquina de estados basada en el comando del motor
        if (current_state == 0) // Parado
        {
            if (cmd_motor > cfg_.zero_ang_motor)
                return 1; // Transición a avanzar
            else if (cmd_motor < cfg_.zero_ang_motor)
                return 2; // Transición a reversa
        }
        else if (current_state == 1 && cmd_motor > cfg_.zero_ang_motor) // Transición a avanzar
        {
            return 3; // Avanzar
        }
        else if (current_state == 2 && cmd_motor < cfg_.zero_ang_motor) // Transición a reversa
        {
            return 4; // Reversa
        }
        else if (current_state == 3 && cmd_motor <= cfg_.zero_ang_motor) // Avanzar a parado
        {
            return 0; // Parado
        }
        else if (current_state == 4 && cmd_motor >= cfg_.zero_ang_motor) // Reversa a parado
        {
            return 0; // Parado
        }

        return current_state; // Si no hay cambio, mantiene el estado actual
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
