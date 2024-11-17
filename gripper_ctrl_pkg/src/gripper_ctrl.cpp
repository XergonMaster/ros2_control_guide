
#include "gripper_ctrl_pkg/gripper_ctrl.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace gripper_ctrl_pkg
{
  using controller_interface::interface_configuration_type;
  using controller_interface::InterfaceConfiguration;
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;

  GripperCtrl::GripperCtrl() : class GripperCtrl : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn GripperCtrl::on_init()
  {
    // Correcto uso del espacio de nombres para ParamListener y Params
    try
    {
      param_listener_ = std::make_shared<gripper_controller::ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Error ocurrido en la tapa de inicio: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration GripperCtrl::command_interface_configuration() const
  {

    std::vector<std::string> conf_names;
    // Comandos de velocidad para las dos juntas
    conf_names.push_back(params_.linear_joint_name "/" + HW_IF_VELOCITY);
    conf_names.push_back(params_.rotational_joint_name "/" + HW_IF_VELOCITY);

    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  InterfaceConfiguration GripperCtrl::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;

    config.names.push_back(params_.linear_joint_name + "/" + HW_IF_POSITION);
    config.names.push_back(params_.rotational_joint_name + "/" + HW_IF_VELOCITY);

    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  controller_interface::CallbackReturn GripperCtrl::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    auto logger = get_node()->get_logger();
    // Verificar si los parámetros han sido modificados y actualizarlos
    if (!param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      RCLCPP_INFO(logger, "Parameters were updated");
    }

    cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};

    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
        params_.cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
        [this](const Twist::SharedPtr msg)
        { handle_velocity_command(msg); });

    RCLCPP_INFO(logger, "GripperCtrl configurado con cmd_vel_topic: %s", params_.cmd_vel_topic.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GripperCtrl::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state; // Marcamos el parámetro como no usado
    last_received_cmd_ = get_node()->now();
    is_halted_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GripperCtrl::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state; // Marcamos el parámetro como no usado
    halt();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GripperCtrl::on_cleanup(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state; // Marcamos el parámetro como no usado
    velocity_command_subscriber_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type GripperCtrl::update(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    auto logger = get_node()->get_logger();

    (void)period; // Marcamos el parámetro como no usado
    // Comprobar si el último comando es demasiado antiguo
    if ((time - last_received_cmd_).seconds() > params_.cmd_vel_timeout)
    {
      halt();
      return controller_interface::return_type::OK;
    }

    auto velocity_msg = received_velocity_msg_ptr_.readFromRT();
    if (!velocity_msg || !(*velocity_msg))
    {
      return controller_interface::return_type::OK;
    }

    // Aplicar comandos de velocidad a las juntas
    joint_handles_[0].velocity_command.get().set_value((*velocity_msg)->linear.x);
    joint_handles_[1].velocity_command.get().set_value((*velocity_msg)->angular.z);

    return controller_interface::return_type::OK;
  }

  void GripperCtrl::handle_velocity_command(const std::shared_ptr<Twist> msg)
  {
    received_velocity_msg_ptr_.writeFromNonRT(msg);
    last_received_cmd_ = get_node()->now();
  }

  void GripperCtrl::halt()
  {
    for (auto &joint_handle : joint_handles_)
    {
      joint_handle.velocity_command.get().set_value(0.0);
    }
    is_halted_ = true;
  }

} // namespace gripper_ctrl_pkg

#include "pluginlib/class_list_macros.hpp"

// Registrar el controlador
PLUGINLIB_EXPORT_CLASS(gripper_ctrl_pkg::GripperCtrl, controller_interface::ControllerInterface)
