#ifndef GRIPPER_CTRL_PKG__GRIPPER_CTRL_HPP_
#define GRIPPER_CTRL_PKG__GRIPPER_CTRL_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "gripper_ctrl_params.hpp"  // Asegúrate de incluir el archivo de parámetros generado

namespace gripper_ctrl_pkg
{
  class GripperCtrl : public controller_interface::ControllerInterface
  {
    using Twist = geometry_msgs::msg::Twist;

  public:
    GripperCtrl();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  protected:
    struct JointHandle
    {
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;
      std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
    };

    controller_interface::CallbackReturn configure_joints(const std::vector<std::string> &joint_names);

    std::vector<JointHandle> joint_handles_;

    // Parámetros gestionados a través de ParamListener
    std::shared_ptr<gripper_controller::ParamListener> param_listener_;
    gripper_controller::Params params_;


    // Buffer de comando de velocidad
    realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> received_velocity_msg_ptr_;

    // Subscripción a cmd_vel
    rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_;

    rclcpp::Time last_received_cmd_;
    bool is_halted_ = false;

    void halt();
    void handle_velocity_command(const std::shared_ptr<Twist> msg);
  };

} // namespace gripper_ctrl_pkg

#endif // GRIPPER_CTRL_PKG__GRIPPER_CTRL_HPP_
