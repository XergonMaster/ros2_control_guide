#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gripper_ctrl_pkg/gripper_ctrl.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;

class TestGripperController : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp() override
  {
    // Inicializamos el controlador
    controller_ = std::make_unique<gripper_ctrl_pkg::GripperCtrl>();
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  /// Método para asignar recursos al controlador (interfaces de hardware)
  void assignResources()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(linear_joint_pos_state_);
    state_ifs.emplace_back(rotational_joint_pos_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(linear_joint_vel_cmd_);
    command_ifs.emplace_back(rotational_joint_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  controller_interface::return_type InitController(const std::string ns = "")
  {
    (void) ns;
    // Definir opciones del nodo
    auto node_options = rclcpp::NodeOptions();
    
    // Inicializamos el controlador
    return controller_->init("test_gripper_controller", urdf_, node_options); // <-- corregido para eliminar parámetros adicionales
  }

  const std::string urdf_ = "";  // Aquí puedes pasar el contenido del URDF
  std::unique_ptr<gripper_ctrl_pkg::GripperCtrl> controller_;

  std::vector<double> position_values_ = {0.1, 0.2};
  std::vector<double> velocity_values_ = {0.01, 0.02};

  hardware_interface::StateInterface linear_joint_pos_state_{
    "linear_joint", HW_IF_POSITION, &position_values_[0]};
  hardware_interface::StateInterface rotational_joint_pos_state_{
    "rotational_joint", HW_IF_POSITION, &position_values_[1]};
  hardware_interface::CommandInterface linear_joint_vel_cmd_{
    "linear_joint", HW_IF_VELOCITY, &velocity_values_[0]};
  hardware_interface::CommandInterface rotational_joint_vel_cmd_{
    "rotational_joint", HW_IF_VELOCITY, &velocity_values_[1]};
};

TEST_F(TestGripperController, init_fails_without_parameters)
{
  const auto ret =
    controller_->init("test_gripper_controller", urdf_, rclcpp::NodeOptions());  // <-- corregido para eliminar método no existente
  ASSERT_EQ(ret, controller_interface::return_type::ERROR);
}

TEST_F(TestGripperController, configure_succeeds_when_joints_are_specified)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(2));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, SizeIs(2));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(TestGripperController, activate_succeeds_with_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResources();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestGripperController, cleanup)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResources();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
