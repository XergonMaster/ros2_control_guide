#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadGripperController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Reemplazar `gripper_urdf` por una cadena vacía o un URDF válido
  std::string urdf = "<robot name=\"gripper_robot\"></robot>";  // Aquí puedes definir tu URDF

  // Ajuste: eliminar el parámetro booleano y pasar solo el namespace como string
  controller_manager::ControllerManager cm(
    executor, urdf, "test_controller_manager", rclcpp::NodeOptions());  // <-- corregido

  // Reemplazar `TEST_FILES_DIRECTORY` por una ruta válida
  const std::string test_file_path = "/home/xergon/control_ws/src/gripper_ctrl_pkg/config/test_gripper_controller.yaml";

  cm.set_parameter({"test_gripper_controller.params_file", test_file_path});
  cm.set_parameter(
    {"test_gripper_controller.type", "gripper_ctrl_pkg/GripperCtrl"});

  ASSERT_NE(cm.load_controller("test_gripper_controller"), nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
