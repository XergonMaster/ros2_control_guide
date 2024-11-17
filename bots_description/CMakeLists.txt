cmake_minimum_required(VERSION 3.8)
project(bots_description)


find_package(ament_cmake REQUIRED)

install(
  DIRECTORY common/urdf common/rviz
  DESTINATION share/${PROJECT_NAME}/common
)
install(
  DIRECTORY gripper/urdf gripper/rviz
  DESTINATION share/${PROJECT_NAME}/gripper
)
install(
  DIRECTORY ackerman/urdf ackerman/rviz
  DESTINATION share/${PROJECT_NAME}/ackerman
)



ament_package()
