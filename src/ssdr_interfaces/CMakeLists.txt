cmake_minimum_required(VERSION 3.5)
project(ssdr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KeyExchange.srv"
  "action/EncryptedAction.action"
)

ament_package()