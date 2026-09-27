cmake_minimum_required(VERSION 3.8)
project(experiment3_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndPlace.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
