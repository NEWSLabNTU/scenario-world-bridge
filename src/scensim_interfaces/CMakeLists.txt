cmake_minimum_required(VERSION 3.14)
project(scensim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetWeather.srv"
  "srv/GetWeather.srv"
  "srv/LoadMap.srv"
  "msg/TruthEntity.msg"
  "msg/TruthEntities.msg"
  "msg/TruthContact.msg"
  "msg/TruthSignal.msg"
  "msg/TruthFrame.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
