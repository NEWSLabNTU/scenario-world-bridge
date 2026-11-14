cmake_minimum_required(VERSION 3.14)
project(swb_examples)

find_package(ament_cmake REQUIRED)

# Starter scenarios, installed to share/swb_examples/scenarios/{basic,multi_av,awf,tutorial,isaac}.
# Every file names its map as $(env CARLA_MAPS)/<Town>: export CARLA_MAPS (the directory
# holding Town01/, Town02/, ...) before running one.
install(
  DIRECTORY scenarios
  DESTINATION share/${PROJECT_NAME}
)
# Working example projects, installed to share/swb_examples/projects/<name>: read-only there,
# so a user copies one (`ros2 run swb project new <dir> --from
# pkg://swb_examples/projects/<name>`). A project's host-made directories and its lock are
# never installed.
install(
  DIRECTORY projects
  DESTINATION share/${PROJECT_NAME}
  PATTERN "assets" EXCLUDE
  PATTERN "state" EXCLUDE
  PATTERN "runs" EXCLUDE
  PATTERN ".swb" EXCLUDE
  PATTERN "swb.lock" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
