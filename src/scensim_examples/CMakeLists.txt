cmake_minimum_required(VERSION 3.14)
project(scensim_examples)

find_package(ament_cmake REQUIRED)

# Starter scenarios, installed to share/scensim_examples/scenarios/{basic,multi_av,awf,tutorial,isaac,demo}.
# Every file names its map as a project's scenarios do, $(env SCENSIM_MAPS)/<Town>: copy one into
# a project's scenarios/ and its stack sets SCENSIM_MAPS (README.md).
install(
  DIRECTORY scenarios
  DESTINATION share/${PROJECT_NAME}
)
# Working example projects, installed to share/scensim_examples/projects/<name>: read-only there,
# so a user copies one (`ros2 run scensim project new <dir> --from
# pkg://scensim_examples/projects/<name>`). A project's host-made directories and its lock are
# never installed.
install(
  DIRECTORY projects
  DESTINATION share/${PROJECT_NAME}
  PATTERN "assets" EXCLUDE
  PATTERN "state" EXCLUDE
  PATTERN "runs" EXCLUDE
  PATTERN ".scensim" EXCLUDE
  PATTERN "scensim.lock" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
