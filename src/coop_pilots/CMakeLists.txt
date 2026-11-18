cmake_minimum_required(VERSION 3.14)
project(coop_pilots)

find_package(ament_cmake REQUIRED)

# The cooperative-driving research code (scripts/coop: the games, arenas, agents and measures
# of roadmaps 074 and 078), installed as data to share/coop_pilots/coop/ so the study
# testbenches that run its agents and score with its measures (c2b, s5, e2) find it through
# the ament index, not through the checkout (roadmap 081 step 4, group C). The modules stay
# where they are and import each other by bare name; a testbench puts this directory on
# sys.path. Under --symlink-install each file links to the source, so an edit there takes
# effect without a rebuild (a new file needs one). A Python package (`from coop_pilots import
# ...`) replaces this once roadmap 078 settles (081, group C).
set(COOP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/coop")
if(NOT EXISTS "${COOP_DIR}/measures.py")
  message(FATAL_ERROR "coop_pilots: ${COOP_DIR} is not scripts/coop of this repository")
endif()
install(
  DIRECTORY "${COOP_DIR}/"
  DESTINATION share/${PROJECT_NAME}/coop
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
