cmake_minimum_required(VERSION 3.14)
project(scensim_study_scenarios)

find_package(ament_cmake REQUIRED)

# The studies' shared scenarios (roadmap 081 step 4), installed to
# share/scensim_study_scenarios/{bench,gap}: the frame-budget benchmark (bench/, written by
# scripts/gen_npc_benchmark.py) and the reproducers of fixed SSv2 and bridge gaps (gap/). The
# study testbenches under scenarios/projects/ name them as pkg://scensim_study_scenarios/....
install(
  DIRECTORY bench gap
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
