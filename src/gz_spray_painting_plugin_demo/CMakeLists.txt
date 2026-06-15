cmake_minimum_required(VERSION 3.5)
project(gz_spray_painting_plugin_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes models rviz scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(hooks/gz_sim_resource_path.sh.in)

install(PROGRAMS
  scripts/cartesian_path_executor.py
  scripts/spray_painting_demo.py
  scripts/generate_spray_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
