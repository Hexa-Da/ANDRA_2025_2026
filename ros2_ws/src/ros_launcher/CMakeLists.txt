cmake_minimum_required(VERSION 3.8)
project(ros_launcher)

find_package(ament_cmake REQUIRED)

# Install launch files
install(FILES
  navigation_stack.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs
)

# Install rviz config
install(FILES
  config.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install map results
install(DIRECTORY map_results/
  DESTINATION share/${PROJECT_NAME}/map_results
)

ament_package()
