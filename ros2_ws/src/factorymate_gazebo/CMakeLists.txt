cmake_minimum_required(VERSION 3.8)
project(factorymate_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  config
  rviz
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/worker_controller.py
  scripts/depth_to_laserscan.py
  scripts/aloha_action_bridge.py
  scripts/aloha_task_orchestrator.py
  scripts/grasp_attach.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
