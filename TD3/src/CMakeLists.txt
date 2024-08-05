cmake_minimum_required(VERSION 3.5)
project(td3)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install the Python scripts
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/replay_buffer.py
  ${CMAKE_CURRENT_SOURCE_DIR}/test_velodyne_td3.py
  ${CMAKE_CURRENT_SOURCE_DIR}/train_velodyne_td3.py
  ${CMAKE_CURRENT_SOURCE_DIR}/velodyne_env.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the assets directory
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

ament_package()

