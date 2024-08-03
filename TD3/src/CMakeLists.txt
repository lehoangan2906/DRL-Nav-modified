cmake_minimum_required(VERSION 3.5)
project(td3)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  DIRECTORY assets
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()