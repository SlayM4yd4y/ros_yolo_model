cmake_minimum_required(VERSION 3.5)
project(ros_yolo_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/headers) 
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(teach_node src/teach_node.cpp)
ament_target_dependencies(teach_node rclcpp OpenCV)
add_executable(detector_node src/detector_node.cpp)
ament_target_dependencies(detector_node rclcpp sensor_msgs cv_bridge OpenCV)
add_executable(card_gen_node src/card_gen_node.cpp)
ament_target_dependencies(card_gen_node rclcpp OpenCV ament_index_cpp)

install(TARGETS
  teach_node
  detector_node
  card_gen_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY headers
  DESTINATION include/${PROJECT_NAME}/headers
)

ament_package()
