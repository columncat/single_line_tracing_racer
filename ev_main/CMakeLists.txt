cmake_minimum_required(VERSION 3.0.2)
project(ev_main)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(FILES MotorMsg.msg ServoMsg.msg)
generate_messages(DEPENDENCIES std_msgs)

find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv_line_detection
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ev_main_node src/ev_main_node.cpp)
add_dependencies(ev_main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ev_main_node
${catkin_LIBRARIES} 
${OpenCV_LIBRARIES}
)
