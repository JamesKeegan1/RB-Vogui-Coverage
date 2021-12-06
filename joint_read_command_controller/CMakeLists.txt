cmake_minimum_required(VERSION 2.8.3)
project(joint_read_command_controller)
set (CMAKE_BUILD_TYPE "Debug")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS realtime_tools roscpp hardware_interface pluginlib controller_interface sensor_msgs)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS realtime_tools roscpp hardware_interface pluginlib controller_interface sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/joint_read_command_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES joint_read_command_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
