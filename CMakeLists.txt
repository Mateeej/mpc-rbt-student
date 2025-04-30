cmake_minimum_required(VERSION 3.8)
project(mpc_rbt_student)

# Use C++ 17
SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Setup Debug and Release flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-Wno-unused-parameter -Wno-unused-variable)

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mpc_rbt_simulator
  nav_msgs
  nav2_msgs
  rclcpp_action
  tf2_geometry_msgs
  )        

add_library(${PROJECT_NAME} SHARED
    src/KeyboardControl.cpp
    src/Localization.cpp
    src/Planning.cpp
    src/MotionControl.cpp
    )

add_executable(keyboard_control src/keyboard_control_node.cpp)
target_link_libraries(keyboard_control ${PROJECT_NAME})
install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})

add_executable(localization src/localization_node.cpp)
target_link_libraries(localization ${PROJECT_NAME})
install(TARGETS localization DESTINATION lib/${PROJECT_NAME})

add_executable(planning src/planning_node.cpp)
target_link_libraries(planning ${PROJECT_NAME})
install(TARGETS planning DESTINATION lib/${PROJECT_NAME})

add_executable(motion_control src/motion_control_node.cpp)
target_link_libraries(motion_control ${PROJECT_NAME})
install(TARGETS motion_control DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})  
ament_package()
