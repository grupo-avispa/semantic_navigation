cmake_minimum_required(VERSION 3.8)
project(semantic_navigation_rviz_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)
find_package(slg_msgs REQUIRED)
find_package(polygon_msgs REQUIRED)
find_package(polygon_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

###########
## Build ##
###########
set(semantic_navigation_headers_to_moc
  include/${PROJECT_NAME}/semantic_annotation_tool.hpp
  include/${PROJECT_NAME}/semantic_navigation_panel.hpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  ${semantic_navigation_headers_to_moc}
  src/semantic_annotation_tool.cpp
  src/semantic_navigation_panel.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  pluginlib
  rviz_common
  rviz_rendering
  semantic_navigation_msgs
  slg_msgs
  polygon_msgs
  polygon_utils
  visualization_msgs
  nav2_util
  Qt5
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SEMANTIC_NAVIGATION_RVIZ_PLUGIN_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  pluginlib
  rviz_common
  rviz_rendering
  semantic_navigation_msgs
  slg_msgs
  polygon_msgs
  polygon_utils
  visualization_msgs
  nav2_util
  Qt5
)

ament_package()
