cmake_minimum_required(VERSION 3.5)
project(semantic_navigation_bt)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    set(CMAKE_CXX_STANDARD 17)
  else()
    message(FATAL_ERROR "cxx_std_17 could not be found.")
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

option(COVERAGE_ENABLED "Enable code coverage" FALSE)

if(COVERAGE_ENABLED)
  add_compile_options(--coverage)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

# Defaults for Microsoft C++ compiler
if(MSVC)
  # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  # Enable Math Constants
  # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)

# ##########
# # Build ##
# ##########
add_library(semantic_navigation_generate_random_goals_service_bt_node SHARED src/action/generate_random_goals_service.cpp)
list(APPEND plugin_libs semantic_navigation_generate_random_goals_service_bt_node)

add_library(semantic_navigation_get_random_region_service_bt_node SHARED src/action/get_random_region_service.cpp)
list(APPEND plugin_libs semantic_navigation_get_random_region_service_bt_node)

add_library(semantic_navigation_get_region_name_service_bt_node SHARED src/action/get_region_name_service.cpp)
list(APPEND plugin_libs semantic_navigation_get_region_name_service_bt_node)

add_library(semantic_navigation_list_all_regions_service_bt_node SHARED src/action/list_all_regions_service.cpp)
list(APPEND plugin_libs semantic_navigation_list_all_regions_service_bt_node)

add_library(semantic_navigation_get_adjacent_regions_service_bt_node SHARED src/action/get_adjacent_regions_service.cpp)
list(APPEND plugin_libs semantic_navigation_get_adjacent_regions_service_bt_node)

add_library(semantic_navigation_are_regions_connected_service_bt_node SHARED src/action/are_regions_connected_service.cpp)
list(APPEND plugin_libs semantic_navigation_are_regions_connected_service_bt_node)

add_library(semantic_navigation_get_region_route_service_bt_node SHARED src/action/get_region_route_service.cpp)
list(APPEND plugin_libs semantic_navigation_get_region_route_service_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
  )
  target_link_libraries(${bt_plugin} PUBLIC
    behaviortree_cpp::behaviortree_cpp
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    ${semantic_navigation_msgs_TARGETS}
  )

  if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    ament_target_dependencies(${bt_plugin} PUBLIC nav2_behavior_tree)
  elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
    target_link_libraries(${bt_plugin}
      PUBLIC
      nav2_behavior_tree::nav2_behavior_tree
    )
  endif()

  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# ############
# # Install ##
# ############
install(TARGETS ${plugin_libs}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# we will embed the list of plugin names inside a header file
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)

add_executable(generate_semantic_navigation_tree_nodes_xml src/generate_semantic_navigation_tree_nodes_xml.cpp)
target_link_libraries(generate_semantic_navigation_tree_nodes_xml PRIVATE
  behaviortree_cpp::behaviortree_cpp
  nav2_util::nav2_util_core
)

# allow generate_semantic_navigation_tree_nodes_xml to find plugins_list.hpp
target_include_directories(generate_semantic_navigation_tree_nodes_xml PRIVATE ${GENERATED_DIR})
install(TARGETS generate_semantic_navigation_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES semantic_navigation_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME})

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries()
ament_export_dependencies(
  behaviortree_cpp
  nav2_behavior_tree
  nav2_util
  nav2_ros_common
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  semantic_navigation_msgs
)
ament_export_targets()
ament_package()
