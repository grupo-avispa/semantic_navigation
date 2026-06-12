cmake_minimum_required(VERSION 3.5)
project(semantic_navigation_rviz_plugins)

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

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_msgs REQUIRED)
find_package(polygon_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
set(QT_NO_CREATE_VERSIONLESS_TARGETS TRUE)
find_package(Qt5 COMPONENTS Core Gui Widgets QUIET)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)
find_package(semantic_navigation_tasks REQUIRED)
find_package(visualization_msgs REQUIRED)

# In Kilted and older distros, rviz with Qt6 is not supported
if(TARGET Qt5::Core)
  set(QT_VERSION_MAJOR 5)
  set(QT_VERSION ${Qt5Core_VERSION})

  # In Qt5, state machine is part of core
  find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
elseif(TARGET Qt6::Core)
  set(QT_VERSION_MAJOR 6)
  set(QT_VERSION ${Qt6Core_VERSION})
  find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets StateMachine Test Concurrent)
endif()

if(${QT_VERSION} VERSION_LESS 5.15.0)
  function(qt_wrap_cpp out)
    qt5_wrap_cpp(_sources ${ARGN})
    set("${out}" ${_sources} PARENT_SCOPE)
  endfunction()

  function(qt_wrap_ui out)
    qt5_wrap_ui(_uis_hdrs ${ARGN})
    set("${out}" ${_uis_hdrs} PARENT_SCOPE)
  endfunction()
endif()

set(QT_LIBRARIES
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::Widgets
)

if(QT_VERSION_MAJOR EQUAL 6)
  list(APPEND QT_LIBRARIES Qt6::StateMachine)
endif()

# ##########
# # Build ##
# ##########
set(semantic_navigation_headers_to_moc
  include/${PROJECT_NAME}/semantic_annotation_tool.hpp
  include/${PROJECT_NAME}/semantic_navigation_panel.hpp
)

set(semantic_navigation_sources
  src/semantic_annotation_tool.cpp
  src/semantic_navigation_panel.cpp
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  ${semantic_navigation_tasks_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  ${semantic_navigation_headers_to_moc}
  ${semantic_navigation_sources}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Qt${QT_VERSION_MAJOR}Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${nav2_msgs_TARGETS}
  ${polygon_msgs_TARGETS}
  Qt${QT_VERSION_MAJOR}::Widgets
  ${semantic_navigation_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  ${visualization_msgs_TARGETS}
  PRIVATE
  nav2_util::nav2_util_core
  polygon_utils::polygon_utils
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SEMANTIC_NAVIGATION_RVIZ_PLUGIN_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# ############
# # Install ##
# ############
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
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

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  nav2_msgs
  nav2_util
  pluginlib
  polygon_msgs
  polygon_utils
  rclcpp
  rclcpp_action
  rviz_common
  rviz_rendering
  semantic_navigation_msgs
  semantic_navigation_tasks
  visualization_msgs
  Qt${QT_VERSION_MAJOR}
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
