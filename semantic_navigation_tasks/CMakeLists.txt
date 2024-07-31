cmake_minimum_required(VERSION 3.5)
project(semantic_navigation_tasks)

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
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(polygon_msgs REQUIRED)
find_package(polygon_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# ##########
# # Build ##
# ##########
set(library_name ${PROJECT_NAME}_core)
set(executable_name ${PROJECT_NAME})

# Add library
add_library(${library_name} SHARED src/task_services.cpp)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name}
  PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${polygon_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${semantic_navigation_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  PRIVATE
  angles::angles
  polygon_utils::polygon_utils
  rclcpp_components::component
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  ${yaml_cpp_vendor_TARGETS}
)

ament_target_dependencies(${library_name} PUBLIC nav2_util) # TODO(ajtudela): Fix this in jazzy

# Add executable
add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name} PRIVATE
  rclcpp::rclcpp
  ${library_name}
)

rclcpp_components_register_nodes(${library_name} "semantic_navigation::SemanticNavigationTasks")

# ############
# # Install ##
# ############
install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES test/regions_test.yaml test/regions_test_empty.yaml
  DESTINATION share/${PROJECT_NAME}/test
)

# ############
# # Testing ##
# ############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  angles
  geometry_msgs
  nav_msgs
  nav2_util
  polygon_msgs
  polygon_utils
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  semantic_navigation_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  yaml_cpp_vendor
)
ament_export_targets(${library_name})
ament_package()
