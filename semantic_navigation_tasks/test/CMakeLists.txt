find_package(ament_index_cpp REQUIRED)

# Test for the shared geometry utilities
ament_add_gtest(test_geometry_utils
  test_geometry_utils.cpp
)
target_include_directories(test_geometry_utils PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
)

# Test for regions
ament_add_gtest(test_regions
  test_regions.cpp
)
target_link_libraries(test_regions
  ${library_name}
  ${polygon_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
)

# Test for the regions file loader (pure domain logic, no rclcpp::init needed)
ament_add_gtest(test_regions_loader
  test_regions_loader.cpp
)
target_link_libraries(test_regions_loader
  ${library_name}
  ament_index_cpp::ament_index_cpp
)

# Test for the region graph
ament_add_gtest(test_region_graph
  test_region_graph.cpp
)
target_link_libraries(test_region_graph
  ${library_name}
  ${polygon_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
)

# Test for the goal sampler (pure domain logic against a synthetic map, no rclcpp::init needed)
ament_add_gtest(test_goal_sampler
  test_goal_sampler.cpp
)
target_link_libraries(test_goal_sampler
  ${library_name}
)

# Test for the region visualization markers (pure domain logic, no rclcpp::init needed)
ament_add_gtest(test_region_markers
  test_region_markers.cpp
)
target_link_libraries(test_region_markers
  ${library_name}
)

# Test for tasks
ament_add_gtest(test_tasks
  test_tasks.cpp
)
target_link_libraries(test_tasks
  ${library_name}
  ament_index_cpp::ament_index_cpp
  nav2_util::nav2_util_core
)

# Test for integration
ament_add_gtest(test_integration
  test_integration.cpp
)
target_link_libraries(test_integration
  ${library_name}
  ament_index_cpp::ament_index_cpp
  nav2_util::nav2_util_core
)
