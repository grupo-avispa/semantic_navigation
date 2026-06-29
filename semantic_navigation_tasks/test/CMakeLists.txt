find_package(ament_index_cpp REQUIRED)

# Test for regions
ament_add_gtest(test_regions
  test_regions.cpp
)
target_link_libraries(test_regions
  ${library_name}
  ${polygon_msgs_TARGETS}
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

# Test for tasks
ament_add_gtest(test_tasks
  test_tasks.cpp
)
target_link_libraries(test_tasks
  ${library_name}
  ament_index_cpp::ament_index_cpp
)

# Test for integration
ament_add_gtest(test_integration
  test_integration.cpp
)
target_link_libraries(test_integration
  ${library_name}
  ament_index_cpp::ament_index_cpp
)