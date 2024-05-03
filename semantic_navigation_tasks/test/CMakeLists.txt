find_package(ament_index_cpp REQUIRED)

# Test for tasks
ament_add_gtest(test_tasks
  test_tasks.cpp
)
ament_target_dependencies(test_tasks
  ${dependencies} ament_index_cpp
)
target_link_libraries(test_tasks
  ${library_name}
)

# Test for map integration
ament_add_gtest(test_map_integration
test_map_integration.cpp
)
ament_target_dependencies(test_map_integration
  ${dependencies} ament_index_cpp
)
target_link_libraries(test_map_integration
  ${library_name}
)