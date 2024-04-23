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