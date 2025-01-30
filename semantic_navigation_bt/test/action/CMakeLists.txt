# Test for the generate random goals service
ament_add_gtest(test_generate_random_goals_service
  test_generate_random_goals_service.cpp
)
target_link_libraries(test_generate_random_goals_service
  semantic_navigation_generate_random_goals_service_bt_node
)
ament_target_dependencies(test_generate_random_goals_service
  ${dependencies}
)

# Test for the list all regions service
ament_add_gtest(test_get_random_region_service
test_get_random_region_service.cpp
)
target_link_libraries(test_get_random_region_service
  semantic_navigation_get_random_region_service_bt_node
)
ament_target_dependencies(test_get_random_region_service
  ${dependencies}
)

# Test for the get region name service
ament_add_gtest(test_get_region_name
  test_get_region_name.cpp
)
target_link_libraries(test_get_region_name
  semantic_navigation_get_region_name_service_bt_node
)
ament_target_dependencies(test_get_region_name
  ${dependencies}
)

# Test for the list all regions service
ament_add_gtest(test_list_all_regions_service
  test_list_all_regions_service.cpp
)
target_link_libraries(test_list_all_regions_service
  semantic_navigation_list_all_regions_service_bt_node
)
ament_target_dependencies(test_list_all_regions_service
  ${dependencies}
)
