// Copyright (c) 2020 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "semantic_navigation_tasks/task_services.hpp"

class SemanticNavigationTasksFixture : public semantic_navigation::SemanticNavigationTasks
{
public:
  SemanticNavigationTasksFixture()
  : SemanticNavigationTasks() {}

  bool getRegionsFromFile(
    const std::string & filename, std::vector<semantic_navigation::Region> & regions)
  {
    return SemanticNavigationTasks::getRegionsFromFile(filename, regions);
  }
  nav_msgs::msg::OccupancyGrid getMap()
  {
    return map_;
  }
};

TEST(SemanticNavigationTasksTest, mapCallback) {
  rclcpp::init(0, nullptr);
  // Create the map publisher node
  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("map_publisher");
  pub_node->configure();
  // Create a publisher for the map
  auto map_pub =
    pub_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ASSERT_EQ(map_pub->get_subscription_count(), 0);
  EXPECT_FALSE(map_pub->is_activated());
  // Activate the publisher
  pub_node->activate();
  EXPECT_TRUE(map_pub->is_activated());
  map_pub->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  // Create and configure the semantic node
  auto semantic_node = std::make_shared<SemanticNavigationTasksFixture>();
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  nav2_util::declare_parameter_if_not_declared(
    semantic_node, "regions_filename", rclcpp::ParameterValue(pkg + "/test/test_regions.yaml"));
  semantic_node->configure();
  semantic_node->activate();

  // Create the map message
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.5;
  map.data = std::vector<int8_t>(100, nav2_util::OCC_GRID_FREE);
  map_pub->publish(map);

  // Spin the semantic node
  rclcpp::spin_some(semantic_node->get_node_base_interface());

  // Check the results
  EXPECT_EQ(semantic_node->getMap().info.width, 10);
  EXPECT_EQ(semantic_node->getMap().info.height, 10);
  EXPECT_DOUBLE_EQ(semantic_node->getMap().info.resolution, 0.5);
  EXPECT_EQ(map_pub->get_subscription_count(), 1);

  // Deactivate the nodes
  semantic_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}
