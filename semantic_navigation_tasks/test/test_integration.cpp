// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/node_thread.hpp"
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

  nav_msgs::msg::Goals generateRandomGoals(
    unsigned int /*n*/, semantic_navigation::Region /*region*/,
    semantic_navigation::CellLimits /*limits*/) override
  {
    if (region_list_.empty() || map_.data.empty()) {
      return nav_msgs::msg::Goals();
    } else {
      nav_msgs::msg::Goals goals;
      geometry_msgs::msg::PoseStamped goal;
      goal.pose.position.x = 1.0;
      goal.pose.position.y = 0.0;
      goal.pose.position.z = 0.0;
      goals.goals.push_back(goal);
      return goals;
    }
  }

  void createFreeMap(int width, int height, double resolution)
  {
    map_.info.width = width;
    map_.info.height = height;
    map_.info.resolution = resolution;
    map_.data = std::vector<int8_t>(width * height, nav2_util::OCC_GRID_FREE);
  }
};

class SemanticNavigationIntegrationTest : public ::testing::Test
{
public:
  SemanticNavigationIntegrationTest() {}

  ~SemanticNavigationIntegrationTest() {}

  void SetUp()
  {
    // RCLCPP
    rclcpp::init(0, nullptr);
    // Create and configure the semantic node
    node_ = std::make_shared<SemanticNavigationTasksFixture>();
    std::filesystem::path pkg_path;
    ament_index_cpp::get_package_share_directory("semantic_navigation_tasks", pkg_path);
    nav2::declare_parameter_if_not_declared(
      node_, "regions_filename",
      rclcpp::ParameterValue(std::string(pkg_path) + "/test/regions_test.yaml"));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
  }

  void TearDown()
  {
    node_->deactivate();
    node_->cleanup();
    node_->shutdown();
    rclcpp::shutdown();
    publisher_node_.reset();
    executor_.reset();
  }

  void activate()
  {
    node_->configure();
    node_->activate();
  }

  void spin_some()
  {
    executor_->spin_some();
  }

  void spin_publisher(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
  {
    publisher_node_ = std::make_unique<nav2::NodeThread>(node_base);
  }

  nav_msgs::msg::OccupancyGrid getMap()
  {
    return node_->getMap();
  }

  void createFreeMap(int width, int height, double resolution)
  {
    node_->createFreeMap(width, height, resolution);
  }

protected:
  std::shared_ptr<SemanticNavigationTasksFixture> node_;
  std::unique_ptr<nav2::NodeThread> publisher_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(SemanticNavigationIntegrationTest, mapCallback) {
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

  // Activate the semantic node
  activate();

  // Create the map message
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.5;
  map.data = std::vector<int8_t>(100, nav2_util::OCC_GRID_FREE);
  map_pub->publish(map);

  // Spin both nodes multiple times to ensure message delivery
  for (int i = 0; i < 10; ++i) {
    executor_->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Check the results
  EXPECT_EQ(getMap().info.width, 10);
  EXPECT_EQ(getMap().info.height, 10);
  EXPECT_DOUBLE_EQ(getMap().info.resolution, 0.5);
  EXPECT_EQ(map_pub->get_subscription_count(), 1);

  // Deactivate the nodes
  pub_node->deactivate();
  pub_node->cleanup();
  pub_node->shutdown();
}

TEST_F(SemanticNavigationIntegrationTest, generateRandomGoalsEmptyRegion) {
  // Activate the node
  activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Request>();
  req->n = 1;
  req->region_name = "region_0";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GenerateRandomGoals>(
    "generate_random_goals");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Wait before checking the results
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check results
  EXPECT_EQ(resp->goals.goals.size(), 0);
}

TEST_F(SemanticNavigationIntegrationTest, generateRandomGoalsEmptyMap) {
  // Activate the node
  activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Request>();
  req->n = 1;
  req->region_name = "small1";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GenerateRandomGoals>(
    "generate_random_goals");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Wait before checking the results
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check results
  EXPECT_EQ(resp->goals.goals.size(), 0);
}

TEST_F(SemanticNavigationIntegrationTest, generateRandomGoalsRegion) {
  // Create a map of 10x10 cells
  node_->createFreeMap(10, 10, 0.5);

  // Activate the node
  activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Request>();
  req->n = 1;
  req->region_name = "small1";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GenerateRandomGoals>(
    "generate_random_goals");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Wait before checking the results
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check results
  EXPECT_EQ(resp->goals.goals.size(), 1);
  EXPECT_DOUBLE_EQ(resp->goals.goals[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(resp->goals.goals[0].pose.position.y, 0.0);
}

TEST_F(SemanticNavigationIntegrationTest, getRandomRegion) {
  // Activate the node
  activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRandomRegion::Request>();
  auto client =
    node_->create_client<semantic_navigation_msgs::srv::GetRandomRegion>("get_random_region");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRandomRegion::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Wait before checking the results
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check results
  EXPECT_FALSE(resp->region_name.empty());
}

TEST_F(SemanticNavigationIntegrationTest, getRegionNameInside) {
  // Activate the node
  activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Request>();
  req->position.header.frame_id = "map";
  req->position.point.x = 0.5;
  req->position.point.y = 0.5;
  auto client = node_->create_client<semantic_navigation_msgs::srv::GetRegionName>(
    "get_region_name");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Wait before checking the results
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check results
  EXPECT_EQ(resp->region_name, "small1");
}

TEST_F(SemanticNavigationIntegrationTest, getRegionNameOutside) {
  // Activate the node
  activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Request>();
  req->position.header.frame_id = "map";
  req->position.point.x = -0.5;
  req->position.point.y = -0.5;
  auto client = node_->create_client<semantic_navigation_msgs::srv::GetRegionName>(
    "get_region_name");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Wait before checking the results
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check results
  EXPECT_EQ(resp->region_name, "unknown");
}

TEST_F(SemanticNavigationIntegrationTest, listAllRegions) {
  // Activate the node
  activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::ListAllRegions::Request>();
  auto client = node_->create_client<semantic_navigation_msgs::srv::ListAllRegions>(
    "list_all_regions");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::ListAllRegions::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Wait before checking the results
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check results
  EXPECT_EQ(resp->region_names.size(), 4);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
