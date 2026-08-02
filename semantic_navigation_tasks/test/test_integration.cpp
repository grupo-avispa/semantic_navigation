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
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/node_thread.hpp"
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
    return goal_sampler_.getMap();
  }

  std::vector<geometry_msgs::msg::PoseStamped> generateRandomGoals(
    unsigned int /*n*/, semantic_navigation::Region /*region*/,
    semantic_navigation::CellLimits /*limits*/, std::string /*orientation*/,
    double /*requested_yaw*/) override
  {
    if (region_list_.empty() || goal_sampler_.getMap().data.empty()) {
      return {};
    } else {
      std::vector<geometry_msgs::msg::PoseStamped> goals;
      geometry_msgs::msg::PoseStamped goal;
      goal.pose.position.x = 1.0;
      goal.pose.position.y = 0.0;
      goal.pose.position.z = 0.0;
      goals.push_back(goal);
      return {goal};
    }
  }

  void createFreeMap(int width, int height, double resolution)
  {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = width;
    map.info.height = height;
    map.info.resolution = resolution;
    map.data = std::vector<int8_t>(width * height, nav2_util::OCC_GRID_FREE);
    goal_sampler_.setMap(map);
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
    auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
    nav2_util::declare_parameter_if_not_declared(
      node_, "regions_filename", rclcpp::ParameterValue(pkg + "/test/regions_test.yaml"));
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
    publisher_node_ = std::make_unique<nav2_util::NodeThread>(node_base);
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
  std::unique_ptr<nav2_util::NodeThread> publisher_node_;
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
  auto result = client->async_send_request(req);

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

  // Check results: an unknown region with full_map disabled is a rejected request
  EXPECT_EQ(resp->goals.size(), 0);
  EXPECT_FALSE(resp->success);
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
  auto result = client->async_send_request(req);

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

  // Check results: no map received yet is a rejected request
  EXPECT_EQ(resp->goals.size(), 0);
  EXPECT_FALSE(resp->success);
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
  auto result = client->async_send_request(req);

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
  EXPECT_EQ(resp->goals.size(), 1);
  EXPECT_DOUBLE_EQ(resp->goals[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(resp->goals[0].pose.position.y, 0.0);
  EXPECT_TRUE(resp->success);
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
  auto result = client->async_send_request(req);

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
  EXPECT_TRUE(resp->success);
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
  auto result = client->async_send_request(req);

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
  EXPECT_TRUE(resp->success);
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
  auto result = client->async_send_request(req);

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

  // Check results: a point outside every region is a normal (successful) outcome
  EXPECT_EQ(resp->region_name, "unknown");
  EXPECT_TRUE(resp->success);
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
  auto result = client->async_send_request(req);

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
  EXPECT_TRUE(resp->success);
}

// The connectivity graph built from regions_test.yaml (default auto_connect and
// connectivity_threshold): small1 and big are connected (forced by the `add` override), small1
// and small2 are not (auto-detected but then explicitly `remove`d), and "outside" is isolated.

TEST_F(SemanticNavigationIntegrationTest, getAdjacentRegionsKnown) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::GetAdjacentRegions::Request>();
  req->region_name = "small1";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GetAdjacentRegions>(
    "get_adjacent_regions");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetAdjacentRegions::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  ASSERT_EQ(resp->adjacent_regions.size(), 1u);
  EXPECT_EQ(resp->adjacent_regions[0], "big");
  EXPECT_TRUE(resp->success);
}

TEST_F(SemanticNavigationIntegrationTest, getAdjacentRegionsUnknown) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::GetAdjacentRegions::Request>();
  req->region_name = "no_such_region";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GetAdjacentRegions>(
    "get_adjacent_regions");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetAdjacentRegions::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  EXPECT_FALSE(resp->success);
}

TEST_F(SemanticNavigationIntegrationTest, areRegionsConnectedTrue) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::AreRegionsConnected::Request>();
  req->region_a = "small1";
  req->region_b = "big";
  auto client = node_->create_client<semantic_navigation_msgs::srv::AreRegionsConnected>(
    "are_regions_connected");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::AreRegionsConnected::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  EXPECT_TRUE(resp->connected);
  EXPECT_TRUE(resp->success);
}

TEST_F(SemanticNavigationIntegrationTest, areRegionsConnectedFalse) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::AreRegionsConnected::Request>();
  req->region_a = "small1";
  req->region_b = "small2";
  auto client = node_->create_client<semantic_navigation_msgs::srv::AreRegionsConnected>(
    "are_regions_connected");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::AreRegionsConnected::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Auto-detected from the geometry, then explicitly removed by the regions file
  EXPECT_FALSE(resp->connected);
  EXPECT_TRUE(resp->success);
}

TEST_F(SemanticNavigationIntegrationTest, areRegionsConnectedUnknown) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::AreRegionsConnected::Request>();
  req->region_a = "small1";
  req->region_b = "no_such_region";
  auto client = node_->create_client<semantic_navigation_msgs::srv::AreRegionsConnected>(
    "are_regions_connected");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::AreRegionsConnected::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  EXPECT_FALSE(resp->success);
}

TEST_F(SemanticNavigationIntegrationTest, getRegionRouteFound) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRegionRoute::Request>();
  req->start_region = "small1";
  req->goal_region = "big";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GetRegionRoute>(
    "get_region_route");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRegionRoute::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  ASSERT_EQ(resp->route.size(), 2u);
  EXPECT_EQ(resp->route[0], "small1");
  EXPECT_EQ(resp->route[1], "big");
  EXPECT_TRUE(resp->success);
}

TEST_F(SemanticNavigationIntegrationTest, getRegionRouteNotFound) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRegionRoute::Request>();
  req->start_region = "small1";
  req->goal_region = "small2";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GetRegionRoute>(
    "get_region_route");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRegionRoute::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // No path between two disconnected, but known, regions is still a successful answer
  EXPECT_TRUE(resp->route.empty());
  EXPECT_TRUE(resp->success);
}

TEST_F(SemanticNavigationIntegrationTest, getRegionRouteUnknown) {
  activate();

  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRegionRoute::Request>();
  req->start_region = "small1";
  req->goal_region = "no_such_region";
  auto client = node_->create_client<semantic_navigation_msgs::srv::GetRegionRoute>(
    "get_region_route");
  ASSERT_TRUE(client->wait_for_service());

  auto result = client->async_send_request(req);
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRegionRoute::Response>();
  while (rclcpp::ok() &&
    result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    executor_->spin_some();
  }
  if (result.valid()) {
    resp = result.get();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  EXPECT_FALSE(resp->success);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
