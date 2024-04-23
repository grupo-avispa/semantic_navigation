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
#include "semantic_navigation_tasks/task_services.hpp"

class SemanticNavigationTasksFixture : public semantic_navigation::SemanticNavigationTasks
{
public:
  SemanticNavigationTasksFixture()
  : SemanticNavigationTasks() {}

  bool getRegionsFromFile(
    const std::string & filename, std::vector<semantic_navigation::ROI> & regions)
  {
    return SemanticNavigationTasks::getRegionsFromFile(filename, regions);
  }

  polygon_msgs::msg::Polygon2DCollection createPolygons(std::vector<semantic_navigation::ROI> list)
  {
    return SemanticNavigationTasks::createPolygons(list);
  }

  visualization_msgs::msg::MarkerArray createNames(std::vector<semantic_navigation::ROI> list)
  {
    return SemanticNavigationTasks::createNames(list);
  }
};

TEST(SemanticNavigationTasksTest, configure) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set an empty rois filename config parameter
  nav2_util::declare_parameter_if_not_declared(node, "rois_filename", rclcpp::ParameterValue(""));

  // Configure the node
  node->configure();
  node->activate();

  // Check results: the node should be in the unconfigured state as filename is empty
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // New, we set the rois filename
  std::string pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  node->set_parameter(rclcpp::Parameter("rois_filename", pkg + "/test/test_rois.yaml"));

  // Configure the node
  node->configure();
  node->activate();

  // Check results: the node should be in the active state
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Check the default parameters
  EXPECT_EQ(node->get_parameter("is_costmap").as_bool(), false);
  EXPECT_EQ(node->get_parameter("full_map").as_bool(), false);
  EXPECT_EQ(node->get_parameter("inflation_radius").as_double(), 0.5);
  EXPECT_EQ(node->get_parameter("goals_topic").as_string(), "semantic_goals");
  EXPECT_EQ(node->get_parameter("polygons_topic").as_string(), "polygons");
  EXPECT_EQ(node->get_parameter("names_topic").as_string(), "names");
  EXPECT_EQ(node->get_parameter("map_topic").as_string(), "map");

  // Clean up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, getRegionsFromFile) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set the rois
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/test_rois.yaml";

  // Get the regions
  std::vector<semantic_navigation::ROI> regions;
  bool result = node->getRegionsFromFile(filename, regions);

  // Check the results
  EXPECT_TRUE(result);
  EXPECT_EQ(regions.size(), 2);
  EXPECT_EQ(regions[0].get_name(), "roi_0");
  EXPECT_EQ(regions[0].polygon.size(), 4);
  EXPECT_DOUBLE_EQ(regions[0].yaw, 0.0);
  EXPECT_EQ(regions[1].get_name(), "roi_1");
  EXPECT_EQ(regions[1].polygon.size(), 4);
  EXPECT_DOUBLE_EQ(regions[1].yaw, 1.5);
}

TEST(SemanticNavigationTasksTest, createPolygons) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();
  node->configure();
  node->activate();

  // Create the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/test_rois.yaml";
  std::vector<semantic_navigation::ROI> regions;
  node->getRegionsFromFile(filename, regions);

  // Create the polygons
  auto polygons = node->createPolygons(regions);

  // Check the results
  EXPECT_DOUBLE_EQ(polygons.polygons.size(), 2);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points.size(), 4);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[0].y, 0.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[1].x, 0.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[1].y, 1.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[2].x, 1.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[2].y, 1.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[3].x, 1.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[0].points[3].y, 0.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points.size(), 4);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points[0].y, 1.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points[1].x, 1.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points[1].y, 2.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points[2].x, 2.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points[2].y, 2.0);
  EXPECT_DOUBLE_EQ(polygons.polygons[1].points[3].x, 2.0);

  // Clean up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, createNames) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();
  node->configure();
  node->activate();

  // Create the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/test_rois.yaml";
  std::vector<semantic_navigation::ROI> regions;
  node->getRegionsFromFile(filename, regions);

  // Create the names
  auto names = node->createNames(regions);

  // Check the results
  EXPECT_EQ(names.markers.size(), 2);
  EXPECT_EQ(names.markers[0].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(names.markers[0].text, "roi_0");
  EXPECT_EQ(names.markers[1].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(names.markers[1].text, "roi_1");

  // Clean up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, generateRandomGoals) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set the test rois filename config parameter
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  nav2_util::declare_parameter_if_not_declared(
    node, "rois_filename", rclcpp::ParameterValue(pkg + "/test/test_rois.yaml"));

  // Configure
  node->configure();
  node->activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Request>();
  req->n = 1;
  req->region_name = "roi_0";
  auto client = node->create_client<semantic_navigation_msgs::srv::GenerateRandomGoals>(
    "generate_random_goals");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GenerateRandomGoals::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, getRegionNameInside) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set the test rois filename config parameter
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  nav2_util::declare_parameter_if_not_declared(
    node, "rois_filename", rclcpp::ParameterValue(pkg + "/test/test_rois.yaml"));

  // Configure
  node->configure();
  node->activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Request>();
  req->position.x = 0.5;
  req->position.y = 0.5;
  auto client = node->create_client<semantic_navigation_msgs::srv::GetRegionName>(
    "get_region_name");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Check results
  EXPECT_EQ(resp->region_name, "roi_0");

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, getRegionNameOutside) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set the test rois filename config parameter
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  nav2_util::declare_parameter_if_not_declared(
    node, "rois_filename", rclcpp::ParameterValue(pkg + "/test/test_rois.yaml"));

  // Configure
  node->configure();
  node->activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Request>();
  req->position.x = -0.5;
  req->position.y = -0.5;
  auto client = node->create_client<semantic_navigation_msgs::srv::GetRegionName>(
    "get_region_name");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Check results
  EXPECT_EQ(resp->region_name, "unknown");

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, listAllRegions) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set the test rois filename config parameter
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  nav2_util::declare_parameter_if_not_declared(
    node, "rois_filename", rclcpp::ParameterValue(pkg + "/test/test_rois.yaml"));

  // Configure
  node->configure();
  node->activate();

  // Create the client service
  auto req = std::make_shared<semantic_navigation_msgs::srv::ListAllRegions::Request>();
  auto client = node->create_client<semantic_navigation_msgs::srv::ListAllRegions>(
    "list_all_regions");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<semantic_navigation_msgs::srv::ListAllRegions::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Check results
  EXPECT_EQ(resp->region_names.size(), 2);

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
