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
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
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

  polygon_msgs::msg::Polygon2DCollection createPolygons(
    std::vector<semantic_navigation::Region> list)
  {
    return SemanticNavigationTasks::createPolygons(list);
  }

  visualization_msgs::msg::MarkerArray createNames(std::vector<semantic_navigation::Region> list)
  {
    return SemanticNavigationTasks::createNames(list);
  }

  void getConnectionsFromFile(
    const std::string & filename, std::vector<semantic_navigation::Connection> & add,
    std::vector<semantic_navigation::Connection> & remove)
  {
    return SemanticNavigationTasks::getConnectionsFromFile(filename, add, remove);
  }

  visualization_msgs::msg::MarkerArray createEdges(
    std::vector<semantic_navigation::Region> list, const semantic_navigation::RegionGraph & graph)
  {
    return SemanticNavigationTasks::createEdges(list, graph);
  }

  semantic_navigation::CellLimits processBoundingBox(
    const nav_msgs::msg::OccupancyGrid & map, semantic_navigation::Region region)
  {
    return SemanticNavigationTasks::processBoundingBox(map, region);
  }

  int8_t cell(unsigned int x, unsigned int y)
  {
    return SemanticNavigationTasks::cell(x, y);
  }

  bool inCollision(int x, int y)
  {
    return SemanticNavigationTasks::inCollision(x, y);
  }

  bool isPointValid(int x, int y, semantic_navigation::Region region, geometry_msgs::msg::Pose pose)
  {
    return SemanticNavigationTasks::isPointValid(x, y, region, pose);
  }

  void orientationFromRequest(
    geometry_msgs::msg::Pose & pose, const semantic_navigation::Region & region,
    std::string orientation, double requested_yaw)
  {
    return SemanticNavigationTasks::orientationFromRequest(
      pose, region, orientation, requested_yaw);
  }

  std::vector<geometry_msgs::msg::PoseStamped> generateRandomGoals(
    unsigned int n, semantic_navigation::Region region, semantic_navigation::CellLimits limits,
    std::string orientation, double requested_yaw)
  {
    return SemanticNavigationTasks::generateRandomGoals(
      n, region, limits, orientation, requested_yaw);
  }

  nav_msgs::msg::OccupancyGrid getMap()
  {
    return goal_sampler_.getMap();
  }

  void setMap(nav_msgs::msg::OccupancyGrid map) {goal_sampler_.setMap(map);}
  void setIsCostmap(bool is_costmap) {goal_sampler_.setIsCostmap(is_costmap);}
  void setFullMap(bool full_map) {full_map_ = full_map;}
  void setInflationRadius(double inflation_radius)
  {
    goal_sampler_.setInflationRadius(inflation_radius);
  }
  void setInflatedFootprintSize(double inflation_radius, double resolution)
  {
    goal_sampler_.setInflatedFootprintSize(inflation_radius, resolution);
  }
  void setBorder(float border) {border_ = border;}
  void setRegionList(std::vector<semantic_navigation::Region> list) {region_list_ = list;}

  void getRandomRegionService(
    std::shared_ptr<semantic_navigation_msgs::srv::GetRandomRegion::Request> request,
    std::shared_ptr<semantic_navigation_msgs::srv::GetRandomRegion::Response> response)
  {
    SemanticNavigationTasks::getRandomRegionService(request, response);
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

TEST(SemanticNavigationTasksTest, configure) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set an empty regions filename config parameter
  nav2_util::declare_parameter_if_not_declared(
    node, "regions_filename", rclcpp::ParameterValue(""));

  // Configure the node
  node->configure();
  node->activate();

  // Check results: the node should be in the unconfigured state as filename is empty
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Now, set a not valid regions filename
  std::string pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  node->set_parameter(rclcpp::Parameter("regions_filename", pkg + "regions_test_empty.yaml"));

  // Configure the node
  node->configure();
  node->activate();

  // Check results: the node should be in the unconfigured state as filename is not valid
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // New, set a valid regions filename
  node->set_parameter(rclcpp::Parameter("regions_filename", pkg + "/test/regions_test.yaml"));

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

  // Load the regions from a file with right and wrong regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/regions_test.yaml";

  // Get the regions
  std::vector<semantic_navigation::Region> regions;
  bool result = node->getRegionsFromFile(filename, regions);

  // Check the results
  EXPECT_TRUE(result);
  EXPECT_EQ(regions.size(), 4);
  EXPECT_EQ(regions[0].name, "small1");
  EXPECT_EQ(regions[0].size(), 4);
  EXPECT_EQ(regions[0].polygon.points[0].x, 0.0);
  EXPECT_EQ(regions[0].polygon.points[0].y, 0.0);
  EXPECT_EQ(regions[0].polygon.points[1].x, 0.0);
  EXPECT_EQ(regions[0].polygon.points[1].y, 1.0);
  EXPECT_EQ(regions[0].polygon.points[2].x, 1.0);
  EXPECT_EQ(regions[0].polygon.points[2].y, 1.0);
  EXPECT_EQ(regions[0].polygon.points[3].x, 1.0);
  EXPECT_EQ(regions[0].polygon.points[3].y, 0.0);
  EXPECT_EQ(regions[1].name, "small2");
  EXPECT_EQ(regions[1].size(), 4);
  EXPECT_EQ(regions[1].polygon.points[0].x, 1.0);
  EXPECT_EQ(regions[1].polygon.points[0].y, 1.0);
  EXPECT_EQ(regions[1].polygon.points[1].x, 1.0);
  EXPECT_EQ(regions[1].polygon.points[1].y, 2.0);
  EXPECT_EQ(regions[1].polygon.points[2].x, 2.0);
  EXPECT_EQ(regions[1].polygon.points[2].y, 2.0);
  EXPECT_EQ(regions[1].polygon.points[3].x, 2.0);
  EXPECT_EQ(regions[1].polygon.points[3].y, 1.0);
  EXPECT_EQ(regions[2].name, "big");
  EXPECT_EQ(regions[2].size(), 4);
  EXPECT_EQ(regions[2].polygon.points[0].x, 12.0);
  EXPECT_EQ(regions[2].polygon.points[0].y, 12.0);
  EXPECT_EQ(regions[2].polygon.points[1].x, 12.0);
  EXPECT_EQ(regions[2].polygon.points[1].y, 20.0);
  EXPECT_EQ(regions[2].polygon.points[2].x, 20.0);
  EXPECT_EQ(regions[2].polygon.points[2].y, 20.0);
  EXPECT_EQ(regions[2].polygon.points[3].x, 20.0);
  EXPECT_EQ(regions[2].polygon.points[3].y, 10.0);
  EXPECT_EQ(regions[3].name, "outside");
  EXPECT_EQ(regions[3].size(), 4);
  EXPECT_EQ(regions[3].polygon.points[0].x, -12.0);
  EXPECT_EQ(regions[3].polygon.points[0].y, -12.0);
  EXPECT_EQ(regions[3].polygon.points[1].x, -12.0);
  EXPECT_EQ(regions[3].polygon.points[1].y, -20.0);
  EXPECT_EQ(regions[3].polygon.points[2].x, -20.0);
  EXPECT_EQ(regions[3].polygon.points[2].y, -20.0);
  EXPECT_EQ(regions[3].polygon.points[3].x, -20.0);
  EXPECT_EQ(regions[3].polygon.points[3].y, -12.0);

  // Now try to get the regions from a file with empty regions
  EXPECT_FALSE(node->getRegionsFromFile(pkg + "/test/regions_test_empty.yaml", regions));
}

TEST(SemanticNavigationTasksTest, createPolygons) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();
  node->configure();
  node->activate();

  // Create the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/regions_test.yaml";
  std::vector<semantic_navigation::Region> regions;
  node->getRegionsFromFile(filename, regions);

  // Create the polygons
  node->createFreeMap(10, 10, 0.5);
  auto polygons = node->createPolygons(regions);

  // Check the results
  EXPECT_DOUBLE_EQ(polygons.polygons.size(), 4);
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
  std::string filename = pkg + "/test/regions_test.yaml";
  std::vector<semantic_navigation::Region> regions;
  node->getRegionsFromFile(filename, regions);

  // Create the names
  node->createFreeMap(10, 10, 0.5);
  auto names = node->createNames(regions);

  // Check the results
  EXPECT_EQ(names.markers.size(), 4);
  EXPECT_EQ(names.markers[0].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(names.markers[0].text, "small1");
  EXPECT_DOUBLE_EQ(names.markers[0].pose.position.x, regions[0].centroid().x);
  EXPECT_DOUBLE_EQ(names.markers[0].pose.position.y, regions[0].centroid().y);
  EXPECT_EQ(names.markers[1].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(names.markers[1].text, "small2");
  EXPECT_DOUBLE_EQ(names.markers[1].pose.position.x, regions[1].centroid().x);
  EXPECT_DOUBLE_EQ(names.markers[1].pose.position.y, regions[1].centroid().y);

  // Clean up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, getConnectionsFromFile) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  std::string pkg_path =
    ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg_path + "/test/regions_test.yaml";

  // Read the connections section
  std::vector<semantic_navigation::Connection> add, remove;
  node->getConnectionsFromFile(filename, add, remove);

  // Check the results
  ASSERT_EQ(add.size(), 1u);
  EXPECT_EQ(add[0].first, "small1");
  EXPECT_EQ(add[0].second, "big");
  ASSERT_EQ(remove.size(), 1u);
  EXPECT_EQ(remove[0].first, "small1");
  EXPECT_EQ(remove[0].second, "small2");

  // A file without a connections section leaves the lists empty
  std::vector<semantic_navigation::Connection> add_empty, remove_empty;
  node->getConnectionsFromFile(
    pkg_path + "/test/regions_test_empty.yaml", add_empty, remove_empty);
  EXPECT_TRUE(add_empty.empty());
  EXPECT_TRUE(remove_empty.empty());
}

TEST(SemanticNavigationTasksTest, createEdges) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();
  node->configure();
  node->activate();

  // Build two adjacent regions and an isolated one
  std::vector<semantic_navigation::Region> regions;
  std::string pkg_path =
    ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  node->getRegionsFromFile(pkg_path + "/test/regions_test.yaml", regions);

  // Connect small1 -- small2 in the graph
  semantic_navigation::RegionGraph graph;
  graph.addEdge("small1", "small2");

  // Create the edges markers
  auto edges = node->createEdges(regions, graph);

  // Check the results: a single line list marker with one segment (two points)
  ASSERT_EQ(edges.markers.size(), 1u);
  EXPECT_EQ(edges.markers[0].type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(edges.markers[0].points.size(), 2u);

  // Clean up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SemanticNavigationTasksTest, processBoundingBoxEmptyRegion) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create a map of 10x10 cells
  node->createFreeMap(10, 10, 0.5);

  // Process the bounding box
  auto [cell_min_x, cell_max_x, cell_min_y, cell_max_y] = node->processBoundingBox(
    node->getMap(), semantic_navigation::Region());

  // Check the results
  EXPECT_EQ(cell_min_x, 0);
  EXPECT_EQ(cell_max_x, 10);
  EXPECT_EQ(cell_min_y, 0);
  EXPECT_EQ(cell_max_y, 10);
}

TEST(SemanticNavigationTasksTest, processBoundingBox) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Set the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/regions_test.yaml";

  // Get the regions
  std::vector<semantic_navigation::Region> regions;
  node->getRegionsFromFile(filename, regions);

  // Create a map of 10x10 cells
  node->createFreeMap(10, 10, 0.5);

  // Process the bounding box for a 1x1 square region
  auto [cell_min_x, cell_max_x, cell_min_y, cell_max_y] = node->processBoundingBox(
    node->getMap(), regions[0]);
  // Check the results: it should be a 1x1 square
  EXPECT_EQ(cell_min_x, 0);
  EXPECT_EQ(cell_max_x, 2);
  EXPECT_EQ(cell_min_y, 0);
  EXPECT_EQ(cell_max_y, 2);

  // Process the bounding box for a region bigger than the map
  std::tie(cell_min_x, cell_max_x, cell_min_y, cell_max_y) = node->processBoundingBox(
    node->getMap(), regions[2]);
  // Check the results: it should be the whole map
  EXPECT_EQ(cell_min_x, 10);
  EXPECT_EQ(cell_max_x, 10);
  EXPECT_EQ(cell_min_y, 10);
  EXPECT_EQ(cell_max_y, 10);

  // Process the bounding box for a region outside the map
  std::tie(cell_min_x, cell_max_x, cell_min_y, cell_max_y) = node->processBoundingBox(
    node->getMap(), regions[3]);
  // Check the results: it should be zero
  EXPECT_EQ(cell_min_x, 0);
  EXPECT_EQ(cell_max_x, 0);
  EXPECT_EQ(cell_min_y, 0);
  EXPECT_EQ(cell_max_y, 0);
}

TEST(SemanticNavigationTasksTest, cellCheck) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create a map of 10x10 cells
  node->createFreeMap(10, 10, 0.5);

  // Check the cells inside the map
  EXPECT_EQ(node->cell(2, 2), nav2_util::OCC_GRID_FREE);
  EXPECT_EQ(node->cell(5, 5), nav2_util::OCC_GRID_FREE);
  // Check the cells in the limits of the map (valid indices are [0, 9] for a 10x10 map)
  EXPECT_EQ(node->cell(0, 0), nav2_util::OCC_GRID_FREE);
  EXPECT_EQ(node->cell(9, 9), nav2_util::OCC_GRID_FREE);
  // Check the cells just outside the limits of the map
  EXPECT_EQ(node->cell(10, 10), nav2_util::OCC_GRID_UNKNOWN);
  EXPECT_EQ(node->cell(0, 13), nav2_util::OCC_GRID_UNKNOWN);
  EXPECT_EQ(node->cell(13, 0), nav2_util::OCC_GRID_UNKNOWN);
  EXPECT_EQ(node->cell(13, 13), nav2_util::OCC_GRID_UNKNOWN);
}

TEST(SemanticNavigationTasksTest, inCollision) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create a map of 10x10 cells
  node->createFreeMap(10, 10, 0.5);
  // Set the map
  node->setIsCostmap(false);
  node->setFullMap(false);
  node->setInflationRadius(0.5);
  node->setInflatedFootprintSize(0.5, 0.5);

  // Check the points in the limits of the map
  EXPECT_TRUE(node->inCollision(0, 0));
  EXPECT_TRUE(node->inCollision(10, 10));
  // Check the points inside the map
  EXPECT_FALSE(node->inCollision(2, 2));
  EXPECT_FALSE(node->inCollision(8, 8));

  // Now set a cell as occupied
  auto map = node->getMap();
  map.data[0] = nav2_util::OCC_GRID_OCCUPIED;
  node->setMap(map);
  // Check the results
  EXPECT_TRUE(node->inCollision(0, 0));
}

TEST(SemanticNavigationTasksTest, inCollisionCostmap) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create a map of 10x10 cells
  node->createFreeMap(10, 10, 0.5);
  // Set the map
  node->setIsCostmap(true);
  node->setFullMap(false);
  node->setInflationRadius(0.5);
  node->setInflatedFootprintSize(0.5, 0.5);

  // Check if the point is in collision
  EXPECT_FALSE(node->inCollision(0, 0));
  EXPECT_FALSE(node->inCollision(10, 10));
  EXPECT_FALSE(node->inCollision(2, 2));
  EXPECT_FALSE(node->inCollision(8, 8));

  // Set a cell as occupied
  auto map = node->getMap();
  map.data[0] = nav2_util::OCC_GRID_OCCUPIED;
  node->setMap(map);
  // Check if the point is in collision
  EXPECT_TRUE(node->inCollision(0, 0));
}

TEST(SemanticNavigationTasksTest, isPointValid) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/regions_test.yaml";
  std::vector<semantic_navigation::Region> regions;
  node->getRegionsFromFile(filename, regions);

  // Create a map of 10x10 cells
  node->createFreeMap(10, 10, 0.5);
  auto map = node->getMap();
  // Set map related values
  node->setIsCostmap(false);
  node->setFullMap(false);
  node->setInflationRadius(0);
  node->setInflatedFootprintSize(0, 0.5);

  // Set the border
  double border = 0.0;
  node->setBorder(border);
  // Set a pose (0.5, 0.5) inside the first region
  int cell_x = 1; int cell_y = 1;
  geometry_msgs::msg::Pose pose;
  pose.position.x = map.info.origin.position.x + cell_x * map.info.resolution;
  pose.position.y = map.info.origin.position.y + cell_y * map.info.resolution;

  // Check the results
  EXPECT_TRUE(regions[0].isPointInside(pose.position.x, pose.position.y));
  EXPECT_TRUE(
    regions[0].isPointAtLeastDistanceFromBorders(pose.position.x, pose.position.y, border));
  EXPECT_TRUE(!node->inCollision(cell_x, cell_y));
  EXPECT_TRUE(node->isPointValid(cell_x, cell_y, regions[0], pose));

  // Set a new cell (50, 50) outside the first region
  cell_x = 50; cell_y = 50;
  pose.position.x = map.info.origin.position.x + cell_x * map.info.resolution;
  pose.position.y = map.info.origin.position.y + cell_y * map.info.resolution;
  // Check the result
  EXPECT_FALSE(regions[0].isPointInside(pose.position.x, pose.position.y));
  EXPECT_TRUE(
    regions[0].isPointAtLeastDistanceFromBorders(pose.position.x, pose.position.y, border));
  EXPECT_FALSE(!node->inCollision(cell_x, cell_y));
  EXPECT_FALSE(node->isPointValid(cell_x, cell_y, regions[0], pose));

  // Set a pose (0.5, 0.5) inside the first region but in collision
  cell_x = 1; cell_y = 1;
  pose.position.x = map.info.origin.position.x + cell_x * map.info.resolution;
  pose.position.y = map.info.origin.position.y + cell_y * map.info.resolution;
  // Set the cell as occupied
  map.data[0] = nav2_util::OCC_GRID_OCCUPIED;
  node->setMap(map);
  // Check the result
  EXPECT_TRUE(regions[0].isPointInside(pose.position.x, pose.position.y));
  EXPECT_TRUE(
    regions[0].isPointAtLeastDistanceFromBorders(pose.position.x, pose.position.y, border));
  EXPECT_FALSE(!node->inCollision(cell_x, cell_y));
  EXPECT_FALSE(node->isPointValid(cell_x, cell_y, regions[0], pose));
}

TEST(SemanticNavigationTasksTest, orientationFromRequest) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/regions_test.yaml";
  std::vector<semantic_navigation::Region> regions;
  node->getRegionsFromFile(filename, regions);

  // Set the pose
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  pose.orientation.w = 1.0;

  // Request the orientation outside the region
  node->orientationFromRequest(
    pose, regions[0], semantic_navigation_msgs::srv::GenerateRandomGoals::Request::OUTSIDE, 0.0);
  // Check the results
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 0.0);

  // Request the orientation inside the region
  node->orientationFromRequest(
    pose, regions[0], semantic_navigation_msgs::srv::GenerateRandomGoals::Request::INSIDE, 0.0);
  // Check the results
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), M_PI);

  // Request the orientation to a specific yaw
  node->orientationFromRequest(
    pose, regions[0], semantic_navigation_msgs::srv::GenerateRandomGoals::Request::REQUESTED, 1.0);
  // Check the results
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 1.0);

  // Request a random orientation: the yaw already sampled by the caller must be kept
  pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.75));
  node->orientationFromRequest(
    pose, regions[0], semantic_navigation_msgs::srv::GenerateRandomGoals::Request::RANDOM, 0.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 0.75);

  // An empty orientation behaves the same as random
  node->orientationFromRequest(pose, regions[0], "", 0.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 0.75);
}

TEST(SemanticNavigationTasksTest, generateRandomGoals) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/regions_test.yaml";
  std::vector<semantic_navigation::Region> regions;
  node->getRegionsFromFile(filename, regions);

  // Create the names
  node->createFreeMap(10, 10, 0.5);

  // Process the bounding box for a 1x1 square region
  auto limits = node->processBoundingBox(node->getMap(), regions[0]);

  // Generate random goals with a requested orientation: it must be propagated to the goal
  auto goals = node->generateRandomGoals(
    1, regions[0], limits, semantic_navigation_msgs::srv::GenerateRandomGoals::Request::REQUESTED,
    1.0);

  // Check the results
  EXPECT_EQ(goals.size(), 1);
  EXPECT_DOUBLE_EQ(tf2::getYaw(goals[0].pose.orientation), 1.0);
}

TEST(SemanticNavigationTasksTest, generateRandomGoalsUnreachableRegionTerminates) {
  // Create the node
  auto node = std::make_shared<SemanticNavigationTasksFixture>();

  // Create the regions
  auto pkg = ament_index_cpp::get_package_share_directory("semantic_navigation_tasks");
  std::string filename = pkg + "/test/regions_test.yaml";
  std::vector<semantic_navigation::Region> regions;
  node->getRegionsFromFile(filename, regions);

  // Create a fully occupied map: no cell can ever be a valid goal
  node->createFreeMap(10, 10, 0.5);
  auto map = node->getMap();
  map.data = std::vector<int8_t>(map.data.size(), nav2_util::OCC_GRID_OCCUPIED);
  node->setMap(map);
  node->setIsCostmap(false);
  node->setInflationRadius(0.0);
  node->setInflatedFootprintSize(0.0, 0.5);
  node->setBorder(0.0);

  auto limits = node->processBoundingBox(node->getMap(), regions[0]);

  // Before the attempts cap, this call would loop forever. It must now return promptly with
  // fewer goals than requested instead of hanging.
  auto goals = node->generateRandomGoals(
    5, regions[0], limits, semantic_navigation_msgs::srv::GenerateRandomGoals::Request::INSIDE,
    0.0);

  EXPECT_LT(goals.size(), 5u);
}

TEST(SemanticNavigationTasksTest, getRandomRegionServiceEmptyList) {
  // Create the node with an empty region list
  auto node = std::make_shared<SemanticNavigationTasksFixture>();
  node->setRegionList({});

  auto request = std::make_shared<semantic_navigation_msgs::srv::GetRandomRegion::Request>();
  auto response = std::make_shared<semantic_navigation_msgs::srv::GetRandomRegion::Response>();
  node->getRandomRegionService(request, response);

  // Check the results: an empty region list is a rejected request, not an out-of-bounds read
  EXPECT_FALSE(response->success);
  EXPECT_EQ(
    response->region_name, semantic_navigation_msgs::srv::GetRandomRegion::Response::UNKNOWN);
}

TEST(SemanticNavigationTasksTest, getRandomRegionServiceSingleRegion) {
  // Create the node with a single region
  auto node = std::make_shared<SemanticNavigationTasksFixture>();
  semantic_navigation::Region region;
  region.name = "only_region";
  polygon_msgs::msg::Point2D point;
  point.x = 0.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 1.0;
  region.polygon.points.push_back(point);
  node->setRegionList({region});

  auto request = std::make_shared<semantic_navigation_msgs::srv::GetRandomRegion::Request>();
  auto response = std::make_shared<semantic_navigation_msgs::srv::GetRandomRegion::Response>();
  node->getRandomRegionService(request, response);

  // Check the results: the only region must be selected deterministically
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->region_name, "only_region");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
