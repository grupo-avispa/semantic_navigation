// Copyright (c) 2020 Alberto J. Tudela Roldán
// Copyright (c) 2020 Grupo Avispa, DTE, Universidad de Málaga
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

#include <yaml-cpp/yaml.h>
#include <limits>

// ROS
#include "angles/angles.h"
#include "tf2/utils.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Semantic Goals
#include "semantic_navigation_tasks/task_services.hpp"

namespace semantic_navigation
{

using std::placeholders::_1, std::placeholders::_2;

SemanticNavigationTasks::SemanticNavigationTasks(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("semantic_navigation_tasks", "", options),
  border_(0.0)
{
  RCLCPP_INFO(get_logger(), "Creating Semantic Navigation Tasks");
}

nav2_util::CallbackReturn SemanticNavigationTasks::on_configure(const rclcpp_lifecycle::State &)
{
  // BOOLEAN PARAMS ..........................................................................
  nav2_util::declare_parameter_if_not_declared(
    this, "is_costmap",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Is the map a costmap?"));
  this->get_parameter("is_costmap", is_costmap_);
  RCLCPP_INFO(
    get_logger(), "The parameter is_costmap is set to: [%s]", is_costmap_ ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    this, "full_map",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Use the full map as Region?"));
  this->get_parameter("full_map", full_map_);
  RCLCPP_INFO(
    get_logger(), "The parameter full_map is set to: [%s]", full_map_ ? "true" : "false");

  nav2::declare_parameter_if_not_declared(
    this, "auto_connect",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Detect connectivity between regions automatically from their geometry?"));
  this->get_parameter("auto_connect", auto_connect_);
  RCLCPP_INFO(
    get_logger(), "The parameter auto_connect is set to: [%s]", auto_connect_ ? "true" : "false");

  // FLOAT PARAMS ..........................................................................
  nav2_util::declare_parameter_if_not_declared(
    this, "inflation_radius",
    rclcpp::ParameterValue(0.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Inflation radius for the robot footprint"));
  this->get_parameter("inflation_radius", inflation_radius_);
  RCLCPP_INFO(
    get_logger(), "The parameter inflation_radius is set to: [%f]", inflation_radius_);

  nav2_util::declare_parameter_if_not_declared(
    this, "transform_tolerance",
    rclcpp::ParameterValue(0.2), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Transform tolerance for TF2"));
  this->get_parameter("transform_tolerance", transform_tolerance_);
  RCLCPP_INFO(
    get_logger(), "The parameter transform_tolerance is set to: [%f]", transform_tolerance_);

  nav2::declare_parameter_if_not_declared(
    this, "connectivity_threshold",
    rclcpp::ParameterValue(0.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum distance between two region borders to consider them connected"));
  this->get_parameter("connectivity_threshold", connectivity_threshold_);
  RCLCPP_INFO(
    get_logger(), "The parameter connectivity_threshold is set to: [%f]",
    connectivity_threshold_);

  // STRING PARAMS ..........................................................................
  nav2_util::declare_parameter_if_not_declared(
    this, "goals_topic",
    rclcpp::ParameterValue("semantic_goals"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Name of the publisher for the goals"));
  this->get_parameter("goals_topic", goals_topic_);
  RCLCPP_INFO(
    get_logger(), "The parameter goals_topic is set to: [%s]", goals_topic_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    this, "polygons_topic",
    rclcpp::ParameterValue("polygons"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Name of the publisher for the regions"));
  this->get_parameter("polygons_topic", polygons_topic_);
  RCLCPP_INFO(
    get_logger(), "The parameter polygons_topic is set to: [%s]", polygons_topic_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    this, "names_topic",
    rclcpp::ParameterValue("names"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Name of the publisher for the names for the visualization of the regions"));
  this->get_parameter("names_topic", names_topic_);
  RCLCPP_INFO(
    get_logger(), "The parameter names_topic is set to: [%s]", names_topic_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    this, "edges_topic",
    rclcpp::ParameterValue("edges"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Name of the publisher for the connectivity edges between regions"));
  this->get_parameter("edges_topic", edges_topic_);
  RCLCPP_INFO(
    get_logger(), "The parameter edges_topic is set to: [%s]", edges_topic_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    this, "map_topic",
    rclcpp::ParameterValue("map"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Name of the map topic"));
  this->get_parameter("map_topic", map_topic_);
  RCLCPP_INFO(
    get_logger(), "The parameter map_topic is set to: [%s]", map_topic_.c_str());

  std::string regions_filename;
  nav2_util::declare_parameter_if_not_declared(
    this, "regions_filename",
    rclcpp::ParameterValue("regions.yaml"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("File where the regions are stored"));
  this->get_parameter("regions_filename", regions_filename);
  RCLCPP_INFO(
    get_logger(), "The parameter regions_filename is set to: [%s]", regions_filename.c_str());

  if (!getRegionsFromFile(regions_filename, region_list_)) {
    RCLCPP_ERROR(get_logger(), "The list of regions could not be found");
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Build the connectivity graph between the regions
  std::vector<Connection> add_edges, remove_edges;
  getConnectionsFromFile(regions_filename, add_edges, remove_edges);
  region_graph_.build(
    region_list_, connectivity_threshold_, auto_connect_, add_edges, remove_edges);
  RCLCPP_INFO(
    get_logger(), "The connectivity graph has %zu edges", region_graph_.edgeCount());

  // Publishers
  rclcpp::QoS latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  goals_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    goals_topic_, latched_profile);
  polygons_viz_pub_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>(
    polygons_topic_, latched_profile);
  names_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    names_topic_, latched_profile);
  edges_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    edges_topic_, latched_profile);

  // Subscribers
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, latched_profile, std::bind(&SemanticNavigationTasks::mapCallback, this, _1));

  // Services
  goals_generator_service_ = this->create_service<GenerateRandomGoals>(
    "generate_random_goals",
    std::bind(&SemanticNavigationTasks::generateRandomGoalsService, this, _1, _2));
  get_random_region_service_ = this->create_service<GetRandomRegion>(
    "get_random_region",
    std::bind(&SemanticNavigationTasks::getRandomRegionService, this, _1, _2));
  get_region_name_service_ = this->create_service<GetRegionName>(
    "get_region_name",
    std::bind(&SemanticNavigationTasks::getRegionNameService, this, _1, _2));
  list_all_regions_service_ = this->create_service<ListAllRegions>(
    "list_all_regions",
    std::bind(&SemanticNavigationTasks::listAllRegionsService, this, _1, _2, _3));
  get_adjacent_regions_service_ = this->create_service<GetAdjacentRegions>(
    "get_adjacent_regions",
    std::bind(&SemanticNavigationTasks::getAdjacentRegionsService, this, _1, _2, _3));
  are_regions_connected_service_ = this->create_service<AreRegionsConnected>(
    "are_regions_connected",
    std::bind(&SemanticNavigationTasks::areRegionsConnectedService, this, _1, _2, _3));
  get_region_route_service_ = this->create_service<GetRegionRoute>(
    "get_region_route",
    std::bind(&SemanticNavigationTasks::getRegionRouteService, this, _1, _2, _3));

  // TF Buffer and Listener
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_, this, true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SemanticNavigationTasks::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  goals_pub_->on_activate();
  polygons_viz_pub_->on_activate();
  names_viz_pub_->on_activate();
  edges_viz_pub_->on_activate();

  // Publish polygons, names and connectivity edges
  polygons_viz_pub_->publish(createPolygons(region_list_));
  names_viz_pub_->publish(createNames(region_list_));
  edges_viz_pub_->publish(createEdges(region_list_, region_graph_));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SemanticNavigationTasks::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  goals_pub_->on_deactivate();
  polygons_viz_pub_->on_deactivate();
  names_viz_pub_->on_deactivate();
  edges_viz_pub_->on_deactivate();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SemanticNavigationTasks::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  goals_pub_.reset();
  polygons_viz_pub_.reset();
  names_viz_pub_.reset();
  edges_viz_pub_.reset();
  tf2_listener_.reset();
  tf2_buffer_.reset();
  goals_generator_service_.reset();
  get_random_region_service_.reset();
  get_region_name_service_.reset();
  list_all_regions_service_.reset();
  get_adjacent_regions_service_.reset();
  are_regions_connected_service_.reset();
  get_region_route_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SemanticNavigationTasks::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

bool SemanticNavigationTasks::getRegionsFromFile(
  const std::string & filename, std::vector<semantic_navigation::Region> & regions)
{
  RCLCPP_INFO(get_logger(), "Reading regions from file: %s", filename.c_str());
  bool success = false;
  try {
    YAML::Node config = YAML::LoadFile(filename);
    // Get the list of regions
    if (config["regions"]) {
      for (const auto & region : config["regions"]) {
        // Extract name and points
        if (region["name"] && region["points"]) {
          Region new_region;
          new_region.name = region["name"].as<std::string>();
          for (const auto & point : region["points"]) {
            polygon_msgs::msg::Point2D new_point;
            new_point.x = point[0].as<float>();
            new_point.y = point[1].as<float>();
            new_region.polygon.points.push_back(new_point);
          }
          regions.push_back(new_region);
          success = true;
        } else {
          RCLCPP_ERROR(get_logger(), "Region is not well defined");
        }
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No regions found in file [%s]", filename.c_str());
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error reading file [%s]", filename.c_str());
  }
  return success;
}

void SemanticNavigationTasks::getConnectionsFromFile(
  const std::string & filename, std::vector<Connection> & add,
  std::vector<Connection> & remove)
{
  try {
    YAML::Node config = YAML::LoadFile(filename);
    // The connections section is optional
    if (!config["connections"]) {
      return;
    }

    // Lambda to parse a list of pairs of region names into a list of connections
    auto parse_edges = [this](const YAML::Node & node, std::vector<Connection> & edges) {
        for (const auto & edge : node) {
          if (edge.size() == 2) {
            edges.emplace_back(edge[0].as<std::string>(), edge[1].as<std::string>());
          } else {
            RCLCPP_ERROR(get_logger(), "A connection must be a pair of region names");
          }
        }
      };

    const auto & connections = config["connections"];
    if (connections["add"]) {
      parse_edges(connections["add"], add);
    }
    if (connections["remove"]) {
      parse_edges(connections["remove"], remove);
    }
  } catch (const YAML::Exception &) {
    RCLCPP_ERROR(get_logger(), "Error reading connections from file [%s]", filename.c_str());
  }
}

void SemanticNavigationTasks::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  RCLCPP_INFO_ONCE(
    get_logger(), "Received a %d X %d map @ %.3f m/pix",
    msg->info.width, msg->info.height, msg->info.resolution);

  map_ = *msg;

  inflated_footprint_size_ = static_cast<int>(inflation_radius_ / map_.info.resolution) + 1;
}

semantic_navigation::CellLimits SemanticNavigationTasks::processBoundingBox(
  const nav_msgs::msg::OccupancyGrid & map, Region region)
{
  int map_min_x = map.info.origin.position.x;
  int map_max_x = map.info.origin.position.x + map.info.width * map.info.resolution;
  int map_min_y = map.info.origin.position.y;
  int map_max_y = map.info.origin.position.y + map.info.height * map.info.resolution;

  // Region must lie inside the map boundaries
  // If the region is empty, the whole map is treated as the region by default
  float bbox_min_x, bbox_max_x, bbox_min_y, bbox_max_y;
  if (region.empty()) {
    bbox_min_x = map_min_x;
    bbox_max_x = map_max_x;
    bbox_min_y = map_min_y;
    bbox_max_y = map_max_y;
    RCLCPP_INFO(get_logger(), "No region specified, full map is used");
  } else {
    // If the region is outside the map, adjust to map boundaries
    // Determine bounding box of Region
    bbox_min_x = std::numeric_limits<double>::infinity();
    bbox_max_x = -std::numeric_limits<double>::infinity();
    bbox_min_y = std::numeric_limits<double>::infinity();
    bbox_max_y = -std::numeric_limits<double>::infinity();

    for (auto & p : region.polygon.points) {
      if (p.x < map_min_x) {p.x = map_min_x;}
      if (p.x > map_max_x) {p.x = map_max_x;}

      if (p.x < bbox_min_x) {bbox_min_x = p.x;}
      if (p.x > bbox_max_x) {bbox_max_x = p.x;}

      if (p.y < map_min_y) {p.y = map_min_y;}
      if (p.y > map_max_y) {p.y = map_max_y;}

      if (p.y < bbox_min_y) {bbox_min_y = p.y;}
      if (p.y > bbox_max_y) {bbox_max_y = p.y;}
    }
  }

  // Calculate bounding box for cell array
  int cell_min_x =
    static_cast<int>((bbox_min_x - map_.info.origin.position.x) / map_.info.resolution);
  int cell_max_x =
    static_cast<int>((bbox_max_x - map_.info.origin.position.x) / map_.info.resolution);
  int cell_min_y =
    static_cast<int>((bbox_min_y - map_.info.origin.position.y) / map_.info.resolution);
  int cell_max_y =
    static_cast<int>((bbox_max_y - map_.info.origin.position.y) / map_.info.resolution);

  RCLCPP_INFO(
    get_logger(), "Region bounding box (meters): (%f,%f) (%f,%f)",
    bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y);
  RCLCPP_INFO(
    get_logger(), "Region bounding box (cells): (%i,%i) (%i,%i)",
    cell_min_x, cell_min_y, cell_max_x, cell_max_y);

  return CellLimits(cell_min_x, cell_max_x, cell_min_y, cell_max_y);
}

bool SemanticNavigationTasks::generateRandomGoalsService(
  const std::shared_ptr<GenerateRandomGoals::Request> request,
  std::shared_ptr<GenerateRandomGoals::Response> response)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  Region current_region;
  RCLCPP_INFO(
    get_logger(), "Incoming goals generator service request: [%i, %s]",
    request->n, request->region_name.c_str());

  // Get arguments
  uint64_t n = request->n;
  for (const auto & region : region_list_) {
    if (region.name == request->region_name) {
      current_region = region;
      break;
    }
  }
  border_ = request->border;

  // If the requested region is empty and we don't want to use the full map
  if (current_region.empty() && !full_map_) {
    RCLCPP_FATAL(
      get_logger(), "The requested region [%s], could not be found in the list",
      request->region_name.c_str());
    return false;
  }

  // Check if we have a map
  if (map_.data.empty()) {
    RCLCPP_FATAL(get_logger(), "Failed to get map at [%s]", map_topic_.c_str());
    return false;
  }

  // Process bounding box
  auto limits = processBoundingBox(map_, current_region);

  // Generate response
  response->goals = generateRandomGoals(n, current_region, limits);

  // Publish goals
  geometry_msgs::msg::PoseArray goals_array;
  goals_array.header.frame_id = map_topic_;
  goals_array.header.stamp = this->now();
  for (const auto & goal : response->goals) {
    goals_array.poses.push_back(goal.pose);
  }
  goals_pub_->publish(goals_array);
  return true;
}

bool SemanticNavigationTasks::getRandomRegionService(
  const std::shared_ptr<GetRandomRegion::Request>/*request*/,
  std::shared_ptr<GetRandomRegion::Response> response)
{
  // Generate random region
  std::random_device rd;       // obtain a random number from hardware
  std::mt19937 gen(rd());       // seed the generator
  std::uniform_int_distribution<int> dist_region(1, region_list_.size());       // define the range

  int region_idx = dist_region(gen);
  response->region_name = region_list_[region_idx].name;

  RCLCPP_INFO(
    get_logger(), "Incoming random region service request: [%s]", response->region_name.c_str());

  return true;
}

bool SemanticNavigationTasks::getRegionNameService(
  const std::shared_ptr<GetRegionName::Request> request,
  std::shared_ptr<GetRegionName::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming semantic position service request: [%f, %f] in frame [%s]",
    request->position.point.x, request->position.point.y,
    request->position.header.frame_id.c_str());

  // If the request frame is different than the map frame, transform the point
  geometry_msgs::msg::PointStamped point_in_map_frame;
  if (request->position.header.frame_id != map_topic_) {
    try {
      point_in_map_frame = tf2_buffer_->transform(
        request->position, map_topic_, tf2::durationFromSec(transform_tolerance_));
    } catch (tf2::TransformException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to transform point from frame [%s] to frame [%s]: %s",
        request->position.header.frame_id.c_str(), map_topic_.c_str(), ex.what());
      return false;
    }
  } else {
    point_in_map_frame = request->position;
  }

  // Get arguments and check if the point lies within the region
  for (const auto & region : region_list_) {
    if (region.isPointInside(point_in_map_frame.point.x, point_in_map_frame.point.y)) {
      response->region_name = region.name;
      return true;
    }
  }

  response->region_name = GetRegionName::Response::UNKNOWN;
  RCLCPP_FATAL(get_logger(), "Failed to get semantic position");
  return false;
}

bool SemanticNavigationTasks::listAllRegionsService(
  const std::shared_ptr<ListAllRegions::Request>/* request */,
  std::shared_ptr<ListAllRegions::Response> response)
{
  RCLCPP_INFO(get_logger(), "Incoming regions service request");

  // Get arguments and check if the point lies within the region
  for (const auto & region : region_list_) {
    response->region_names.push_back(region.name);
  }

  return true;
}

bool SemanticNavigationTasks::getAdjacentRegionsService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<GetAdjacentRegions::Request> request,
  std::shared_ptr<GetAdjacentRegions::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming adjacent regions service request for region [%s]",
    request->region_name.c_str());

  response->adjacent_regions = region_graph_.getNeighbors(request->region_name);

  return true;
}

bool SemanticNavigationTasks::areRegionsConnectedService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<AreRegionsConnected::Request> request,
  std::shared_ptr<AreRegionsConnected::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming connectivity service request between [%s] and [%s]",
    request->region_a.c_str(), request->region_b.c_str());

  response->connected = region_graph_.areConnected(request->region_a, request->region_b);

  return true;
}

bool SemanticNavigationTasks::getRegionRouteService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<GetRegionRoute::Request> request,
  std::shared_ptr<GetRegionRoute::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming route service request from [%s] to [%s]",
    request->start_region.c_str(), request->goal_region.c_str());

  response->route = region_graph_.findRoute(request->start_region, request->goal_region);

  return true;
}

nav_msgs::msg::Goals SemanticNavigationTasks::generateRandomGoals(
  unsigned int n, Region region, CellLimits limits)
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;

  // Generate random goal pose
  auto [cell_min_x, cell_max_x, cell_min_y, cell_max_y] = limits;
  std::random_device rd;       // obtain a random number from hardware
  std::mt19937 gen(rd());       // seed the generator
  std::uniform_int_distribution<int> dist_x(cell_min_x, cell_max_x);       // define the range
  std::uniform_int_distribution<int> dist_y(cell_min_y, cell_max_y);       // define the range
  std::uniform_real_distribution<double> dist_pi(-M_PI, M_PI);

  unsigned int count = 0;
  while (goals.size() < n) {
    count += 1;
    int cell_x = dist_x(gen);
    int cell_y = dist_y(gen);
    double yaw = dist_pi(gen);

    // Set a random position and orientation for the goal
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_topic_;
    pose.header.stamp = this->now();
    pose.pose.position.x = map_.info.origin.position.x + cell_x * map_.info.resolution;
    pose.pose.position.y = map_.info.origin.position.y + cell_y * map_.info.resolution;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, yaw));

    // If the point lies within region and is not in collision
    if (isPointValid(cell_x, cell_y, region, pose.pose)) {
      // Generate orientation depending on the request
      orientationFromRequest(pose.pose, region, GenerateRandomGoals::Request::INSIDE, 0.0);
      RCLCPP_INFO(
        get_logger(), "Pose %lu (x: %f, y: %f, yaw: %f)",
        goals.size() + 1, pose.pose.position.x, pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation));
      goals.push_back(pose);
    }
  }

  return goals;
}

int8_t SemanticNavigationTasks::cell(unsigned int x, unsigned int y)
{
  // Return 'unknown' if out of bounds
  if (x > map_.info.width || y > map_.info.height) {
    return nav2_util::OCC_GRID_UNKNOWN;
  }

  return map_.data[x + map_.info.width * y];
}

bool SemanticNavigationTasks::inCollision(int x, int y)
{
  int x_min, x_max, y_min, y_max;

  if (is_costmap_) {
    return cell(x, y) != nav2_util::OCC_GRID_FREE;
  }

  x_min = x - inflated_footprint_size_;
  x_max = x + inflated_footprint_size_;
  y_min = y - inflated_footprint_size_;
  y_max = y + inflated_footprint_size_;

  for (int i = x_min; i < x_max; i++) {
    for (int j = y_min; j < y_max; j++) {
      if (cell(i, j) != nav2_util::OCC_GRID_FREE) {
        return true;
      }
    }
  }
  return false;
}

bool SemanticNavigationTasks::isPointValid(
  int x, int y, Region region, geometry_msgs::msg::Pose pose)
{
  return region.isPointInside(pose.position.x, pose.position.y) &&
         region.isPointAtLeastDistanceFromBorders(pose.position.x, pose.position.y, border_) &&
         !inCollision(x, y);
}

void SemanticNavigationTasks::orientationFromRequest(
  geometry_msgs::msg::Pose & pose, const Region & region, std::string orientation,
  double requested_yaw)
{
  double yaw = 0.0;
  if (orientation == GenerateRandomGoals::Request::OUTSIDE) {
    yaw = atan2(
      (pose.position.y - region.centroid().y), (pose.position.x - region.centroid().x));
  } else if (orientation == GenerateRandomGoals::Request::INSIDE) {
    yaw = atan2(
      (pose.position.y - region.centroid().y), (pose.position.x - region.centroid().x)) + M_PI;
  } else if (orientation == GenerateRandomGoals::Request::REQUESTED) {
    yaw = angles::normalize_angle(requested_yaw);
  }
  pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, yaw));
}

polygon_msgs::msg::Polygon2DCollection SemanticNavigationTasks::createPolygons(
  std::vector<Region> list)
{
  polygon_msgs::msg::Polygon2DCollection polygon_array;
  polygon_array.header.frame_id = map_topic_;
  polygon_array.header.stamp = this->now();

  for (const auto & region : list) {
    polygon_array.polygons.push_back(region.polygon);
  }

  return polygon_array;
}

visualization_msgs::msg::MarkerArray SemanticNavigationTasks::createNames(std::vector<Region> list)
{
  visualization_msgs::msg::MarkerArray names_array;
  for (const auto & region : list) {
    // Create label
    visualization_msgs::msg::Marker label_marker;
    label_marker.header.frame_id = map_topic_;
    label_marker.header.stamp = this->now();
    label_marker.ns = "label_region";
    label_marker.id = names_array.markers.size();
    label_marker.text = region.name;
    label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label_marker.action = visualization_msgs::msg::Marker::ADD;
    label_marker.pose.position.x = region.centroid().x;
    label_marker.pose.position.y = region.centroid().y;
    label_marker.pose.position.z = 0.05;
    label_marker.pose.orientation.x = 0.0;
    label_marker.pose.orientation.y = 0.0;
    label_marker.pose.orientation.z = 0.0;
    label_marker.pose.orientation.w = 1.0;
    label_marker.scale.z = 0.5;
    label_marker.color.r = 1.0;
    label_marker.color.g = 1.0;
    label_marker.color.b = 1.0;
    label_marker.color.a = 1.0f;
    names_array.markers.push_back(label_marker);
  }
  return names_array;
}

visualization_msgs::msg::MarkerArray SemanticNavigationTasks::createEdges(
  std::vector<Region> list, const RegionGraph & graph)
{
  visualization_msgs::msg::MarkerArray edges_array;

  // Index the centroids by region name for a quick lookup
  std::unordered_map<std::string, geometry_msgs::msg::Point> centroids;
  for (const auto & region : list) {
    if (!region.polygon.points.empty()) {
      centroids[region.name] = region.centroid();
    }
  }

  // Draw a single line list with one segment per undirected edge, avoiding duplicates
  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = map_topic_;
  edge_marker.header.stamp = this->now();
  edge_marker.ns = "edges_region";
  edge_marker.id = 0;
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.05;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0f;

  for (const auto & region : list) {
    auto centroid_it = centroids.find(region.name);
    if (centroid_it == centroids.end()) {
      continue;
    }
    for (const auto & neighbor : graph.getNeighbors(region.name)) {
      // Each undirected edge is drawn only once
      if (region.name >= neighbor) {
        continue;
      }
      auto neighbor_it = centroids.find(neighbor);
      if (neighbor_it == centroids.end()) {
        continue;
      }
      edge_marker.points.push_back(centroid_it->second);
      edge_marker.points.push_back(neighbor_it->second);
    }
  }

  edges_array.markers.push_back(edge_marker);
  return edges_array;
}

}  // namespace semantic_navigation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(semantic_navigation::SemanticNavigationTasks)
