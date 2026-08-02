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

#include <random>

// ROS
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Semantic Navigation
#include "semantic_navigation_tasks/region_markers.hpp"
#include "semantic_navigation_tasks/regions_loader.hpp"
#include "semantic_navigation_tasks/task_services.hpp"

namespace semantic_navigation
{

using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3;

SemanticNavigationTasks::SemanticNavigationTasks(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("semantic_navigation_tasks", "", options),
  goal_sampler_(get_logger()),
  border_(0.0)
{
  RCLCPP_INFO(get_logger(), "Creating Semantic Navigation Tasks");
}

nav2_util::CallbackReturn SemanticNavigationTasks::on_configure(const rclcpp_lifecycle::State &)
{
  // BOOLEAN PARAMS ..........................................................................
  bool is_costmap = false;
  nav2_util::declare_parameter_if_not_declared(
    this, "is_costmap",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Is the map a costmap?"));
  this->get_parameter("is_costmap", is_costmap);
  goal_sampler_.setIsCostmap(is_costmap);
  RCLCPP_INFO(
    get_logger(), "The parameter is_costmap is set to: [%s]", is_costmap ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    this, "full_map",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Use the full map as Region?"));
  this->get_parameter("full_map", full_map_);
  RCLCPP_INFO(
    get_logger(), "The parameter full_map is set to: [%s]", full_map_ ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    this, "auto_connect",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Detect connectivity between regions automatically from their geometry?"));
  this->get_parameter("auto_connect", auto_connect_);
  RCLCPP_INFO(
    get_logger(), "The parameter auto_connect is set to: [%s]", auto_connect_ ? "true" : "false");

  // FLOAT PARAMS ..........................................................................
  double inflation_radius = 0.5;
  nav2_util::declare_parameter_if_not_declared(
    this, "inflation_radius",
    rclcpp::ParameterValue(0.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Inflation radius for the robot footprint"));
  this->get_parameter("inflation_radius", inflation_radius);
  goal_sampler_.setInflationRadius(inflation_radius);
  RCLCPP_INFO(
    get_logger(), "The parameter inflation_radius is set to: [%f]", inflation_radius);

  nav2_util::declare_parameter_if_not_declared(
    this, "transform_tolerance",
    rclcpp::ParameterValue(0.2), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Transform tolerance for TF2"));
  this->get_parameter("transform_tolerance", transform_tolerance_);
  RCLCPP_INFO(
    get_logger(), "The parameter transform_tolerance is set to: [%f]", transform_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
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

  nav2_util::declare_parameter_if_not_declared(
    this, "global_frame",
    rclcpp::ParameterValue("map"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "TF frame used as header.frame_id for published messages and goals. Independent of "
      "map_topic, which may be remapped to a topic whose name does not match a TF frame."));
  this->get_parameter("global_frame", global_frame_);
  RCLCPP_INFO(
    get_logger(), "The parameter global_frame is set to: [%s]", global_frame_.c_str());

  std::string regions_filename;
  nav2_util::declare_parameter_if_not_declared(
    this, "regions_filename",
    rclcpp::ParameterValue("regions.yaml"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("File where the regions are stored"));
  this->get_parameter("regions_filename", regions_filename);
  RCLCPP_INFO(
    get_logger(), "The parameter regions_filename is set to: [%s]", regions_filename.c_str());

  std::vector<Connection> add_edges, remove_edges;
  if (!loadRegionsAndConnections(regions_filename, region_list_, add_edges, remove_edges)) {
    RCLCPP_ERROR(get_logger(), "The list of regions could not be found");
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Build the connectivity graph between the regions
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
    std::bind(&SemanticNavigationTasks::listAllRegionsService, this, _1, _2));
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

  // Seed the random number generators once, instead of on every service call. rng_ and
  // goal_sampler_'s internal generator are independent, so each gets its own draw of entropy.
  std::random_device rd;
  rng_.seed(rd());
  goal_sampler_.seed(rd());

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
  auto data = loadRegionsFile(filename);
  for (const auto & warning : data.warnings) {
    RCLCPP_ERROR(get_logger(), "%s", warning.c_str());
  }
  regions.insert(regions.end(), data.regions.begin(), data.regions.end());
  return data.success;
}

void SemanticNavigationTasks::getConnectionsFromFile(
  const std::string & filename, std::vector<Connection> & add,
  std::vector<Connection> & remove)
{
  auto data = loadRegionsFile(filename);
  for (const auto & warning : data.warnings) {
    RCLCPP_ERROR(get_logger(), "%s", warning.c_str());
  }
  add.insert(add.end(), data.add_edges.begin(), data.add_edges.end());
  remove.insert(remove.end(), data.remove_edges.begin(), data.remove_edges.end());
}

bool SemanticNavigationTasks::loadRegionsAndConnections(
  const std::string & filename, std::vector<semantic_navigation::Region> & regions,
  std::vector<Connection> & add, std::vector<Connection> & remove)
{
  RCLCPP_INFO(get_logger(), "Reading regions from file: %s", filename.c_str());
  auto data = loadRegionsFile(filename);
  for (const auto & warning : data.warnings) {
    RCLCPP_ERROR(get_logger(), "%s", warning.c_str());
  }
  regions.insert(regions.end(), data.regions.begin(), data.regions.end());
  add.insert(add.end(), data.add_edges.begin(), data.add_edges.end());
  remove.insert(remove.end(), data.remove_edges.begin(), data.remove_edges.end());
  return data.success;
}

void SemanticNavigationTasks::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  RCLCPP_INFO_ONCE(
    get_logger(), "Received a %d X %d map @ %.3f m/pix",
    msg->info.width, msg->info.height, msg->info.resolution);

  goal_sampler_.setMap(*msg);
}

semantic_navigation::CellLimits SemanticNavigationTasks::processBoundingBox(
  const nav_msgs::msg::OccupancyGrid & map, const Region & region)
{
  return goal_sampler_.processBoundingBox(map, region);
}

void SemanticNavigationTasks::generateRandomGoalsService(
  const std::shared_ptr<GenerateRandomGoals::Request> request,
  std::shared_ptr<GenerateRandomGoals::Response> response)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  Region current_region;
  RCLCPP_INFO(
    get_logger(), "Incoming goals generator service request: [%i, %s]",
    request->n, request->region_name.c_str());

  // Get arguments
  if (request->n <= 0) {
    response->message = "The number of goals requested must be greater than zero";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    response->success = false;
    return;
  }
  unsigned int n = static_cast<unsigned int>(request->n);
  for (const auto & region : region_list_) {
    if (region.name == request->region_name) {
      current_region = region;
      break;
    }
  }
  border_ = request->border;

  // If the requested region is empty and we don't want to use the full map
  if (current_region.empty() && !full_map_) {
    response->message = "The requested region [" + request->region_name +
      "], could not be found in the list";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    response->success = false;
    return;
  }

  // Check if we have a map
  if (goal_sampler_.getMap().data.empty()) {
    response->message = "Failed to get map at [" + map_topic_ + "]";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    response->success = false;
    return;
  }

  // Process bounding box
  auto limits = processBoundingBox(goal_sampler_.getMap(), current_region);
  auto [cell_min_x, cell_max_x, cell_min_y, cell_max_y] = limits;
  if (cell_min_x > cell_max_x || cell_min_y > cell_max_y) {
    response->message = "Invalid bounding box for region [" + request->region_name + "]";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    response->success = false;
    return;
  }

  // Generate response
  response->goals = generateRandomGoals(
    n, current_region, limits, request->orientation, static_cast<double>(request->yaw));
  response->success = true;

  // Publish goals
  geometry_msgs::msg::PoseArray goals_array;
  goals_array.header.frame_id = global_frame_;
  goals_array.header.stamp = this->now();
  for (const auto & goal : response->goals) {
    goals_array.poses.push_back(goal.pose);
  }
  goals_pub_->publish(goals_array);
}

void SemanticNavigationTasks::getRandomRegionService(
  const std::shared_ptr<GetRandomRegion::Request>/*request*/,
  std::shared_ptr<GetRandomRegion::Response> response)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  if (region_list_.empty()) {
    response->message = "Cannot get a random region: the list of regions is empty";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    response->region_name = GetRandomRegion::Response::UNKNOWN;
    response->success = false;
    return;
  }

  // Generate random region
  std::uniform_int_distribution<size_t> dist_region(0, region_list_.size() - 1);

  size_t region_idx = dist_region(rng_);
  response->region_name = region_list_[region_idx].name;
  response->success = true;

  RCLCPP_INFO(
    get_logger(), "Incoming random region service request: [%s]", response->region_name.c_str());
}

void SemanticNavigationTasks::getRegionNameService(
  const std::shared_ptr<GetRegionName::Request> request,
  std::shared_ptr<GetRegionName::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming semantic position service request: [%f, %f] in frame [%s]",
    request->position.point.x, request->position.point.y,
    request->position.header.frame_id.c_str());

  // If the request frame is different than the global frame, transform the point
  geometry_msgs::msg::PointStamped point_in_map_frame;
  if (request->position.header.frame_id != global_frame_) {
    try {
      point_in_map_frame = tf2_buffer_->transform(
        request->position, global_frame_, tf2::durationFromSec(transform_tolerance_));
    } catch (tf2::TransformException & ex) {
      response->message = std::string("Failed to transform point from frame [") +
        request->position.header.frame_id + "] to frame [" + global_frame_ + "]: " + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      response->region_name = GetRegionName::Response::UNKNOWN;
      response->success = false;
      return;
    }
  } else {
    point_in_map_frame = request->position;
  }

  // Get arguments and check if the point lies within the region
  for (const auto & region : region_list_) {
    if (region.isPointInside(point_in_map_frame.point.x, point_in_map_frame.point.y)) {
      response->region_name = region.name;
      response->success = true;
      return;
    }
  }

  // The point does not lie in any region: a normal, expected outcome, not a service failure
  response->region_name = GetRegionName::Response::UNKNOWN;
  response->success = true;
  RCLCPP_WARN(get_logger(), "Failed to get semantic position: point is outside all regions");
}

void SemanticNavigationTasks::listAllRegionsService(
  const std::shared_ptr<ListAllRegions::Request>/* request */,
  std::shared_ptr<ListAllRegions::Response> response)
{
  RCLCPP_INFO(get_logger(), "Incoming regions service request");

  // Get arguments and check if the point lies within the region
  for (const auto & region : region_list_) {
    response->region_names.push_back(region.name);
  }
  response->success = true;
}

void SemanticNavigationTasks::getAdjacentRegionsService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<GetAdjacentRegions::Request> request,
  std::shared_ptr<GetAdjacentRegions::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming adjacent regions service request for region [%s]",
    request->region_name.c_str());

  if (!region_graph_.hasRegion(request->region_name)) {
    response->message = "Unknown region [" + request->region_name + "]";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    response->success = false;
    return;
  }

  response->adjacent_regions = region_graph_.getNeighbors(request->region_name);
  response->success = true;
}

void SemanticNavigationTasks::areRegionsConnectedService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<AreRegionsConnected::Request> request,
  std::shared_ptr<AreRegionsConnected::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming connectivity service request between [%s] and [%s]",
    request->region_a.c_str(), request->region_b.c_str());

  if (!region_graph_.hasRegion(request->region_a) ||
    !region_graph_.hasRegion(request->region_b))
  {
    response->message = "Unknown region: [" + request->region_a + "] or [" +
      request->region_b + "]";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    response->success = false;
    return;
  }

  response->connected = region_graph_.areConnected(request->region_a, request->region_b);
  response->success = true;
}

void SemanticNavigationTasks::getRegionRouteService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<GetRegionRoute::Request> request,
  std::shared_ptr<GetRegionRoute::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Incoming route service request from [%s] to [%s]",
    request->start_region.c_str(), request->goal_region.c_str());

  if (!region_graph_.hasRegion(request->start_region) ||
    !region_graph_.hasRegion(request->goal_region))
  {
    response->message = "Unknown region: [" + request->start_region + "] or [" +
      request->goal_region + "]";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    response->success = false;
    return;
  }

  response->route = region_graph_.findRoute(request->start_region, request->goal_region);
  response->success = true;
}

std::vector<geometry_msgs::msg::PoseStamped> SemanticNavigationTasks::generateRandomGoals(
  unsigned int n, const Region & region, CellLimits limits, std::string orientation,
  double requested_yaw)
{
  return goal_sampler_.generateRandomGoals(
    n, region, limits, orientation, requested_yaw, border_, global_frame_, this->now());
}

int8_t SemanticNavigationTasks::cell(unsigned int x, unsigned int y)
{
  return goal_sampler_.cell(x, y);
}

bool SemanticNavigationTasks::inCollision(int x, int y)
{
  return goal_sampler_.inCollision(x, y);
}

bool SemanticNavigationTasks::isPointValid(
  int x, int y, const Region & region, const geometry_msgs::msg::Pose & pose)
{
  return goal_sampler_.isPointValid(x, y, region, pose, border_);
}

void SemanticNavigationTasks::orientationFromRequest(
  geometry_msgs::msg::Pose & pose, const Region & region, std::string orientation,
  double requested_yaw)
{
  goal_sampler_.orientationFromRequest(pose, region, orientation, requested_yaw);
}

polygon_msgs::msg::Polygon2DCollection SemanticNavigationTasks::createPolygons(
  const std::vector<Region> & list)
{
  return region_markers::createPolygons(list, global_frame_, this->now());
}

visualization_msgs::msg::MarkerArray SemanticNavigationTasks::createNames(
  const std::vector<Region> & list)
{
  return region_markers::createNames(list, global_frame_, this->now());
}

visualization_msgs::msg::MarkerArray SemanticNavigationTasks::createEdges(
  const std::vector<Region> & list, const RegionGraph & graph)
{
  return region_markers::createEdges(list, graph, global_frame_, this->now());
}

}  // namespace semantic_navigation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(semantic_navigation::SemanticNavigationTasks)
