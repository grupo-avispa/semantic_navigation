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

#ifndef SEMANTIC_NAVIGATION_TASKS__TASK_SERVICES_HPP_
#define SEMANTIC_NAVIGATION_TASKS__TASK_SERVICES_HPP_

// C++
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/subscription.hpp"
#include "nav2_ros_common/service_server.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "polygon_msgs/msg/polygon2_d_collection.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "semantic_navigation_msgs/srv/are_regions_connected.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_msgs/srv/get_adjacent_regions.hpp"
#include "semantic_navigation_msgs/srv/get_random_region.hpp"
#include "semantic_navigation_msgs/srv/get_region_name.hpp"
#include "semantic_navigation_msgs/srv/get_region_route.hpp"
#include "semantic_navigation_msgs/srv/list_all_regions.hpp"
#include "semantic_navigation_tasks/goal_sampler.hpp"
#include "semantic_navigation_tasks/region.hpp"
#include "semantic_navigation_tasks/region_graph.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"


namespace semantic_navigation
{

/**
 * @class semantic_navigation::SemanticNavigationTasks
 * @brief Lifecycle node that wires together region loading (RegionsLoader), the connectivity
 * graph (RegionGraph), goal sampling (GoalSampler) and visualization (region_markers) into the
 * generate_random_goals / semantic position / connectivity ROS services.
 */
class SemanticNavigationTasks : public nav2::LifecycleNode
{
public:
  /**
   * @brief Construct a new Semantic Goals Generator object.
   *
   */
  explicit SemanticNavigationTasks(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Semantic Goals Generator object.
   *
   */
  ~SemanticNavigationTasks() = default;

protected:
  using AreRegionsConnected = semantic_navigation_msgs::srv::AreRegionsConnected;
  using GenerateRandomGoals = semantic_navigation_msgs::srv::GenerateRandomGoals;
  using GetAdjacentRegions = semantic_navigation_msgs::srv::GetAdjacentRegions;
  using GetRandomRegion = semantic_navigation_msgs::srv::GetRandomRegion;
  using GetRegionName = semantic_navigation_msgs::srv::GetRegionName;
  using GetRegionRoute = semantic_navigation_msgs::srv::GetRegionRoute;
  using ListAllRegions = semantic_navigation_msgs::srv::ListAllRegions;

  /**
   * @brief Configures the modules parameters and member variables
   *
   * Configures modules plugin.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize module
   * plugin
   */
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates member variables
   *
   * Activates the modules
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   *
   * Deactivates the modules.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * module clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Get the region parameters from a file. Thin wrapper around loadRegionsFile() (see
   * regions_loader.hpp) that logs any warning it returns.
   *
   * @param filepath Name of the file.
   * @param regions Regions of interest. New regions are appended to any already present.
   * @return true if at least one region was loaded from this call.
   */
  bool getRegionsFromFile(
    const std::string & filename, std::vector<semantic_navigation::Region> & regions);

  /**
   * @brief Get the manual connections (add / remove edges) from a file. Thin wrapper around
   * loadRegionsFile() (see regions_loader.hpp) that logs any warning it returns.
   *
   * Reads the optional `connections` section of the regions file. Missing sections are not an
   * error: the returned lists are simply left empty.
   *
   * @param filename Name of the file.
   * @param add Edges to force regardless of the geometry.
   * @param remove Edges to forbid regardless of the geometry.
   */
  void getConnectionsFromFile(
    const std::string & filename, std::vector<Connection> & add,
    std::vector<Connection> & remove);

  /**
   * @brief Load the regions and their manual connectivity overrides from a single read of the
   * regions file.
   *
   * Equivalent to calling getRegionsFromFile followed by getConnectionsFromFile, but the file is
   * only read and parsed from disk once instead of twice.
   *
   * @param filename Name of the file.
   * @param regions Regions of interest. New regions are appended to any already present.
   * @param add Edges to force regardless of the geometry.
   * @param remove Edges to forbid regardless of the geometry.
   * @return true if at least one region was loaded.
   */
  bool loadRegionsAndConnections(
    const std::string & filename, std::vector<semantic_navigation::Region> & regions,
    std::vector<Connection> & add, std::vector<Connection> & remove);

  /**
   * @brief Generate goals inside the regions.
   *
   * The outcome is reported through response->success/message, not through a return value: the
   * ROS 2 service transport delivers the response message regardless of what the bound callback
   * returns, so a bool return here would never actually reach the client.
   *
   * @param request_header Request header.
   * @param request Request with the name of the region.
   * @param response Response with the goals and the success/message outcome.
   */
  void generateRandomGoalsService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GenerateRandomGoals::Request> request,
    std::shared_ptr<GenerateRandomGoals::Response> response);

  /**
   * @brief Get a random named region.
   *
   * @param request_header Request header.
   * @param request Request.
   * @param response Response with the region and the success/message outcome.
   */
  void getRandomRegionService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRandomRegion::Request> request,
    std::shared_ptr<GetRandomRegion::Response> response);

  /**
   * @brief Get the name of the region from a position.
   *
   * @param request_header Request header.
   * @param request Request with the name of the region.
   * @param response Response with the position and the success/message outcome.
   */
  void getRegionNameService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRegionName::Request> request,
    std::shared_ptr<GetRegionName::Response> response);

  /**
   * @brief Get the names of all the regions.
   *
   * @param request_header Request header.
   * @param request Request with the name of the region.
   * @param response Response with the regions and the success/message outcome.
   */
  void listAllRegionsService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListAllRegions::Request> request,
    std::shared_ptr<ListAllRegions::Response> response);

  /**
   * @brief Get the regions directly connected to a given region.
   *
   * @param request_header Request header.
   * @param request Request with the name of the region.
   * @param response Response with the names of the adjacent regions and the success/message
   * outcome.
   */
  void getAdjacentRegionsService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetAdjacentRegions::Request> request,
    std::shared_ptr<GetAdjacentRegions::Response> response);

  /**
   * @brief Check if two regions are connected, directly or transitively.
   *
   * @param request_header Request header.
   * @param request Request with the names of the two regions.
   * @param response Response with the connectivity result and the success/message outcome.
   */
  void areRegionsConnectedService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AreRegionsConnected::Request> request,
    std::shared_ptr<AreRegionsConnected::Response> response);

  /**
   * @brief Get the topological route (sequence of regions) between two regions.
   *
   * @param request_header Request header.
   * @param request Request with the start and goal regions.
   * @param response Response with the ordered route and the success/message outcome.
   */
  void getRegionRouteService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRegionRoute::Request> request,
    std::shared_ptr<GetRegionRoute::Response> response);

  /**
   * @brief Generate random goals inside a region. Thin wrapper around
   * goal_sampler_.generateRandomGoals(), threading in border_/global_frame_/now(). Kept virtual
   * (and at this exact signature) so tests can stub it out without a real map.
   *
   * @param n Number of goals.
   * @param region Region of interest.
   * @param limits Limits of the cells.
   * @param orientation Requested orientation (see GoalSampler::orientationFromRequest).
   * @param requested_yaw Requested yaw, used only when @p orientation is REQUESTED.
   * @return std::vector<geometry_msgs::msg::PoseStamped> Goals (may contain fewer than @p n).
   */
  virtual std::vector<geometry_msgs::msg::PoseStamped> generateRandomGoals(
    unsigned int n, const Region & region, CellLimits limits, std::string orientation,
    double requested_yaw);

  /**
   * @brief Callback to update the map.
   *
   * @param msg Message with the map.
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Create a collection of polygons. Thin wrapper around region_markers::createPolygons().
   *
   * @param list List of regions of interest.
   * @return polygon_msgs::msg::Polygon2DCollection Collection of polygons.
   */
  polygon_msgs::msg::Polygon2DCollection createPolygons(const std::vector<Region> & list);

  /**
   * @brief Create a collection of markers with the names of the regions. Thin wrapper around
   * region_markers::createNames().
   *
   * @param list List of regions of interest.
   * @return visualization_msgs::msg::MarkerArray Collection of markers.
   */
  visualization_msgs::msg::MarkerArray createNames(const std::vector<Region> & list);

  /**
   * @brief Create a collection of markers with the edges of the connectivity graph. Thin wrapper
   * around region_markers::createEdges().
   *
   * @param list List of regions of interest.
   * @param graph Connectivity graph between the regions.
   * @return visualization_msgs::msg::MarkerArray Collection of markers.
   */
  visualization_msgs::msg::MarkerArray createEdges(
    const std::vector<Region> & list, const RegionGraph & graph);

  /**
   * @brief Process the bounding box of the regions inside the map. Thin wrapper around
   * goal_sampler_.processBoundingBox().
   *
   * @param map Map.
   * @param region Region of interest.
   * @return CellLimits Limits of the cells.
   */
  CellLimits processBoundingBox(const nav_msgs::msg::OccupancyGrid & map, const Region & region);

  /**
   * @brief Get the cell value of the map. Thin wrapper around goal_sampler_.cell().
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @return uint8_t Value of the cell.
   */
  int8_t cell(unsigned int x, unsigned int y);

  /**
   * @brief Check if a point is inside the map. Thin wrapper around goal_sampler_.inCollision().
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @return true if the point is inside the map.
   */
  bool inCollision(int x, int y);

  /**
   * @brief Check if the point is valid (i.e., not in collision, inside the map and away from the
   * border). Thin wrapper around goal_sampler_.isPointValid().
   *
   * @return bool True if the point is valid.
   */
  bool isPointValid(int x, int y, const Region & region, const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Get the orientation depending on the request. Thin wrapper around
   * goal_sampler_.orientationFromRequest().
   *
   * @param pose Pose of the goal.
   * @param region Region of interest.
   * @param orientation Requested orientation in string format.
   * @param requested_yaw Requested yaw (Optional).
   */
  void orientationFromRequest(
    geometry_msgs::msg::Pose & pose, const Region & region, std::string orientation,
    double requested_yaw);

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_pub_;
  rclcpp_lifecycle::LifecyclePublisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr
    polygons_viz_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    names_viz_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    edges_viz_pub_;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  nav2::ServiceServer<GenerateRandomGoals>::SharedPtr goals_generator_service_;
  nav2::ServiceServer<GetRandomRegion>::SharedPtr get_random_region_service_;
  nav2::ServiceServer<GetRegionName>::SharedPtr get_region_name_service_;
  nav2::ServiceServer<ListAllRegions>::SharedPtr list_all_regions_service_;
  nav2::ServiceServer<GetAdjacentRegions>::SharedPtr get_adjacent_regions_service_;
  nav2::ServiceServer<AreRegionsConnected>::SharedPtr are_regions_connected_service_;
  nav2::ServiceServer<GetRegionRoute>::SharedPtr get_region_route_service_;

  std::recursive_mutex mutex_;
  // Map/collision model and random goal sampling; see goal_sampler.hpp.
  GoalSampler goal_sampler_;
  bool full_map_, auto_connect_;
  float border_;
  double connectivity_threshold_;
  std::string goals_topic_, polygons_topic_, names_topic_, edges_topic_, map_topic_;
  // TF frame used as header.frame_id for published messages and goals, independent of the
  // (potentially remapped) name of the map topic.
  std::string global_frame_;
  std::vector<semantic_navigation::Region> region_list_;
  RegionGraph region_graph_;
  // Pseudo-random number generator used to pick a random region name in getRandomRegionService,
  // seeded once from hardware entropy in on_configure. Sampling goals within a region has its own
  // independent generator inside goal_sampler_.
  std::mt19937 rng_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  // Transform tolerance for getting the robot pose
  double transform_tolerance_;
};

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__TASK_SERVICES_HPP_
