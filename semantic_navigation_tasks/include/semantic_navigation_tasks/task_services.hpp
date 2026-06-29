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
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <tuple>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/subscription.hpp"
#include "nav2_ros_common/service_server.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
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
#include "semantic_navigation_tasks/region.hpp"
#include "semantic_navigation_tasks/region_graph.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"


namespace semantic_navigation
{

using CellLimits = std::tuple<int, int, int, int>;

/**
 * @class semantic_navigation::SemanticNavigationTasks
 * @brief Class to generate goals inside regions.
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
   * @brief Get the region parameters from a file.
   *
   * @param filepath Name of the file.
   * @param regions Regions of interest.
   * @return true if the regions are loaded.
   */
  bool getRegionsFromFile(
    const std::string & filename, std::vector<semantic_navigation::Region> & regions);

  /**
   * @brief Get the manual connections (add / remove edges) from a file.
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
   * @brief Generate goals inside the regions.
   *
   * @param request_header Request header.
   * @param request Request with the name of the region.
   * @param response Response with the goals.
   * @return true if the goals are generated.
   */
  bool generateRandomGoalsService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GenerateRandomGoals::Request> request,
    std::shared_ptr<GenerateRandomGoals::Response> response);

  /**
   * @brief Get a random named region.
   *
   * @param request_header Request header.
   * @param request Request.
   * @param response Response with the region.
   * @return true if the position is generated.
   */
  bool getRandomRegionService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRandomRegion::Request> request,
    std::shared_ptr<GetRandomRegion::Response> response);

  /**
   * @brief Get the name of the region from a position.
   *
   * @param request_header Request header.
   * @param request Request with the name of the region.
   * @param response Response with the position.
   * @return true if the position is generated.
   */
  bool getRegionNameService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRegionName::Request> request,
    std::shared_ptr<GetRegionName::Response> response);

  /**
   * @brief Get the names of all the regions.
   *
   * @param request_header Request header.
   * @param request Request with the name of the regions.
   * @param response Response with the regions.
   * @return true if the regions are generated.
   */
  bool listAllRegionsService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListAllRegions::Request> request,
    std::shared_ptr<ListAllRegions::Response> response);

  /**
   * @brief Get the regions directly connected to a given region.
   *
   * @param request_header Request header.
   * @param request Request with the name of the region.
   * @param response Response with the names of the adjacent regions.
   * @return true if the service is processed.
   */
  bool getAdjacentRegionsService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetAdjacentRegions::Request> request,
    std::shared_ptr<GetAdjacentRegions::Response> response);

  /**
   * @brief Check if two regions are connected, directly or transitively.
   *
   * @param request_header Request header.
   * @param request Request with the names of the two regions.
   * @param response Response with the connectivity result.
   * @return true if the service is processed.
   */
  bool areRegionsConnectedService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AreRegionsConnected::Request> request,
    std::shared_ptr<AreRegionsConnected::Response> response);

  /**
   * @brief Get the topological route (sequence of regions) between two regions.
   *
   * @param request_header Request header.
   * @param request Request with the start and goal regions.
   * @param response Response with the ordered route.
   * @return true if the service is processed.
   */
  bool getRegionRouteService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRegionRoute::Request> request,
    std::shared_ptr<GetRegionRoute::Response> response);

  /**
   * @brief Generate random goals inside a region.
   *
   * @param n Number of goals.
   * @param region Region of interest.
   * @param limits Limits of the cells.
   * @return nav_msgs::msg::Goals Goals.
   */
  virtual nav_msgs::msg::Goals generateRandomGoals(
    unsigned int n, Region region, CellLimits limits);

  /**
   * @brief Callback to update the map.
   *
   * @param msg Message with the map.
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Create a collection of polygons.
   *
   * @param list List of regions of interest.
   * @return polygon_msgs::msg::Polygon2DCollection Collection of polygons.
   */
  polygon_msgs::msg::Polygon2DCollection createPolygons(std::vector<Region> list);

  /**
   * @brief Create a collection of markers with the names of the regions.
   *
   * @param list List of regions of interest.
   * @return visualization_msgs::msg::MarkerArray Collection of markers.
   */
  visualization_msgs::msg::MarkerArray createNames(std::vector<Region> list);

  /**
   * @brief Create a collection of markers with the edges of the connectivity graph.
   *
   * Each edge is drawn as a line between the centroids of the two connected regions.
   *
   * @param list List of regions of interest.
   * @param graph Connectivity graph between the regions.
   * @return visualization_msgs::msg::MarkerArray Collection of markers.
   */
  visualization_msgs::msg::MarkerArray createEdges(
    std::vector<Region> list, const RegionGraph & graph);

  /**
   * @brief Process the bounding box of the regions inside the map.
   *
   * @param map Map.
   * @param region Region of interest.
   * @return CellLimits Limits of the cells.
   */
  CellLimits processBoundingBox(const nav_msgs::msg::OccupancyGrid & map, Region region);

  /**
   * @brief Get the cell value of the map.
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @return uint8_t Value of the cell.
   */
  int8_t cell(unsigned int x, unsigned int y);

  /**
   * @brief Check if a point is inside the map.
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @return true if the point is inside the map.
   */
  bool inCollision(int x, int y);

  /**
   * @brief Check if the point is valid (i.e., not in collision, inside the map and away from the
   * border).
   *
   * @return bool True if the point is valid.
   */
  bool isPointValid(int x, int y, Region region, geometry_msgs::msg::Pose pose);

  /**
   * @brief Get the orientation depending on the request:
   * - Outside: arrow pointing outside the region.
   * - Inside: arrow pointing inside the region.
   * - Requested: arrow pointing to the requested position.
   * - Random: random orientation.
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
  nav_msgs::msg::OccupancyGrid map_;
  bool is_costmap_, full_map_, auto_connect_;
  int inflated_footprint_size_;
  float inflation_radius_, border_;
  double connectivity_threshold_;
  std::string goals_topic_, polygons_topic_, names_topic_, edges_topic_, map_topic_;
  std::vector<semantic_navigation::Region> region_list_;
  RegionGraph region_graph_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  // Transform tolerance for getting the robot pose
  double transform_tolerance_;
};

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__TASK_SERVICES_HPP_
