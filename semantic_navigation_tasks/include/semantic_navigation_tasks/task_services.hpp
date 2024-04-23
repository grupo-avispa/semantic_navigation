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

#ifndef SEMANTIC_NAVIGATION_TASKS__TASK_SERVICES_HPP_
#define SEMANTIC_NAVIGATION_TASKS__TASK_SERVICES_HPP_

// C++
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "polygon_msgs/msg/polygon2_d_collection.hpp"
#include "slg_msgs/polygon.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_msgs/srv/get_region_name.hpp"
#include "semantic_navigation_msgs/srv/list_all_regions.hpp"

struct ROI
{
  slg::Polygon polygon;
  float yaw;

  inline bool empty() {return polygon.empty();}
  inline void clear() {return polygon.clear();}
  inline std::string get_name() {return polygon.get_name();}
  inline void set_name(std::string name) {polygon.set_name(name);}

  /* Check if a point is inside the region of interest (ROI) */
  bool in_roi(float x, float y)
  {
    if (polygon.size() == 0) {return true;}
    return polygon.contains(slg::Point2D(x, y));
  }

  /* Check if the point is at distance from all borders */
  bool distance_from_borders(float x, float y, float border)
  {
    for (auto & edge : polygon.get_edges()) {
      if (edge.distance(slg::Point2D(x, y)) < border) {return false;}
    }
    return true;
  }
};

namespace semantic_navigation
{

/**
 * @class semantic_navigation::SemanticNavigationTasks
 * @brief Class to generate goals inside regions of interest (ROIs).
 */
class SemanticNavigationTasks : public rclcpp::Node
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

private:
  using GenerateRandomGoals = semantic_navigation_msgs::srv::GenerateRandomGoals;
  using GetRegionName = semantic_navigation_msgs::srv::GetRegionName;
  using ListAllRegions = semantic_navigation_msgs::srv::ListAllRegions;

  /**
   * @brief Update parameters of the node.
   *
   */
  void getParams();

  /**
   * @brief Get the region parameters from a file.
   *
   * @param filename Name of the file.
   */
  void getRegionParams(const std::string & filename);

  /**
   * @brief Generate goals inside the regions of interest (ROIs).
   *
   * @param request Request with the name of the region.
   * @param response Response with the goals.
   * @return true if the goals are generated.
   */
  bool generateRandomGoalsService(
    const std::shared_ptr<GenerateRandomGoals::Request> request,
    std::shared_ptr<GenerateRandomGoals::Response> response);

  /**
   * @brief Get the name of the region of interest (ROI) from a position.
   *
   * @param request Request with the name of the region.
   * @param response Response with the position.
   * @return true if the position is generated.
   */
  bool getRegionNameService(
    const std::shared_ptr<GetRegionName::Request> request,
    std::shared_ptr<GetRegionName::Response> response);

  /**
   * @brief Get the names of all the regions of interest (ROIs).
   *
   * @param request Request with the name of the region.
   * @param response Response with the regions.
   * @return true if the regions are generated.
   */
  bool listAllRegionsService(
    const std::shared_ptr<ListAllRegions::Request> request,
    std::shared_ptr<ListAllRegions::Response> response);

  /**
   * @brief Callback to update the map.
   *
   * @param msg Message with the map.
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Show the visualization of the regions of interest (ROIs).
   *
   */
  void showVisualization();

  /**
   * @brief Process the bounding box of the regions of interest (ROIs).
   *
   * @param roi Region of interest.
   */
  void processBoundingbox(ROI roi);

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

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_pub_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygons_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr names_viz_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Service<GenerateRandomGoals>::SharedPtr goals_generator_service_;
  rclcpp::Service<GetRegionName>::SharedPtr get_region_name_service_;
  rclcpp::Service<ListAllRegions>::SharedPtr list_all_regions_service_;

  std::recursive_mutex mutex_;
  nav_msgs::msg::OccupancyGrid map_;
  bool is_costmap_, full_map_;
  int inflated_footprint_size_;
  int cell_min_x_, cell_max_x_, cell_min_y_, cell_max_y_;
  float bbox_min_x_, bbox_max_x_, bbox_min_y_, bbox_max_y_;
  float map_min_x_, map_max_x_, map_min_y_, map_max_y_;
  float inflation_radius_, border_;
  std::string goals_topic_, polygons_topic_, names_topic_, map_topic_;
  std::string orientation_;
  std::vector<ROI> region_list_;
};

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__TASK_SERVICES_HPP_
