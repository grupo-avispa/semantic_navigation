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

#ifndef SEMANTIC_GOALS_GENERATOR__SEMANTIC_GOALS_GENERATOR_HPP_
#define SEMANTIC_GOALS_GENERATOR__SEMANTIC_GOALS_GENERATOR_HPP_

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
#include "semantic_navigation_msgs/srv/semantic_goals.hpp"
#include "semantic_navigation_msgs/srv/semantic_position.hpp"
#include "semantic_navigation_msgs/srv/semantic_regions.hpp"

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

/**
 * @class SemanticGoalsGenerator
 * @brief Class to generate goals inside regions of interest (ROIs).
 */
class SemanticGoalsGenerator : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Semantic Goals Generator object.
   *
   */
  SemanticGoalsGenerator();

private:
  using SemanticGoals = semantic_navigation_msgs::srv::SemanticGoals;
  using SemanticPosition = semantic_navigation_msgs::srv::SemanticPosition;
  using SemanticRegions = semantic_navigation_msgs::srv::SemanticRegions;

  /**
   * @brief Update parameters of the node.
   *
   */
  void get_params();

  /**
   * @brief Get the ROI parameters from a file.
   *
   * @param filename Name of the file.
   */
  void get_roi_params(const std::string & filename);

  /**
   * @brief Generate goals inside the regions of interest (ROIs).
   *
   * @param request Request with the name of the ROI.
   * @param response Response with the goals.
   * @return true if the goals are generated.
   */
  bool goals_generator_service(
    const std::shared_ptr<SemanticGoals::Request> request,
    std::shared_ptr<SemanticGoals::Response> response);

  /**
   * @brief Generate a random position inside the region of interest (ROI).
   *
   * @param request Request with the name of the ROI.
   * @param response Response with the position.
   * @return true if the position is generated.
   */
  bool semantic_position_service(
    const std::shared_ptr<SemanticPosition::Request> request,
    std::shared_ptr<SemanticPosition::Response> response);

  /**
   * @brief Generate a list of regions of interest (ROIs).
   *
   * @param request Request with the name of the ROI.
   * @param response Response with the ROIs.
   * @return true if the ROIs are generated.
   */
  bool semantic_regions_service(
    const std::shared_ptr<SemanticRegions::Request> request,
    std::shared_ptr<SemanticRegions::Response> response);

  /**
   * @brief Callback to update the map.
   *
   * @param msg Message with the map.
   */
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Show the visualization of the regions of interest (ROIs).
   *
   */
  void show_visualization();

  /**
   * @brief Process the bounding box of the regions of interest (ROIs).
   *
   * @param roi Region of interest.
   */
  void process_boundingbox(ROI roi);

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
  bool in_collision(int x, int y);

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_pub_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygons_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr names_viz_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Service<SemanticGoals>::SharedPtr goals_generator_service_;
  rclcpp::Service<SemanticPosition>::SharedPtr semantic_position_service_;
  rclcpp::Service<SemanticRegions>::SharedPtr semantic_regions_service_;

  std::recursive_mutex mutex_;
  nav_msgs::msg::OccupancyGrid map_;
  bool is_costmap_, full_map_;
  int inflated_footprint_size_;
  int cell_min_x_, cell_max_x_, cell_min_y_, cell_max_y_;
  float bbox_min_x_, bbox_max_x_, bbox_min_y_, bbox_max_y_;
  float map_min_x_, map_max_x_, map_min_y_, map_max_y_;
  float inflation_radius_, border_;
  std::string goals_topic_, polygons_topic_, names_topic_, map_topic_;
  std::string direction_;
  std::vector<ROI> roi_list_;
};

#endif  // SEMANTIC_GOALS_GENERATOR__SEMANTIC_GOALS_GENERATOR_HPP_
