// Copyright (c) 2026 Alberto J. Tudela Roldán
// Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef SEMANTIC_NAVIGATION_TASKS__GOAL_SAMPLER_HPP_
#define SEMANTIC_NAVIGATION_TASKS__GOAL_SAMPLER_HPP_

#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_navigation_tasks/region.hpp"

namespace semantic_navigation
{

/// Cell-space bounding box of a region: (min_x, max_x, min_y, max_y).
using CellLimits = std::tuple<int, int, int, int>;

/**
 * @class semantic_navigation::GoalSampler
 * @brief Map/collision model and random goal sampling, decoupled from any ROS node.
 *
 * Owns the occupancy grid and the parameters needed to test whether a cell is free and a point
 * is a valid goal, and samples random goal poses inside a region. It only depends on message
 * types and an rclcpp::Logger (which does not require rclcpp::init()), so it can be unit tested
 * with a synthetic map instead of a full lifecycle node.
 */
class GoalSampler
{
public:
  /**
   * @brief Construct a new Goal Sampler object.
   * @param logger Logger used for informational and warning messages.
   */
  explicit GoalSampler(const rclcpp::Logger & logger = rclcpp::get_logger("goal_sampler"));

  /**
   * @brief Set the occupancy grid used for collision checking and sampling.
   *
   * Recomputes the inflated footprint size from the current inflation radius and the map's
   * resolution, matching the previous per-map-message behaviour.
   *
   * @param map Occupancy grid.
   */
  void setMap(nav_msgs::msg::OccupancyGrid map);

  /**
   * @brief Get the current occupancy grid.
   * @return const nav_msgs::msg::OccupancyGrid& Occupancy grid.
   */
  const nav_msgs::msg::OccupancyGrid & getMap() const;

  /**
   * @brief Set whether the map is a costmap (already inflated) or a raw occupancy grid.
   * @param is_costmap True if the map argument is a costmap.
   */
  void setIsCostmap(bool is_costmap);

  /**
   * @brief Set the inflation radius of the robot's footprint, in meters.
   * @param inflation_radius Inflation radius.
   */
  void setInflationRadius(double inflation_radius);

  /**
   * @brief Directly set the inflated footprint size, in cells.
   *
   * Exposed so that callers (and tests) can configure the derived footprint size without
   * depending on the resolution of whatever map happens to be set.
   *
   * @param inflation_radius Inflation radius, in meters.
   * @param resolution Map resolution, in meters/cell.
   */
  void setInflatedFootprintSize(double inflation_radius, double resolution);

  /**
   * @brief Seed the internal random number generator.
   * @param value Seed value.
   */
  void seed(std::mt19937::result_type value);

  /**
   * @brief Process the bounding box of a region inside the map.
   *
   * Uses only @p map (not the stored map_), so the result is consistent whether or not @p map
   * is the same object last passed to setMap().
   *
   * @param map Map.
   * @param region Region of interest.
   * @return CellLimits Limits of the cells.
   */
  CellLimits processBoundingBox(
    const nav_msgs::msg::OccupancyGrid & map, const Region & region) const;

  /**
   * @brief Get the cell value of the map.
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @return int8_t Value of the cell, or OCC_GRID_UNKNOWN if (x, y) is out of bounds.
   */
  int8_t cell(unsigned int x, unsigned int y) const;

  /**
   * @brief Check if a cell (and, unless is_costmap_ is set, its inflated footprint) is free.
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @return true if the cell is in collision.
   */
  bool inCollision(int x, int y) const;

  /**
   * @brief Check if the point is valid (i.e., not in collision, inside the region and away from
   * its border).
   *
   * @param x X coordinate (cell space).
   * @param y Y coordinate (cell space).
   * @param region Region the point must lie inside.
   * @param pose Pose to validate.
   * @param border Minimum distance from the region borders.
   * @return true if the point is valid.
   */
  bool isPointValid(
    int x, int y, const Region & region, const geometry_msgs::msg::Pose & pose,
    float border) const;

  /**
   * @brief Get the orientation depending on the request:
   * - Outside: arrow pointing outside the region.
   * - Inside: arrow pointing inside the region.
   * - Requested: arrow pointing to the requested position.
   * - Random (or empty/unknown): keeps the orientation already set in @p pose by the caller.
   *
   * @param pose Pose of the goal. Its orientation is left untouched for RANDOM.
   * @param region Region of interest.
   * @param orientation Requested orientation in string format.
   * @param requested_yaw Requested yaw, used only when @p orientation is REQUESTED.
   */
  void orientationFromRequest(
    geometry_msgs::msg::Pose & pose, const Region & region, std::string orientation,
    double requested_yaw) const;

  /**
   * @brief Generate random goals inside a region.
   *
   * Sampling is bounded to a maximum number of attempts, so an unreachable region (e.g. fully
   * occupied, or with a degenerate bounding box) returns fewer than @p n goals instead of
   * blocking forever.
   *
   * @param n Number of goals.
   * @param region Region of interest.
   * @param limits Limits of the cells.
   * @param orientation Requested orientation of the goals (see orientationFromRequest).
   * @param requested_yaw Requested yaw, used only when @p orientation is REQUESTED.
   * @param border Minimum distance from the region borders.
   * @param frame_id Frame id stamped on each generated pose.
   * @param stamp Timestamp stamped on each generated pose.
   * @return std::vector<geometry_msgs::msg::PoseStamped> Goals (may contain fewer than @p n).
   */
  std::vector<geometry_msgs::msg::PoseStamped> generateRandomGoals(
    unsigned int n, const Region & region, CellLimits limits, std::string orientation,
    double requested_yaw, float border, const std::string & frame_id, const rclcpp::Time & stamp);

private:
  rclcpp::Logger logger_;
  nav_msgs::msg::OccupancyGrid map_;
  bool is_costmap_ = false;
  double inflation_radius_ = 0.5;
  int inflated_footprint_size_ = 1;
  // Pseudo-random number generator; seed() should be called once from hardware entropy by the
  // owner before relying on it for anything requiring true randomness.
  std::mt19937 rng_;
};

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__GOAL_SAMPLER_HPP_
