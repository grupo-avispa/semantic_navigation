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

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"
#include "nav2_util/occ_grid_values.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_tasks/goal_sampler.hpp"

namespace semantic_navigation
{

GoalSampler::GoalSampler(const rclcpp::Logger & logger)
: logger_(logger)
{
}

void GoalSampler::setMap(nav_msgs::msg::OccupancyGrid map)
{
  map_ = std::move(map);
  setInflatedFootprintSize(inflation_radius_, map_.info.resolution);
}

const nav_msgs::msg::OccupancyGrid & GoalSampler::getMap() const
{
  return map_;
}

void GoalSampler::setIsCostmap(bool is_costmap)
{
  is_costmap_ = is_costmap;
}

void GoalSampler::setInflationRadius(double inflation_radius)
{
  inflation_radius_ = inflation_radius;
}

void GoalSampler::setInflatedFootprintSize(double inflation_radius, double resolution)
{
  inflated_footprint_size_ = static_cast<int>(inflation_radius / resolution) + 1;
}

void GoalSampler::seed(std::mt19937::result_type value)
{
  rng_.seed(value);
}

CellLimits GoalSampler::processBoundingBox(
  const nav_msgs::msg::OccupancyGrid & map, const Region & region) const
{
  double map_min_x = map.info.origin.position.x;
  double map_max_x = map.info.origin.position.x + map.info.width * map.info.resolution;
  double map_min_y = map.info.origin.position.y;
  double map_max_y = map.info.origin.position.y + map.info.height * map.info.resolution;

  // Region must lie inside the map boundaries
  // If the region is empty, the whole map is treated as the region by default
  double bbox_min_x, bbox_max_x, bbox_min_y, bbox_max_y;
  if (region.empty()) {
    bbox_min_x = map_min_x;
    bbox_max_x = map_max_x;
    bbox_min_y = map_min_y;
    bbox_max_y = map_max_y;
    RCLCPP_INFO(logger_, "No region specified, full map is used");
  } else {
    // If the region is outside the map, adjust to map boundaries
    // Determine bounding box of Region
    bbox_min_x = std::numeric_limits<double>::infinity();
    bbox_max_x = -std::numeric_limits<double>::infinity();
    bbox_min_y = std::numeric_limits<double>::infinity();
    bbox_max_y = -std::numeric_limits<double>::infinity();

    // Clamp each point into the map bounds (without mutating the caller's region) before
    // folding it into the bounding box.
    for (const auto & p : region.polygon.points) {
      double x = std::clamp(static_cast<double>(p.x), map_min_x, map_max_x);
      double y = std::clamp(static_cast<double>(p.y), map_min_y, map_max_y);

      if (x < bbox_min_x) {bbox_min_x = x;}
      if (x > bbox_max_x) {bbox_max_x = x;}

      if (y < bbox_min_y) {bbox_min_y = y;}
      if (y > bbox_max_y) {bbox_max_y = y;}
    }
  }

  // Calculate bounding box for cell array. Floor (rather than truncate) so that negative
  // coordinates round towards the map origin instead of towards zero.
  int cell_min_x = static_cast<int>(
    std::floor((bbox_min_x - map_.info.origin.position.x) / map_.info.resolution));
  int cell_max_x = static_cast<int>(
    std::floor((bbox_max_x - map_.info.origin.position.x) / map_.info.resolution));
  int cell_min_y = static_cast<int>(
    std::floor((bbox_min_y - map_.info.origin.position.y) / map_.info.resolution));
  int cell_max_y = static_cast<int>(
    std::floor((bbox_max_y - map_.info.origin.position.y) / map_.info.resolution));

  RCLCPP_INFO(
    logger_, "Region bounding box (meters): (%f,%f) (%f,%f)",
    bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y);
  RCLCPP_INFO(
    logger_, "Region bounding box (cells): (%i,%i) (%i,%i)",
    cell_min_x, cell_min_y, cell_max_x, cell_max_y);

  return CellLimits(cell_min_x, cell_max_x, cell_min_y, cell_max_y);
}

int8_t GoalSampler::cell(unsigned int x, unsigned int y) const
{
  // Return 'unknown' if out of bounds. Valid indices are [0, width) and [0, height).
  if (x >= map_.info.width || y >= map_.info.height) {
    return nav2_util::OCC_GRID_UNKNOWN;
  }

  return map_.data[x + map_.info.width * y];
}

bool GoalSampler::inCollision(int x, int y) const
{
  if (is_costmap_) {
    return cell(x, y) != nav2_util::OCC_GRID_FREE;
  }

  int x_min = x - inflated_footprint_size_;
  int x_max = x + inflated_footprint_size_;
  int y_min = y - inflated_footprint_size_;
  int y_max = y + inflated_footprint_size_;

  for (int i = x_min; i < x_max; i++) {
    for (int j = y_min; j < y_max; j++) {
      if (cell(i, j) != nav2_util::OCC_GRID_FREE) {
        return true;
      }
    }
  }
  return false;
}

bool GoalSampler::isPointValid(
  int x, int y, const Region & region, const geometry_msgs::msg::Pose & pose, float border) const
{
  return region.isPointInside(pose.position.x, pose.position.y) &&
         region.isPointAtLeastDistanceFromBorders(pose.position.x, pose.position.y, border) &&
         !inCollision(x, y);
}

void GoalSampler::orientationFromRequest(
  geometry_msgs::msg::Pose & pose, const Region & region, std::string orientation,
  double requested_yaw) const
{
  using semantic_navigation_msgs::srv::GenerateRandomGoals;

  // Random (default) or unset orientation: keep the yaw already sampled by the caller
  if (orientation.empty() || orientation == GenerateRandomGoals::Request::RANDOM) {
    return;
  }

  double yaw = 0.0;
  if (orientation == GenerateRandomGoals::Request::OUTSIDE) {
    yaw = atan2(
      (pose.position.y - region.centroid().y), (pose.position.x - region.centroid().x));
  } else if (orientation == GenerateRandomGoals::Request::INSIDE) {
    yaw = atan2(
      (pose.position.y - region.centroid().y), (pose.position.x - region.centroid().x)) + M_PI;
  } else if (orientation == GenerateRandomGoals::Request::REQUESTED) {
    yaw = angles::normalize_angle(requested_yaw);
  } else {
    RCLCPP_WARN(
      logger_, "Unknown orientation [%s], keeping the random yaw", orientation.c_str());
    return;
  }
  pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, yaw));
}

std::vector<geometry_msgs::msg::PoseStamped> GoalSampler::generateRandomGoals(
  unsigned int n, const Region & region, CellLimits limits, std::string orientation,
  double requested_yaw, float border, const std::string & frame_id, const rclcpp::Time & stamp)
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;

  // Generate random goal pose
  auto [cell_min_x, cell_max_x, cell_min_y, cell_max_y] = limits;
  std::uniform_int_distribution<int> dist_x(cell_min_x, cell_max_x);       // define the range
  std::uniform_int_distribution<int> dist_y(cell_min_y, cell_max_y);       // define the range
  std::uniform_real_distribution<double> dist_pi(-M_PI, M_PI);

  // Bound the number of attempts so that an unreachable region (e.g. fully occupied, or
  // degenerate after being clamped to the map) cannot hang the caller indefinitely.
  const unsigned int max_attempts = std::max<unsigned int>(1000u, n * 100u);
  unsigned int attempts = 0;
  while (goals.size() < n && attempts < max_attempts) {
    ++attempts;
    int cell_x = dist_x(rng_);
    int cell_y = dist_y(rng_);
    double yaw = dist_pi(rng_);

    // Set a random position and orientation for the goal
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = map_.info.origin.position.x + cell_x * map_.info.resolution;
    pose.pose.position.y = map_.info.origin.position.y + cell_y * map_.info.resolution;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, yaw));

    // If the point lies within region and is not in collision
    if (isPointValid(cell_x, cell_y, region, pose.pose, border)) {
      // Generate orientation depending on the request (keeps the sampled yaw for RANDOM)
      orientationFromRequest(pose.pose, region, orientation, requested_yaw);
      RCLCPP_INFO(
        logger_, "Pose %lu (x: %f, y: %f, yaw: %f)",
        goals.size() + 1, pose.pose.position.x, pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation));
      goals.push_back(pose);
    }
  }

  if (goals.size() < n) {
    RCLCPP_WARN(
      logger_, "Only %zu/%u goals could be generated after %u attempts",
      goals.size(), n, attempts);
  }

  return goals;
}

}  // namespace semantic_navigation
