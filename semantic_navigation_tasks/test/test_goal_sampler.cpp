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

#include "gtest/gtest.h"
#include "nav2_util/occ_grid_values.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_tasks/goal_sampler.hpp"

// This whole test file exercises pure domain logic through a synthetic map: it never calls
// rclcpp::init() and never constructs a node, matching the "test the algorithm without a
// lifecycle node" goal.

using semantic_navigation::GoalSampler;
using semantic_navigation::Region;
using GenerateRandomGoals = semantic_navigation_msgs::srv::GenerateRandomGoals;

namespace
{

nav_msgs::msg::OccupancyGrid makeFreeMap(int width, int height, double resolution)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.data = std::vector<int8_t>(width * height, nav2_util::OCC_GRID_FREE);
  return map;
}

Region makeSquareRegion(const std::string & name, double x0, double y0, double side)
{
  Region region;
  region.name = name;
  polygon_msgs::msg::Point2D p;
  p.x = x0; p.y = y0; region.polygon.points.push_back(p);
  p.x = x0; p.y = y0 + side; region.polygon.points.push_back(p);
  p.x = x0 + side; p.y = y0 + side; region.polygon.points.push_back(p);
  p.x = x0 + side; p.y = y0; region.polygon.points.push_back(p);
  return region;
}

}  // namespace

TEST(GoalSamplerTest, cellOutOfBoundsIsUnknown) {
  GoalSampler sampler;
  sampler.setMap(makeFreeMap(10, 10, 0.5));

  EXPECT_EQ(sampler.cell(9, 9), nav2_util::OCC_GRID_FREE);
  EXPECT_EQ(sampler.cell(10, 10), nav2_util::OCC_GRID_UNKNOWN);
  EXPECT_EQ(sampler.cell(0, 13), nav2_util::OCC_GRID_UNKNOWN);
}

TEST(GoalSamplerTest, inCollisionRespectsInflatedFootprint) {
  GoalSampler sampler;
  auto map = makeFreeMap(10, 10, 0.5);
  sampler.setMap(map);
  sampler.setInflationRadius(0.5);
  sampler.setInflatedFootprintSize(0.5, 0.5);

  EXPECT_FALSE(sampler.inCollision(5, 5));

  map.data[0] = nav2_util::OCC_GRID_OCCUPIED;
  sampler.setMap(map);
  EXPECT_TRUE(sampler.inCollision(0, 0));
}

TEST(GoalSamplerTest, inCollisionOnCostmapIgnoresFootprint) {
  GoalSampler sampler;
  auto map = makeFreeMap(10, 10, 0.5);
  sampler.setMap(map);
  sampler.setIsCostmap(true);

  EXPECT_FALSE(sampler.inCollision(0, 0));

  map.data[0] = nav2_util::OCC_GRID_OCCUPIED;
  sampler.setMap(map);
  EXPECT_TRUE(sampler.inCollision(0, 0));
}

TEST(GoalSamplerTest, processBoundingBoxEmptyRegionIsFullMap) {
  GoalSampler sampler;
  auto map = makeFreeMap(10, 10, 0.5);

  auto [min_x, max_x, min_y, max_y] = sampler.processBoundingBox(map, Region());

  EXPECT_EQ(min_x, 0);
  EXPECT_EQ(max_x, 10);
  EXPECT_EQ(min_y, 0);
  EXPECT_EQ(max_y, 10);
}

TEST(GoalSamplerTest, processBoundingBoxClampsRegionOutsideTheMap) {
  GoalSampler sampler;
  auto map = makeFreeMap(10, 10, 0.5);
  auto region = makeSquareRegion("outside", -20.0, -20.0, 8.0);

  auto [min_x, max_x, min_y, max_y] = sampler.processBoundingBox(map, region);

  EXPECT_EQ(min_x, 0);
  EXPECT_EQ(max_x, 0);
  EXPECT_EQ(min_y, 0);
  EXPECT_EQ(max_y, 0);
}

TEST(GoalSamplerTest, isPointValidChecksRegionBorderAndCollision) {
  GoalSampler sampler;
  sampler.setMap(makeFreeMap(10, 10, 0.5));
  sampler.setInflationRadius(0.0);
  sampler.setInflatedFootprintSize(0.0, 0.5);
  auto region = makeSquareRegion("r", 0.0, 0.0, 1.0);

  geometry_msgs::msg::Pose inside_pose;
  inside_pose.position.x = 0.5;
  inside_pose.position.y = 0.5;
  EXPECT_TRUE(sampler.isPointValid(1, 1, region, inside_pose, 0.0f));

  geometry_msgs::msg::Pose outside_pose;
  outside_pose.position.x = 25.0;
  outside_pose.position.y = 25.0;
  EXPECT_FALSE(sampler.isPointValid(50, 50, region, outside_pose, 0.0f));
}

TEST(GoalSamplerTest, orientationFromRequestVariants) {
  GoalSampler sampler;
  auto region = makeSquareRegion("r", 0.0, 0.0, 1.0);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  pose.orientation.w = 1.0;

  sampler.orientationFromRequest(pose, region, GenerateRandomGoals::Request::OUTSIDE, 0.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 0.0);

  sampler.orientationFromRequest(pose, region, GenerateRandomGoals::Request::INSIDE, 0.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), M_PI);

  sampler.orientationFromRequest(pose, region, GenerateRandomGoals::Request::REQUESTED, 1.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 1.0);

  // RANDOM (and empty) must keep whatever yaw the caller already sampled
  pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.75));
  sampler.orientationFromRequest(pose, region, GenerateRandomGoals::Request::RANDOM, 0.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 0.75);
}

TEST(GoalSamplerTest, generateRandomGoalsProducesRequestedCountAndOrientation) {
  GoalSampler sampler;
  sampler.setMap(makeFreeMap(10, 10, 0.5));
  sampler.seed(42);
  // The region sits at the map's corner: disable the inflated footprint so its cells are not
  // all considered in collision with the outside of the map.
  sampler.setInflationRadius(0.0);
  sampler.setInflatedFootprintSize(0.0, 0.5);
  auto region = makeSquareRegion("r", 0.0, 0.0, 1.0);
  auto limits = sampler.processBoundingBox(sampler.getMap(), region);

  auto goals = sampler.generateRandomGoals(
    3, region, limits, GenerateRandomGoals::Request::REQUESTED, 1.0, 0.0f, "map", rclcpp::Time());

  EXPECT_EQ(goals.size(), 3u);
  for (const auto & goal : goals) {
    EXPECT_EQ(goal.header.frame_id, "map");
    EXPECT_DOUBLE_EQ(tf2::getYaw(goal.pose.orientation), 1.0);
  }
}

TEST(GoalSamplerTest, generateRandomGoalsTerminatesWhenRegionIsUnreachable) {
  GoalSampler sampler;
  auto map = makeFreeMap(10, 10, 0.5);
  map.data = std::vector<int8_t>(map.data.size(), nav2_util::OCC_GRID_OCCUPIED);
  sampler.setMap(map);
  sampler.setInflationRadius(0.0);
  sampler.setInflatedFootprintSize(0.0, 0.5);
  auto region = makeSquareRegion("r", 0.0, 0.0, 1.0);
  auto limits = sampler.processBoundingBox(sampler.getMap(), region);

  // Every cell is occupied: before the attempts cap, this would loop forever
  auto goals = sampler.generateRandomGoals(
    5, region, limits, GenerateRandomGoals::Request::INSIDE, 0.0, 0.0f, "map", rclcpp::Time());

  EXPECT_LT(goals.size(), 5u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
