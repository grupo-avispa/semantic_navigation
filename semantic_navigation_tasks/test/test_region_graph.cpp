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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "polygon_msgs/msg/point2_d.hpp"
#include "semantic_navigation_tasks/region.hpp"
#include "semantic_navigation_tasks/region_graph.hpp"

using semantic_navigation::Connection;
using semantic_navigation::Region;
using semantic_navigation::RegionGraph;

/// Build a rectangular region defined by two opposite corners.
static Region makeRect(
  const std::string & name, float x0, float y0, float x1, float y1)
{
  Region region;
  region.name = name;
  std::vector<std::pair<float, float>> corners = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  for (const auto & corner : corners) {
    polygon_msgs::msg::Point2D point;
    point.x = corner.first;
    point.y = corner.second;
    region.polygon.points.push_back(point);
  }
  return region;
}

/// Three rooms in a row: a -- b -- c, plus an isolated d.
static std::vector<Region> makeRowRegions()
{
  return {
    makeRect("a", 0.0, 0.0, 1.0, 1.0),
    makeRect("b", 1.0, 0.0, 2.0, 1.0),
    makeRect("c", 2.0, 0.0, 3.0, 1.0),
    makeRect("d", 10.0, 10.0, 11.0, 11.0)
  };
}

TEST(RegionGraphTest, autoConnectDetectsAdjacency) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, true, {}, {});

  // Contiguous rooms share a border, so they are connected
  EXPECT_TRUE(graph.areConnected("a", "b"));
  EXPECT_TRUE(graph.areConnected("b", "c"));
  // The isolated room is not connected to anything
  EXPECT_FALSE(graph.areConnected("a", "d"));
  EXPECT_TRUE(graph.getNeighbors("d").empty());
  // a -- b -- c gives two edges
  EXPECT_EQ(graph.edgeCount(), 2u);
}

TEST(RegionGraphTest, autoConnectDisabled) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, false, {}, {});

  EXPECT_EQ(graph.edgeCount(), 0u);
  EXPECT_FALSE(graph.areConnected("a", "b"));
  // The regions are still registered as nodes
  EXPECT_TRUE(graph.hasRegion("a"));
  EXPECT_TRUE(graph.hasRegion("d"));
}

TEST(RegionGraphTest, manualAddOverride) {
  RegionGraph graph;
  // Force a connection to the otherwise isolated room d
  std::vector<Connection> add = {{"c", "d"}};
  graph.build(makeRowRegions(), 0.1, true, add, {});

  EXPECT_TRUE(graph.areConnected("c", "d"));
  EXPECT_TRUE(graph.areConnected("a", "d"));
}

TEST(RegionGraphTest, manualRemoveOverride) {
  RegionGraph graph;
  // Forbid the connection between b and c (e.g. a wall)
  std::vector<Connection> remove = {{"b", "c"}};
  graph.build(makeRowRegions(), 0.1, true, {}, remove);

  EXPECT_TRUE(graph.areConnected("a", "b"));
  EXPECT_FALSE(graph.areConnected("b", "c"));
  EXPECT_FALSE(graph.areConnected("a", "c"));
}

TEST(RegionGraphTest, getNeighborsSorted) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, true, {}, {});

  auto neighbors = graph.getNeighbors("b");
  ASSERT_EQ(neighbors.size(), 2u);
  EXPECT_EQ(neighbors[0], "a");
  EXPECT_EQ(neighbors[1], "c");
  // An unknown region has no neighbors
  EXPECT_TRUE(graph.getNeighbors("unknown").empty());
}

TEST(RegionGraphTest, addAndRemoveEdge) {
  RegionGraph graph;
  graph.addEdge("x", "y");
  EXPECT_TRUE(graph.areConnected("x", "y"));
  EXPECT_EQ(graph.edgeCount(), 1u);

  // Self-edges are ignored
  graph.addEdge("x", "x");
  EXPECT_EQ(graph.edgeCount(), 1u);

  graph.removeEdge("x", "y");
  EXPECT_FALSE(graph.areConnected("x", "y"));
  EXPECT_EQ(graph.edgeCount(), 0u);
}

TEST(RegionGraphTest, findRouteMultiHop) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, true, {}, {});

  auto route = graph.findRoute("a", "c");
  ASSERT_EQ(route.size(), 3u);
  EXPECT_EQ(route[0], "a");
  EXPECT_EQ(route[1], "b");
  EXPECT_EQ(route[2], "c");
}

TEST(RegionGraphTest, findRouteDirect) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, true, {}, {});

  auto route = graph.findRoute("a", "b");
  ASSERT_EQ(route.size(), 2u);
  EXPECT_EQ(route[0], "a");
  EXPECT_EQ(route[1], "b");
}

TEST(RegionGraphTest, findRouteSameRegion) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, true, {}, {});

  auto route = graph.findRoute("a", "a");
  ASSERT_EQ(route.size(), 1u);
  EXPECT_EQ(route[0], "a");
}

TEST(RegionGraphTest, findRouteUnreachable) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, true, {}, {});

  // d is isolated
  EXPECT_TRUE(graph.findRoute("a", "d").empty());
  // Unknown regions yield no route
  EXPECT_TRUE(graph.findRoute("a", "unknown").empty());
  EXPECT_TRUE(graph.findRoute("unknown", "a").empty());
}

TEST(RegionGraphTest, clear) {
  RegionGraph graph;
  graph.build(makeRowRegions(), 0.1, true, {}, {});
  ASSERT_GT(graph.edgeCount(), 0u);

  graph.clear();
  EXPECT_EQ(graph.edgeCount(), 0u);
  EXPECT_FALSE(graph.hasRegion("a"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
