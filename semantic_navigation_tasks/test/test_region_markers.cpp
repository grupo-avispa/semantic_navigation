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
#include "semantic_navigation_tasks/region_markers.hpp"

// Pure domain logic: no rclcpp::init() and no node are needed to exercise these free functions.

using semantic_navigation::Region;
using semantic_navigation::RegionGraph;
namespace region_markers = semantic_navigation::region_markers;

namespace
{

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

TEST(RegionMarkersTest, createPolygonsCopiesPolygonsAndStampsHeader) {
  std::vector<Region> regions = {makeSquareRegion("r1", 0.0, 0.0, 1.0)};

  auto collection = region_markers::createPolygons(regions, "map", rclcpp::Time());

  EXPECT_EQ(collection.header.frame_id, "map");
  ASSERT_EQ(collection.polygons.size(), 1u);
  EXPECT_EQ(collection.polygons[0].points.size(), 4u);
}

TEST(RegionMarkersTest, createNamesOneMarkerPerRegionAtItsCentroid) {
  std::vector<Region> regions = {
    makeSquareRegion("r1", 0.0, 0.0, 1.0),
    makeSquareRegion("r2", 5.0, 5.0, 2.0),
  };

  auto names = region_markers::createNames(regions, "map", rclcpp::Time());

  ASSERT_EQ(names.markers.size(), 2u);
  EXPECT_EQ(names.markers[0].text, "r1");
  EXPECT_DOUBLE_EQ(names.markers[0].pose.position.x, regions[0].centroid().x);
  EXPECT_DOUBLE_EQ(names.markers[0].pose.position.y, regions[0].centroid().y);
  EXPECT_EQ(names.markers[1].text, "r2");
}

TEST(RegionMarkersTest, createEdgesDrawsOneSegmentPerUndirectedEdge) {
  std::vector<Region> regions = {
    makeSquareRegion("r1", 0.0, 0.0, 1.0),
    makeSquareRegion("r2", 5.0, 5.0, 1.0),
    makeSquareRegion("r3", 10.0, 10.0, 1.0),
  };
  RegionGraph graph;
  graph.addEdge("r1", "r2");

  auto edges = region_markers::createEdges(regions, graph, "map", rclcpp::Time());

  ASSERT_EQ(edges.markers.size(), 1u);
  // A single line-list marker with exactly one segment (two points): r1-r2
  EXPECT_EQ(edges.markers[0].points.size(), 2u);
}

TEST(RegionMarkersTest, createEdgesSkipsRegionsWithoutPoints) {
  std::vector<Region> regions = {makeSquareRegion("r1", 0.0, 0.0, 1.0), Region()};
  regions[1].name = "empty";
  RegionGraph graph;
  graph.addEdge("r1", "empty");

  auto edges = region_markers::createEdges(regions, graph, "map", rclcpp::Time());

  ASSERT_EQ(edges.markers.size(), 1u);
  EXPECT_TRUE(edges.markers[0].points.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
