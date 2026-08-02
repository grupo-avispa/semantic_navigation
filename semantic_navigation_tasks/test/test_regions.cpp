// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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
#include "polygon_msgs/msg/point2_d.hpp"
#include "semantic_navigation_tasks/region.hpp"

TEST(SemanticRegionTest, constructor) {
  semantic_navigation::Region region;
  EXPECT_EQ(region.name, "");
  EXPECT_TRUE(region.empty());
}

TEST(SemanticRegionTest, settersAndGetters) {
  semantic_navigation::Region region;

  // Set values
  region.name = "test";
  polygon_msgs::msg::Point2D point;
  point.x = 1.0; point.y = 2.0;
  region.polygon.points.push_back(point);
  point.x = 3.0; point.y = 4.0;
  region.polygon.points.push_back(point);

  // Check results
  EXPECT_EQ(region.name, "test");
  EXPECT_EQ(region.polygon.points.size(), 2);
  EXPECT_EQ(region.polygon.points[0].x, 1.0);
  EXPECT_EQ(region.polygon.points[0].y, 2.0);
  EXPECT_EQ(region.polygon.points[1].x, 3.0);
  EXPECT_EQ(region.polygon.points[1].y, 4.0);
}

TEST(SemanticRegionTest, centroid) {
  semantic_navigation::Region region;
  polygon_msgs::msg::Point2D point;
  point.x = 0.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 1.0;
  region.polygon.points.push_back(point);
  point.x = 0.0; point.y = 1.0;
  region.polygon.points.push_back(point);

  // Check the centroid of the polygon
  geometry_msgs::msg::Point centroid = region.centroid();
  EXPECT_EQ(centroid.x, 0.5);
  EXPECT_EQ(centroid.y, 0.5);
}

TEST(SemanticRegionTest, isPointInside) {
  semantic_navigation::Region region;
  polygon_msgs::msg::Point2D point;
  point.x = 0.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 1.0;
  region.polygon.points.push_back(point);
  point.x = 0.0; point.y = 1.0;
  region.polygon.points.push_back(point);

  // Check if the point (0.5, 0.5) is inside the polygon
  EXPECT_TRUE(region.isPointInside(0.5, 0.5));

  // Check if the point (2.0, 2.0) is inside the polygon
  EXPECT_FALSE(region.isPointInside(2.0, 2.0));
}

TEST(SemanticRegionTest, isPointAtLeastDistanceFromBorders) {
  semantic_navigation::Region region;
  polygon_msgs::msg::Point2D point;
  point.x = 0.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 0.0;
  region.polygon.points.push_back(point);
  point.x = 1.0; point.y = 1.0;
  region.polygon.points.push_back(point);
  point.x = 0.0; point.y = 1.0;
  region.polygon.points.push_back(point);

  // Check if the point (0.2, 0.2) is at a distance greater than the border
  EXPECT_FALSE(region.isPointAtLeastDistanceFromBorders(0.2, 0.2, 0.5));
  // Check if the point (2.0, 2.0) is at a distance greater than the border
  EXPECT_TRUE(region.isPointAtLeastDistanceFromBorders(2.0, 2.0, 0.1));
  // Check if the point (0.2, 0.7) is at a distance greater than the border
  EXPECT_FALSE(region.isPointAtLeastDistanceFromBorders(0.2, 0.7, 0.25));
}

TEST(SemanticRegionTest, isPointAtLeastDistanceFromBordersDegenerate) {
  // A region with no points (e.g. loaded from a malformed `points: []` entry) must not crash
  semantic_navigation::Region empty_region;
  EXPECT_TRUE(empty_region.isPointAtLeastDistanceFromBorders(0.0, 0.0, 0.5));

  // Neither should a region with a single point
  semantic_navigation::Region single_point_region;
  polygon_msgs::msg::Point2D point;
  point.x = 0.0; point.y = 0.0;
  single_point_region.polygon.points.push_back(point);
  EXPECT_TRUE(single_point_region.isPointAtLeastDistanceFromBorders(1.0, 1.0, 0.5));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
