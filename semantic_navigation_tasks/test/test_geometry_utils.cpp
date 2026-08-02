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
#include "semantic_navigation_tasks/geometry_utils.hpp"

using semantic_navigation::geometry_utils::pointToSegmentDistance;

TEST(GeometryUtilsTest, pointProjectsInsideSegment) {
  // The closest point on the segment (0,0)-(2,0) to (1,1) is (1,0)
  EXPECT_DOUBLE_EQ(pointToSegmentDistance(1.0, 1.0, 0.0, 0.0, 2.0, 0.0), 1.0);
}

TEST(GeometryUtilsTest, pointProjectsBeforeSegmentStart) {
  // The closest point is clamped to the segment start (0,0)
  EXPECT_DOUBLE_EQ(pointToSegmentDistance(-1.0, 0.0, 0.0, 0.0, 2.0, 0.0), 1.0);
}

TEST(GeometryUtilsTest, pointProjectsAfterSegmentEnd) {
  // The closest point is clamped to the segment end (2,0)
  EXPECT_DOUBLE_EQ(pointToSegmentDistance(3.0, 0.0, 0.0, 0.0, 2.0, 0.0), 1.0);
}

TEST(GeometryUtilsTest, degenerateSegmentDoesNotDivideByZero) {
  // Both endpoints coincide: the distance must fall back to point-to-point
  EXPECT_DOUBLE_EQ(pointToSegmentDistance(3.0, 4.0, 0.0, 0.0, 0.0, 0.0), 5.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
