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
#include "ament_index_cpp/get_package_share_path.hpp"
#include "semantic_navigation_tasks/regions_loader.hpp"

// This whole test file exercises pure domain logic: it never calls rclcpp::init() and never
// constructs a node, matching the "test the algorithm without a lifecycle node" goal.

TEST(RegionsLoaderTest, loadsValidRegionsAndSkipsMalformedOnes) {
  auto pkg = ament_index_cpp::get_package_share_path("semantic_navigation_tasks").string();
  auto data = semantic_navigation::loadRegionsFile(pkg + "/test/regions_test.yaml");

  EXPECT_TRUE(data.success);
  ASSERT_EQ(data.regions.size(), 4u);
  EXPECT_EQ(data.regions[0].name, "small1");
  EXPECT_EQ(data.regions[0].size(), 4);
  EXPECT_EQ(data.regions[1].name, "small2");
  EXPECT_EQ(data.regions[2].name, "big");
  EXPECT_EQ(data.regions[3].name, "outside");

  // The 4 trailing malformed entries (missing name/points, too few points, a bad point arity)
  // must be reported, not silently dropped.
  EXPECT_GE(data.warnings.size(), 5u);
}

TEST(RegionsLoaderTest, parsesManualConnections) {
  auto pkg = ament_index_cpp::get_package_share_path("semantic_navigation_tasks").string();
  auto data = semantic_navigation::loadRegionsFile(pkg + "/test/regions_test.yaml");

  ASSERT_EQ(data.add_edges.size(), 1u);
  EXPECT_EQ(data.add_edges[0].first, "small1");
  EXPECT_EQ(data.add_edges[0].second, "big");
  ASSERT_EQ(data.remove_edges.size(), 1u);
  EXPECT_EQ(data.remove_edges[0].first, "small1");
  EXPECT_EQ(data.remove_edges[0].second, "small2");
}

TEST(RegionsLoaderTest, emptyFileFailsWithoutCrashing) {
  auto pkg = ament_index_cpp::get_package_share_path("semantic_navigation_tasks").string();
  auto data = semantic_navigation::loadRegionsFile(pkg + "/test/regions_test_empty.yaml");

  EXPECT_FALSE(data.success);
  EXPECT_TRUE(data.regions.empty());
  EXPECT_TRUE(data.add_edges.empty());
  EXPECT_TRUE(data.remove_edges.empty());
  EXPECT_FALSE(data.warnings.empty());
}

TEST(RegionsLoaderTest, missingFileFailsWithoutCrashing) {
  auto data = semantic_navigation::loadRegionsFile("/no/such/file/regions.yaml");

  EXPECT_FALSE(data.success);
  EXPECT_TRUE(data.regions.empty());
  ASSERT_FALSE(data.warnings.empty());
  EXPECT_NE(data.warnings[0].find("/no/such/file/regions.yaml"), std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
