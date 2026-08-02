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

#ifndef SEMANTIC_NAVIGATION_TASKS__REGIONS_LOADER_HPP_
#define SEMANTIC_NAVIGATION_TASKS__REGIONS_LOADER_HPP_

#include <string>
#include <vector>

#include "semantic_navigation_tasks/region.hpp"
#include "semantic_navigation_tasks/region_graph.hpp"

namespace semantic_navigation
{

/**
 * @struct semantic_navigation::RegionsFileData
 * @brief Result of parsing a regions YAML file: the regions, the manual connectivity overrides,
 * and a human-readable warning for every malformed entry that was skipped or every I/O/parse
 * error, so the caller can decide how to report them (log, propagate, ignore in tests, ...).
 */
struct RegionsFileData
{
  std::vector<Region> regions;
  std::vector<Connection> add_edges;
  std::vector<Connection> remove_edges;
  std::vector<std::string> warnings;
  // True if at least one valid region was loaded.
  bool success = false;
};

/**
 * @brief Load the regions and their manual connectivity overrides from a single read of a YAML
 * file.
 *
 * The file is parsed exactly once. Each region must have a `name` and at least 3 `points`, each
 * with exactly 2 coordinates; malformed entries are skipped and reported as a warning instead of
 * aborting the whole file. The `connections` section (with optional `add`/`remove` lists of
 * region-name pairs) is optional; a missing section is not an error.
 *
 * This function has no ROS dependency, so it can be unit tested without rclcpp::init().
 *
 * @param filename Path to the regions YAML file.
 * @return RegionsFileData Parsed regions, connections and any warnings.
 */
RegionsFileData loadRegionsFile(const std::string & filename);

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__REGIONS_LOADER_HPP_
