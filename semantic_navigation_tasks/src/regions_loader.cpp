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

#include <yaml-cpp/yaml.h>

#include "semantic_navigation_tasks/regions_loader.hpp"

namespace semantic_navigation
{

namespace
{

/**
 * @brief Parse the list of regions from an already-loaded YAML node.
 *
 * @param config Root YAML node of the regions file.
 * @param data Result accumulator: parsed regions are appended, warnings are recorded.
 */
void parseRegionsNode(const YAML::Node & config, RegionsFileData & data)
{
  if (!config["regions"]) {
    data.warnings.push_back("No regions found in file");
    return;
  }

  const size_t initial_size = data.regions.size();
  for (const auto & region : config["regions"]) {
    if (!region["name"] || !region["points"]) {
      data.warnings.push_back("Region is not well defined: missing name or points");
      continue;
    }
    if (region["points"].size() < 3) {
      data.warnings.push_back(
        "Region [" + region["name"].as<std::string>() + "] has fewer than 3 points, skipping");
      continue;
    }

    Region new_region;
    new_region.name = region["name"].as<std::string>();
    bool valid_points = true;
    for (const auto & point : region["points"]) {
      if (point.size() != 2) {
        data.warnings.push_back(
          "Region [" + new_region.name + "] has a point without exactly 2 coordinates, "
          "skipping");
        valid_points = false;
        break;
      }
      polygon_msgs::msg::Point2D new_point;
      new_point.x = point[0].as<float>();
      new_point.y = point[1].as<float>();
      new_region.polygon.points.push_back(new_point);
    }
    if (valid_points) {
      data.regions.push_back(new_region);
    }
  }
  data.success = data.regions.size() > initial_size;
}

/**
 * @brief Parse the manual connections (add / remove edges) from an already-loaded YAML node.
 *
 * The `connections` section is optional: a missing section is not an error, the add/remove lists
 * are simply left empty.
 *
 * @param config Root YAML node of the regions file.
 * @param data Result accumulator: parsed connections are appended, warnings are recorded.
 */
void parseConnectionsNode(const YAML::Node & config, RegionsFileData & data)
{
  if (!config["connections"]) {
    return;
  }

  // Lambda to parse a list of pairs of region names into a list of connections
  auto parse_edges = [&data](const YAML::Node & node, std::vector<Connection> & edges) {
      for (const auto & edge : node) {
        if (edge.size() == 2) {
          edges.emplace_back(edge[0].as<std::string>(), edge[1].as<std::string>());
        } else {
          data.warnings.push_back("A connection must be a pair of region names");
        }
      }
    };

  const auto & connections = config["connections"];
  if (connections["add"]) {
    parse_edges(connections["add"], data.add_edges);
  }
  if (connections["remove"]) {
    parse_edges(connections["remove"], data.remove_edges);
  }
}

}  // namespace

RegionsFileData loadRegionsFile(const std::string & filename)
{
  RegionsFileData data;
  try {
    YAML::Node config = YAML::LoadFile(filename);
    parseRegionsNode(config, data);
    parseConnectionsNode(config, data);
  } catch (const YAML::Exception & e) {
    data.warnings.push_back("Error reading file [" + filename + "]: " + e.what());
    data.success = false;
  }
  return data;
}

}  // namespace semantic_navigation
