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

#ifndef SEMANTIC_NAVIGATION_TASKS__REGION_GRAPH_HPP_
#define SEMANTIC_NAVIGATION_TASKS__REGION_GRAPH_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "semantic_navigation_tasks/region.hpp"

namespace semantic_navigation
{

/// A connection between two regions, expressed as a pair of region names.
using Connection = std::pair<std::string, std::string>;

/**
 * @class semantic_navigation::RegionGraph
 * @brief Undirected, unweighted topological graph that models the connectivity between regions.
 *
 * The graph is built once from a list of regions. Connectivity can be detected automatically
 * from the geometry of the polygons (two regions are connected when their borders are closer
 * than a threshold) and refined with manual edges that are added or removed.
 */
class RegionGraph
{
public:
  /**
   * @brief Construct a new empty Region Graph object.
   */
  RegionGraph() = default;

  /**
   * @brief Build the connectivity graph from a list of regions.
   *
   * When @p auto_connect is true, every pair of regions whose borders are closer than
   * @p threshold metres is connected. After the automatic detection, the @p add edges are
   * forced and the @p remove edges are deleted, in that order.
   *
   * @param regions List of regions of interest.
   * @param threshold Maximum distance (in metres) between two borders to consider them connected.
   * @param auto_connect Whether to detect connectivity automatically from the geometry.
   * @param add Edges to force regardless of the geometry (e.g. a door between distant regions).
   * @param remove Edges to forbid regardless of the geometry (e.g. a wall between adjacent rooms).
   */
  void build(
    const std::vector<Region> & regions, double threshold, bool auto_connect,
    const std::vector<Connection> & add, const std::vector<Connection> & remove);

  /**
   * @brief Clear the graph, removing all the nodes and edges.
   */
  void clear();

  /**
   * @brief Add an undirected edge between two regions.
   *
   * Both regions are registered as nodes if they were not present. Self-edges are ignored.
   *
   * @param a Name of the first region.
   * @param b Name of the second region.
   */
  void addEdge(const std::string & a, const std::string & b);

  /**
   * @brief Remove the undirected edge between two regions, if it exists.
   *
   * @param a Name of the first region.
   * @param b Name of the second region.
   */
  void removeEdge(const std::string & a, const std::string & b);

  /**
   * @brief Get the regions directly connected to a given region.
   *
   * @param name Name of the region.
   * @return Names of the directly connected regions (empty if the region is unknown).
   */
  std::vector<std::string> getNeighbors(const std::string & name) const;

  /**
   * @brief Check whether two regions are connected, directly or transitively.
   *
   * @param a Name of the first region.
   * @param b Name of the second region.
   * @return True if there is a path between both regions.
   */
  bool areConnected(const std::string & a, const std::string & b) const;

  /**
   * @brief Find the shortest topological route (in number of hops) between two regions.
   *
   * @param start Name of the start region.
   * @param goal Name of the goal region.
   * @return Ordered sequence of regions from @p start to @p goal, or an empty vector if there
   * is no route. If @p start equals @p goal and the region exists, the route contains it once.
   */
  std::vector<std::string> findRoute(const std::string & start, const std::string & goal) const;

  /**
   * @brief Check if a region is a node of the graph.
   *
   * @param name Name of the region.
   * @return True if the region exists in the graph.
   */
  bool hasRegion(const std::string & name) const;

  /**
   * @brief Get the number of edges in the graph.
   *
   * @return Number of undirected edges.
   */
  size_t edgeCount() const;

protected:
  /**
   * @brief Compute the minimum distance between the borders of two regions.
   *
   * @param a First region.
   * @param b Second region.
   * @return Minimum distance between any border of @p a and any border of @p b.
   */
  double minDistanceBetweenRegions(const Region & a, const Region & b) const;

  std::unordered_map<std::string, std::unordered_set<std::string>> adjacency_;
};

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__REGION_GRAPH_HPP_
