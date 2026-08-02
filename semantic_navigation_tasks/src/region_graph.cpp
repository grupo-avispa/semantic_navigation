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
#include <limits>
#include <queue>

#include "semantic_navigation_tasks/geometry_utils.hpp"
#include "semantic_navigation_tasks/region_graph.hpp"

namespace semantic_navigation
{

void RegionGraph::build(
  const std::vector<Region> & regions, double threshold, bool auto_connect,
  const std::vector<Connection> & add, const std::vector<Connection> & remove)
{
  clear();

  // Register every region as a node, even if it ends up isolated
  for (const auto & region : regions) {
    adjacency_.try_emplace(region.name);
  }

  // Detect connectivity automatically from the geometry of the polygons
  if (auto_connect) {
    for (size_t i = 0; i < regions.size(); ++i) {
      for (size_t j = i + 1; j < regions.size(); ++j) {
        if (minDistanceBetweenRegions(regions[i], regions[j]) <= threshold) {
          addEdge(regions[i].name, regions[j].name);
        }
      }
    }
  }

  // Apply the manual overrides: first force the added edges, then forbid the removed ones
  for (const auto & edge : add) {
    addEdge(edge.first, edge.second);
  }
  for (const auto & edge : remove) {
    removeEdge(edge.first, edge.second);
  }
}

void RegionGraph::clear()
{
  adjacency_.clear();
}

void RegionGraph::addEdge(const std::string & a, const std::string & b)
{
  if (a == b) {
    return;
  }
  adjacency_[a].insert(b);
  adjacency_[b].insert(a);
}

void RegionGraph::removeEdge(const std::string & a, const std::string & b)
{
  auto it_a = adjacency_.find(a);
  if (it_a != adjacency_.end()) {
    it_a->second.erase(b);
  }
  auto it_b = adjacency_.find(b);
  if (it_b != adjacency_.end()) {
    it_b->second.erase(a);
  }
}

std::vector<std::string> RegionGraph::getNeighbors(const std::string & name) const
{
  std::vector<std::string> neighbors;
  auto it = adjacency_.find(name);
  if (it != adjacency_.end()) {
    neighbors.assign(it->second.begin(), it->second.end());
    std::sort(neighbors.begin(), neighbors.end());
  }
  return neighbors;
}

bool RegionGraph::areConnected(const std::string & a, const std::string & b) const
{
  return !findRoute(a, b).empty();
}

std::vector<std::string> RegionGraph::findRoute(
  const std::string & start, const std::string & goal) const
{
  std::vector<std::string> route;

  // Both endpoints must be known regions
  if (adjacency_.find(start) == adjacency_.end() ||
    adjacency_.find(goal) == adjacency_.end())
  {
    return route;
  }

  if (start == goal) {
    route.push_back(start);
    return route;
  }

  // Breadth-first search keeping the predecessor of each visited region
  std::unordered_map<std::string, std::string> predecessor;
  std::queue<std::string> frontier;
  frontier.push(start);
  predecessor[start] = start;

  bool found = false;
  while (!frontier.empty() && !found) {
    const std::string current = frontier.front();
    frontier.pop();

    for (const auto & neighbor : adjacency_.at(current)) {
      if (predecessor.find(neighbor) != predecessor.end()) {
        continue;
      }
      predecessor[neighbor] = current;
      if (neighbor == goal) {
        found = true;
        break;
      }
      frontier.push(neighbor);
    }
  }

  if (!found) {
    return route;
  }

  // Rebuild the route walking the predecessors backwards from the goal
  for (std::string node = goal; node != start; node = predecessor.at(node)) {
    route.push_back(node);
  }
  route.push_back(start);
  std::reverse(route.begin(), route.end());
  return route;
}

bool RegionGraph::hasRegion(const std::string & name) const
{
  return adjacency_.find(name) != adjacency_.end();
}

size_t RegionGraph::edgeCount() const
{
  size_t half_edges = 0;
  for (const auto & node : adjacency_) {
    half_edges += node.second.size();
  }
  return half_edges / 2;
}

double RegionGraph::minDistanceBetweenRegions(const Region & a, const Region & b) const
{
  const auto & pa = a.polygon.points;
  const auto & pb = b.polygon.points;
  if (pa.empty() || pb.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();

  // Distance from every vertex of A to every border of B
  for (const auto & point : pa) {
    for (size_t i = 0; i < pb.size(); ++i) {
      const auto & p0 = pb[i];
      const auto & p1 = pb[(i + 1) % pb.size()];
      min_distance = std::min(
        min_distance,
        geometry_utils::pointToSegmentDistance(point.x, point.y, p0.x, p0.y, p1.x, p1.y));
    }
  }

  // Distance from every vertex of B to every border of A
  for (const auto & point : pb) {
    for (size_t i = 0; i < pa.size(); ++i) {
      const auto & p0 = pa[i];
      const auto & p1 = pa[(i + 1) % pa.size()];
      min_distance = std::min(
        min_distance,
        geometry_utils::pointToSegmentDistance(point.x, point.y, p0.x, p0.y, p1.x, p1.y));
    }
  }

  return min_distance;
}

}  // namespace semantic_navigation
