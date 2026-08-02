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

#include <unordered_map>

#include "semantic_navigation_tasks/region_markers.hpp"

namespace semantic_navigation::region_markers
{

polygon_msgs::msg::Polygon2DCollection createPolygons(
  const std::vector<Region> & list, const std::string & frame_id, const rclcpp::Time & stamp)
{
  polygon_msgs::msg::Polygon2DCollection polygon_array;
  polygon_array.header.frame_id = frame_id;
  polygon_array.header.stamp = stamp;

  for (const auto & region : list) {
    polygon_array.polygons.push_back(region.polygon);
  }

  return polygon_array;
}

visualization_msgs::msg::MarkerArray createNames(
  const std::vector<Region> & list, const std::string & frame_id, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray names_array;
  for (const auto & region : list) {
    // Create label
    visualization_msgs::msg::Marker label_marker;
    label_marker.header.frame_id = frame_id;
    label_marker.header.stamp = stamp;
    label_marker.ns = "label_region";
    label_marker.id = names_array.markers.size();
    label_marker.text = region.name;
    label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label_marker.action = visualization_msgs::msg::Marker::ADD;
    label_marker.pose.position.x = region.centroid().x;
    label_marker.pose.position.y = region.centroid().y;
    label_marker.pose.position.z = 0.05;
    label_marker.pose.orientation.x = 0.0;
    label_marker.pose.orientation.y = 0.0;
    label_marker.pose.orientation.z = 0.0;
    label_marker.pose.orientation.w = 1.0;
    label_marker.scale.z = 0.5;
    label_marker.color.r = 1.0;
    label_marker.color.g = 1.0;
    label_marker.color.b = 1.0;
    label_marker.color.a = 1.0f;
    names_array.markers.push_back(label_marker);
  }
  return names_array;
}

visualization_msgs::msg::MarkerArray createEdges(
  const std::vector<Region> & list, const RegionGraph & graph, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray edges_array;

  // Index the centroids by region name for a quick lookup
  std::unordered_map<std::string, geometry_msgs::msg::Point> centroids;
  for (const auto & region : list) {
    if (!region.polygon.points.empty()) {
      centroids[region.name] = region.centroid();
    }
  }

  // Draw a single line list with one segment per undirected edge, avoiding duplicates
  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = frame_id;
  edge_marker.header.stamp = stamp;
  edge_marker.ns = "edges_region";
  edge_marker.id = 0;
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.05;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0f;

  for (const auto & region : list) {
    auto centroid_it = centroids.find(region.name);
    if (centroid_it == centroids.end()) {
      continue;
    }
    for (const auto & neighbor : graph.getNeighbors(region.name)) {
      // Each undirected edge is drawn only once
      if (region.name >= neighbor) {
        continue;
      }
      auto neighbor_it = centroids.find(neighbor);
      if (neighbor_it == centroids.end()) {
        continue;
      }
      edge_marker.points.push_back(centroid_it->second);
      edge_marker.points.push_back(neighbor_it->second);
    }
  }

  edges_array.markers.push_back(edge_marker);
  return edges_array;
}

}  // namespace semantic_navigation::region_markers
