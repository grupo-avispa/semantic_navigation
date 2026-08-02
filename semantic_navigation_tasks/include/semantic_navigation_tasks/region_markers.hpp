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

#ifndef SEMANTIC_NAVIGATION_TASKS__REGION_MARKERS_HPP_
#define SEMANTIC_NAVIGATION_TASKS__REGION_MARKERS_HPP_

#include <string>
#include <vector>

#include "polygon_msgs/msg/polygon2_d_collection.hpp"
#include "rclcpp/time.hpp"
#include "semantic_navigation_tasks/region.hpp"
#include "semantic_navigation_tasks/region_graph.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace semantic_navigation::region_markers
{

/**
 * @brief Create a collection of polygons for visualization.
 *
 * @param list List of regions of interest.
 * @param frame_id Frame id stamped on the collection.
 * @param stamp Timestamp stamped on the collection.
 * @return polygon_msgs::msg::Polygon2DCollection Collection of polygons.
 */
polygon_msgs::msg::Polygon2DCollection createPolygons(
  const std::vector<Region> & list, const std::string & frame_id, const rclcpp::Time & stamp);

/**
 * @brief Create a collection of markers with the names of the regions.
 *
 * @param list List of regions of interest.
 * @param frame_id Frame id stamped on each marker.
 * @param stamp Timestamp stamped on each marker.
 * @return visualization_msgs::msg::MarkerArray Collection of markers.
 */
visualization_msgs::msg::MarkerArray createNames(
  const std::vector<Region> & list, const std::string & frame_id, const rclcpp::Time & stamp);

/**
 * @brief Create a collection of markers with the edges of the connectivity graph.
 *
 * Each edge is drawn as a line between the centroids of the two connected regions.
 *
 * @param list List of regions of interest.
 * @param graph Connectivity graph between the regions.
 * @param frame_id Frame id stamped on the marker.
 * @param stamp Timestamp stamped on the marker.
 * @return visualization_msgs::msg::MarkerArray Collection of markers.
 */
visualization_msgs::msg::MarkerArray createEdges(
  const std::vector<Region> & list, const RegionGraph & graph, const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace semantic_navigation::region_markers

#endif  // SEMANTIC_NAVIGATION_TASKS__REGION_MARKERS_HPP_
