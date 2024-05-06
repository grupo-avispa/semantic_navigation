// Copyright (c) 2020 Alberto J. Tudela Roldán
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

#ifndef SEMANTIC_NAVIGATION_RVIZ_PLUGINS__SEMANTIC_ANNOTATION_TOOL_HPP_
#define SEMANTIC_NAVIGATION_RVIZ_PLUGINS__SEMANTIC_ANNOTATION_TOOL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Ogre.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "polygon_msgs/msg/polygon2_d_collection.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "semantic_navigation_tasks/region.hpp"

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class StringProperty;
}
class VisualizationManager;
class ViewportMouseEvent;
}

namespace semantic_navigation_rviz_plugins
{

/**
 * @class semantic_navigation_rviz_plugins::SemanticAnnotationTool
 * @brief Tool to annotate regions of interest (ROIs) in the map.
 */
class SemanticAnnotationTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  /**
   * @brief Constructor.
   */
  SemanticAnnotationTool();

  /**
   * @brief Destructor.
   */
  ~SemanticAnnotationTool() override = default;

  /**
   * @brief Initialize the tool.
   */
  void onInitialize() override;

  /**
   * @brief Activate the tool.
   */
  void activate() override;

  /**
   * @brief Deactivate the tool.
   */
  void deactivate() override;

  /**
   * @brief Process the mouse event.
   * - Click with the left button on the map to add new points to a polygon.
   * - Click with the right button to start a new polygon.
   * - Click with the middle button to erase all polygons, save them and start over.
   * @param event Mouse event
   */
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

public Q_SLOTS:
  /**
   * @brief Update the properties of the tool.
   */
  void update_property();

protected:
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  /**
   * @brief Save the polygons in a YAML file.
   * @param filename Name of the file.
   */
  void save_polygon(const std::string filename);

  /**
   * @brief Show the names of the polygons in the map.
   */
  void show_polygon_names();

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygons_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr names_viz_pub_;
  rviz_common::properties::FloatProperty * inflation_property_;
  rviz_common::properties::StringProperty * names_property_;
  rviz_common::properties::StringProperty * filename_property_;

  std::vector<semantic_navigation::Region> region_list_;
  std::vector<std::string> names_;
  bool new_polygon_;
  float inflation_radius_;
  std::string filename_;
};

}  // namespace semantic_navigation_rviz_plugins

#endif  // SEMANTIC_NAVIGATION_RVIZ_PLUGINS__SEMANTIC_ANNOTATION_TOOL_HPP_
