// Copyright (c) 2020 Alberto J. Tudela Roldán
// Copyright (c) 2020 Grupo Avispa, DTE, Universidad de Málaga
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

// C++
#include <fstream>

// OGRE
#include <OgrePlane.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreViewport.h>

// ROS
#include "ament_index_cpp/get_package_share_path.hpp"
#include "nav2_util/string_utils.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/render_window.hpp"
#include "polygon_utils/polygon_utils.hpp"

#include "semantic_navigation_rviz_plugins/semantic_annotation_tool.hpp"

namespace semantic_navigation_rviz_plugins
{

SemanticAnnotationTool::SemanticAnnotationTool()
: rviz_common::Tool()
{
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  shortcut_key_ = 's';
  inflation_property_ = new rviz_common::properties::FloatProperty(
    "Inflation radius", 0.5, "Inflation radius",
    getPropertyContainer(), SLOT(update_property()), this);
  names_property_ = new rviz_common::properties::StringProperty(
    "Regions names", "", "List of regions names",
    getPropertyContainer(), SLOT(update_property()), this);
  filename_property_ = new rviz_common::properties::StringProperty(
    "YAML config filename", QString::fromStdString("default"),
    "Filename to save the regions located in the share folder.",
    getPropertyContainer(), SLOT(update_property()), this);
}

void SemanticAnnotationTool::onInitialize()
{
  auto lock = context_->getRosNodeAbstraction().lock();
  ros_node_ = lock->get_raw_node();

  polygons_viz_pub_ = ros_node_->create_publisher<polygon_msgs::msg::Polygon2DCollection>(
    "polygons", rclcpp::QoS(1).transient_local());
  names_viz_pub_ = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "names", rclcpp::QoS(1).transient_local());

  new_polygon_ = true;
  update_property();
}

void SemanticAnnotationTool::update_property()
{
  inflation_radius_ = inflation_property_->getFloat();
  filename_ = filename_property_->getStdString();

  // Convert string "list" to vector of strings
  std::string name_list = names_property_->getStdString();
  if (name_list.empty()) {
    names_.clear();
  } else {
    if (name_list.back() == ' ' || name_list.back() == ',') {name_list.pop_back();}
    names_ = nav2_util::split(name_list, ',');
  }
}

void SemanticAnnotationTool::activate()
{
  onInitialize();
  RCLCPP_INFO(ros_node_->get_logger(), "Semantic annotation tool started!");
}

void SemanticAnnotationTool::deactivate()
{
}

int SemanticAnnotationTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  polygon_msgs::msg::Polygon2DCollection polygon_array;
  polygon_array.header.frame_id = "map";
  polygon_array.header.stamp = ros_node_->now();
  semantic_navigation::Region current_region;

  auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

  // Add points to the region with left button
  if (event.leftDown()) {
    // Extract the last region
    if (!new_polygon_) {
      current_region = region_list_.back();
      region_list_.pop_back();
    }
    // Capture the point from the map
    polygon_msgs::msg::Point2D point;
    point.x = point_projection_on_xy_plane.second.x;
    point.y = point_projection_on_xy_plane.second.y;
    // Create the new polygon
    current_region.polygon.points.push_back(point);
    region_list_.push_back(current_region);
    // Check the name vector with the bigger size and resize and fill the other one
    if (names_.size() < region_list_.size()) {
      names_.resize(region_list_.size(), "unknown");
    }
    // Publish it
    for (unsigned int i = 0; i < region_list_.size(); i++) {
      region_list_[i].name = names_[i];
      polygon_array.polygons.push_back(region_list_[i].polygon);
    }
    polygons_viz_pub_->publish(polygon_array);

    // Show names
    show_polygon_names();
    new_polygon_ = false;
    // Add new polygon with right button
  } else if (event.rightUp()) {
    new_polygon_ = true;
    // Save and clear the polygons with central button
  } else if (event.middleUp()) {
    save_polygon(filename_);
    region_list_.clear();
    polygons_viz_pub_->publish(polygon_array);
    show_polygon_names();
    new_polygon_ = true;
  }

  return 0;
}

void SemanticAnnotationTool::save_polygon(const std::string filename)
{
  std::filesystem::path pkg_path =
    ament_index_cpp::get_package_share_path("semantic_navigation_tasks");
  std::string filepath = std::string(pkg_path) + "/params/" + filename + ".yaml";
  std::ofstream regionfile(filepath, std::ofstream::app);

  regionfile << "regions:" << std::endl;
  for (const auto & region : region_list_) {
    regionfile << "  - {name: '" << region.name << "', points: [";
    auto points = region.polygon.points;
    for (unsigned int p = 0; p < points.size() - 1; p++) {
      regionfile << "[" << points[p].x << ", " << points[p].y << "], ";
    }
    regionfile << "[" << points.back().x << ", " << points.back().y << "]]}" << std::endl;
  }
  regionfile << "\n";
  regionfile.close();

  RCLCPP_INFO(ros_node_->get_logger(), "Regions saved in %s", filepath.c_str());
}

void SemanticAnnotationTool::show_polygon_names()
{
  visualization_msgs::msg::MarkerArray names_array;
  int p = 0;
  for (auto & region : region_list_) {
    // Create label
    visualization_msgs::msg::Marker label_marker;
    label_marker.header.frame_id = "map";
    label_marker.header.stamp = ros_node_->now();
    label_marker.ns = "label";
    label_marker.id = p;
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
    p++;
  }

  if (region_list_.empty()) {
    visualization_msgs::msg::Marker label_marker;
    label_marker.header.frame_id = "map";
    label_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    names_array.markers.push_back(label_marker);
  }
  names_viz_pub_->publish(names_array);
}

}  // namespace semantic_navigation_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(semantic_navigation_rviz_plugins::SemanticAnnotationTool, rviz_common::Tool)
