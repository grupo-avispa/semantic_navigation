/*
 * SEMANTIC ANNOTATION RVIZ TOOL
 *
 * Copyright (c) 2021-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of semantic_navigation.
 * 
 * All rights reserved.
 *
 */

#ifndef SEMANTIC_NAVIGATION__SEMANTIC_ANNOTATION_TOOL_HPP_
#define SEMANTIC_NAVIGATION__SEMANTIC_ANNOTATION_TOOL_HPP_

#include <OgreVector3.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "polygon_msgs/msg/polygon2_d_collection.hpp"
#include "slg_msgs/polygon.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rviz_common{
	namespace properties{
		class FloatProperty;
		class StringProperty;
	}
	class VisualizationManager;
	class ViewportMouseEvent;
}

namespace semantic_navigation_rviz_plugin{

class semanticAnnotationTool: public rviz_common::Tool{
Q_OBJECT
	public:
		explicit semanticAnnotationTool();
		~semanticAnnotationTool() override = default;
		void onInitialize() override;
		void activate() override;
		void deactivate() override;
		int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

	public Q_SLOTS:
		void update_property();

	protected:
		std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

	private:
		rclcpp::Node::SharedPtr ros_node_;
		rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygons_viz_pub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr names_viz_pub_;
		rviz_common::properties::FloatProperty* inflation_property_;
		rviz_common::properties::StringProperty* names_property_;
		rviz_common::properties::StringProperty* filename_property_;

		std::vector<slg::Polygon> polygons_;
		std::vector<std::string> names_;
		bool new_polygon_;
		float inflation_radius_;
		std::string filename_;

		void save_polygon(const std::string filename);
		void show_polygon_names();
};

}  // namespace semantic_navigation_rviz_plugin

#endif 
