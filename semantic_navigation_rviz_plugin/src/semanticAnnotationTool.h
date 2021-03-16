/*
 * SEMANTIC ANNOTATION RVIZ TOOL
 *
 * Copyright (c) 2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of semantic_navigation.
 * 
 * All rights reserved.
 *
 */

#ifndef SEMANTIC_ANNOTATION_TOOL_H
#define SEMANTIC_ANNOTATION_TOOL_H

#include "ros/ros.h"
#include <rviz/tool.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <laser_utils/polygon.h>

namespace Ogre{
	class SceneNode;
	class Vector3;
}

namespace rviz{
	class VectorProperty;
	class VisualizationManager;
	class ViewportMouseEvent;
}

namespace semantic_navigation_rviz_plugin{

class semanticAnnotationTool: public rviz::Tool{
Q_OBJECT
	public:
		semanticAnnotationTool();
		~semanticAnnotationTool();

		virtual void initialize();

		virtual void activate();
		virtual void deactivate();

		virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

	protected Q_SLOTS:
		void polygonArrayToEdges(jsk_recognition_msgs::PolygonArray polygonArray);
		void savePolygon(const std::string modelFilepath);

	private:
		ros::NodeHandle node_;
		ros::Publisher polygonPub_;
		geometry_msgs::PolygonStamped polygonMk_;
		jsk_recognition_msgs::PolygonArray polygonArray_;
		std::vector<Polygon> polygons_;

		bool newPolygon_; 
};


} // end namespace 

#endif 
