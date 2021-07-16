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

#include <iostream>
#include <fstream>

#include <boost/algorithm/string.hpp> 

#include <ros/ros.h>
#include <rviz/tool.h>
#include <ros/package.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <simple_laser_geometry/polygon.h>

namespace Ogre{
	class SceneNode;
	class Vector3;
}

namespace rviz{
	class VectorProperty;
	class VisualizationManager;
	class ViewportMouseEvent;
	class FloatProperty;
	class StringProperty;
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

	public Q_SLOTS:
		virtual void updateProperty();

	protected Q_SLOTS:
		std::vector<Polygon> polygonArrayToVector(jsk_recognition_msgs::PolygonArray polygonArray);
		void savePolygon(const std::string pathFilename);
		void showPolygonNames();

	private:
		ros::NodeHandle node_;
		ros::Publisher roisVizPub_, roisNamesVizPub_;
		jsk_recognition_msgs::PolygonArray polygonArray_;
		std::vector<Polygon> polygons_;
		bool newPolygon_;
		rviz::FloatProperty* inflationProperty_;
		rviz::StringProperty* roiNamesListProperty_;
		rviz::StringProperty* pathProperty_;
		float inflationRadius_;
		std::string pathFilename_;
		std::vector<std::string> roiNamesList_;
};


} // end namespace 

#endif 
