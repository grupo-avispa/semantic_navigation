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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>
#include <geometry_msgs/Point.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "semanticAnnotationTool.h"

namespace semantic_navigation_rviz_plugin{

/* Constructor */
semanticAnnotationTool::semanticAnnotationTool(){
	shortcut_key_ = 's';
}

/* Destructor */
semanticAnnotationTool::~semanticAnnotationTool(){
}

void semanticAnnotationTool::initialize(){
	polygonPub_ = node_.advertise<jsk_recognition_msgs::PolygonArray>("/rois_viz", 1, true);
	newPolygon_ = true;
}

/* Activation  */
void semanticAnnotationTool::activate(){
	initialize();
	ROS_INFO("Semantic annotation tool started!");
}

/* Deactivate */
void semanticAnnotationTool::deactivate(){
}

/* Handling mouse events */
int semanticAnnotationTool::processMouseEvent(rviz::ViewportMouseEvent& event){
	Ogre::Vector3 intersection;
	Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
	geometry_msgs::PolygonStamped polyStamp;
	polygonArray_.header.frame_id = "map";
	polygonArray_.header.stamp = ros::Time::now();

	try{
		if( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection )){
			if(event.leftDown()){
				// Extract the last polygon
				polyStamp.header = polygonArray_.header;
				//if( (!newPolygon) && (!polygonArray_.polygons.empty()) ){
				if(!newPolygon_){
					polyStamp.polygon = polygonArray_.polygons.back().polygon;
					polygonArray_.polygons.pop_back();
				}
				// Capture the point from the map
				geometry_msgs::Point32 point;
				point.x = intersection.x;
				point.y = intersection.y;
				point.z = 0.0;
				// Create the new polygon and publish it
				polyStamp.polygon.points.push_back(point);
				polygonArray_.polygons.push_back(polyStamp);
				polygonPub_.publish(polygonArray_);
				newPolygon_ = false;
			}

			if(event.rightUp()) newPolygon_ = true;

			if(event.middleUp()){
				// Clear the polygons
				polygonArray_.polygons.clear();
				polygonPub_.publish(polygonArray_);
			}
		}
	}catch(int a){
		ROS_ERROR("Error Occured!!");
		return 0;
	}

	return 1;
}

/* Convert a jsk_recognition_msgs::PolygonArray to a vector of Polygon with edges */
void semanticAnnotationTool::polygonArrayToEdges(jsk_recognition_msgs::PolygonArray polygonArray){
	for(int i = 0; i < polygonArray.polygons.size(); i++){
		geometry_msgs::Polygon poly = polygonArray.polygons[i].polygon;
		Polygon area;
		// Read n-1 points
		for(int p = 0; p < poly.points.size() - 1; p++){
			geometry_msgs::Point32 currPoint = poly.points[p];
			geometry_msgs::Point32 nextPoint = poly.points[p+1];

			Point a(currPoint.x, currPoint.y);
			Point b(nextPoint.x, nextPoint.y);
			area.addEdge({a,b});
		}
		// Add the last edge
		geometry_msgs::Point32 lastPoint = poly.points[poly.points.size()-1];
		geometry_msgs::Point32 firstPoint = poly.points[0];

		Point a(lastPoint.x, lastPoint.y);
		Point b(firstPoint.x, firstPoint.y);
		area.addEdge({a,b});
		polygons_.push_back(area);
	}
}

/* Save polygon into a file */
void semanticAnnotationTool::savePolygon(const std::string modelFilepath){
	std::ofstream polygonFile(modelFilepath, std::ofstream::app);

	polygonFile << "Features: " << std::endl;

	polygonFile<<"\n";
	polygonFile.close();
}

} // end namespace 

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(semantic_navigation_rviz_plugin::semanticAnnotationTool, rviz::Tool)

