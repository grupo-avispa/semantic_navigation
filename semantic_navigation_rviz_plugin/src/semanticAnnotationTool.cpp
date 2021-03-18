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
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"

#include "semanticAnnotationTool.h"

namespace semantic_navigation_rviz_plugin{

/* Constructor */
semanticAnnotationTool::semanticAnnotationTool(){
	std::string filename = "roi_" + std::to_string(ros::Time::now().toSec()) + ".yaml";

	shortcut_key_ = 's';
	inflationProperty_ = new rviz::FloatProperty("Inflation radius", 0.5,"Inflation radius", getPropertyContainer(), SLOT(updateProperty()), this);
	roiNamesListProperty_ = new rviz::StringProperty("ROIs names", "", "List of ROIs names", getPropertyContainer(), SLOT(updateProperty()), this);
	pathProperty_ = new rviz::StringProperty("YAML config filename", QString::fromStdString(filename) , "Filename to save the ROIs.", getPropertyContainer(), SLOT(updateProperty()), this);
}

/* Destructor */
semanticAnnotationTool::~semanticAnnotationTool(){
}

/* Update properties */
void semanticAnnotationTool::updateProperty(){
	inflationRadius_ = inflationProperty_->getFloat();
	pathFilename_ = pathProperty_->getStdString();

	std::string roiList = roiNamesListProperty_->getStdString();
	if(roiList == "") roiNamesList_.clear();
	else boost::split(roiNamesList_, roiList, boost::is_any_of(","));
}

/* Initiate */
void semanticAnnotationTool::initialize(){
	roisVizPub_ = node_.advertise<jsk_recognition_msgs::PolygonArray>("/rois_viz", 1, true);
	roisNamesVizPub_ = node_.advertise<visualization_msgs::MarkerArray>("rois_names_viz", 1, true);
	newPolygon_ = true;
	updateProperty();
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
				polyStamp.header = polygonArray_.header;
				// Extract the last polygon
				if(!newPolygon_){
					polyStamp.polygon = polygonArray_.polygons.back().polygon;
					polygonArray_.polygons.pop_back();
				}
				// Capture the point from the map
				geometry_msgs::Point32 point;
				point.x = intersection.x;
				point.y = intersection.y;
				point.z = 0.0;
				// Create the new polygon 
				polyStamp.polygon.points.push_back(point);
				polygonArray_.polygons.push_back(polyStamp);
				// Publish it
				roisVizPub_.publish(polygonArray_);
				// Convert to vector of polygons
				polygons_ = polygonArrayToVector(polygonArray_);
				// Show names
				showPolygonNames();
				newPolygon_ = false;
			}

			if(event.rightUp()) newPolygon_ = true;

			if(event.middleUp()){
				newPolygon_ = true;
				// Save and clear the polygons
				savePolygon(pathFilename_);
				polygonArray_.polygons.clear();
				polygons_.clear();
				roisVizPub_.publish(polygonArray_);
				showPolygonNames();
			}
		}
	}catch(int a){
		ROS_ERROR("Error Occured!!");
		return 0;
	}

	return 1;
}

/* Convert a jsk_recognition_msgs::PolygonArray to a vector of Polygon with edges */
std::vector<Polygon> semanticAnnotationTool::polygonArrayToVector(jsk_recognition_msgs::PolygonArray polygonArray){
	std::vector<Polygon> polyVector;
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

		// Add name to the rois
		if(i < roiNamesList_.size()){
			area.setName(roiNamesList_[i]);
		}else{
			area.setName("roi_" + std::to_string(i));
		}

		polyVector.push_back(area);
	}
	return polyVector;
}

/* Save polygon into a file */
void semanticAnnotationTool::savePolygon(const std::string pathFilename){
	std::string filePath = ros::package::getPath("semantic_goals_generator") + "/params/" + pathFilename;
	std::ofstream polygonFile(filePath, std::ofstream::app);

	polygonFile << "inflation_radius: " << inflationRadius_ << std::endl;
	polygonFile << "rois:" << std::endl;
	for(Polygon poly: polygons_){
		polygonFile << "  - {name: '" << poly.getName() <<"', edges: [";
		std::vector<Edge> edges = poly.getEdges();
		for(int e = 0; e < edges.size() - 1; e++){
			Edge edge = edges[e];
			polygonFile << "[["<< edge.a.x << ", " << edge.a.y << "], [" << edge.b.x << ", " << edge.b.y << "]], " << std::endl;
			polygonFile << "                              ";
		}
		int edgesSize = edges.size();
		polygonFile << "[["<< edges[edgesSize-1].a.x << ", " << edges[edgesSize-1].a.y << "], [" << edges[edgesSize-1].b.x << ", " << edges[edgesSize-1].b.y << "]]]}" << std::endl;
	}

	polygonFile<<"\n";
	polygonFile.close();
}

/* Show polygon names */
void semanticAnnotationTool::showPolygonNames(){
	visualization_msgs::MarkerArray namesArray;
	for(int p = 0; p < polygons_.size(); p++){
		// Create label
		visualization_msgs::Marker labelMk;
		labelMk.header.frame_id = "map";
		labelMk.header.stamp = ros::Time::now();
		labelMk.ns = "labelroi";
		labelMk.id = p;
		labelMk.text = polygons_[p].getName();
		labelMk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		labelMk.action = visualization_msgs::Marker::ADD;
		labelMk.pose.position.x = polygons_[p].centroid().x;
		labelMk.pose.position.y = polygons_[p].centroid().y;
		labelMk.pose.position.z = 0.05;
		labelMk.pose.orientation.x = 0.0;
		labelMk.pose.orientation.y = 0.0;
		labelMk.pose.orientation.z = 0.0;
		labelMk.pose.orientation.w = 1.0;
		labelMk.scale.z = 0.5;
		labelMk.color.r = 1.0;
		labelMk.color.g = 1.0;
		labelMk.color.b = 1.0;
		labelMk.color.a = 1.0f;
		namesArray.markers.push_back(labelMk);
	}

	if(polygons_.empty()){
		visualization_msgs::Marker labelMk;
		labelMk.header.frame_id = "map";
		labelMk.action = visualization_msgs::Marker::DELETEALL;
		namesArray.markers.push_back(labelMk);
	}
	roisNamesVizPub_.publish(namesArray);
}

} // end namespace 

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(semantic_navigation_rviz_plugin::semanticAnnotationTool, rviz::Tool)

