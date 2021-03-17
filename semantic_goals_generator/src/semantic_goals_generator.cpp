/*
 * SEMANTIC GOALS GENERATOR ROS NODE
 *
 * Copyright (c) 2020 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of semantic_navigation.
 * 
 * All rights reserved.
 *
 */

#include <map>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

#include "semantic_goals_generator/semantic_goals_generator.h"

/* Initialize the subscribers and publishers */
SemanticGoalsGenerator::SemanticGoalsGenerator(ros::NodeHandle& node, ros::NodeHandle& node_private) : node_(node), nodePrivate_(node_private){
	paramsSrv_ = nodePrivate_.advertiseService("params", &SemanticGoalsGenerator::updateParams, this);

	initialize();

	navGoalsPub_ = nodePrivate_.advertise<geometry_msgs::PoseArray>("semantic_goals", 1);
	roisVizPub_ = nodePrivate_.advertise<jsk_recognition_msgs::PolygonArray>("rois_viz", 1, true);
	roisNamesVizPub_ = nodePrivate_.advertise<visualization_msgs::MarkerArray>("rois_names_viz", 1, true);

	navsGenSrv_ = nodePrivate_.advertiseService("/semantic_goals", &SemanticGoalsGenerator::SemanticGoalsService, this);
	semanticPosSrv_ = nodePrivate_.advertiseService("/semantic_position", &SemanticGoalsGenerator::SemanticPositionService, this);
	showVisualization();

	border_ = 0.0;
	orientation_ = "any";
}

/* Delete all parameteres */
SemanticGoalsGenerator::~SemanticGoalsGenerator() {
	nodePrivate_.deleteParam("map_topic");
	nodePrivate_.deleteParam("is_costmap");
}

/* Update parameters of the node */
bool SemanticGoalsGenerator::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	nodePrivate_.param<std::string>("map_topic", mapTopic_, "map");
	nodePrivate_.param<bool>("is_costmap", isCostmap_, false);
	roisList_ = getROIParams();
	return true;
}

/* Map callback */
void SemanticGoalsGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgMap){
	resolution_ = msgMap->info.resolution;
	width_ = msgMap->info.width;
	height_ = msgMap->info.height;
	origin_ = msgMap->info.origin;
	mapData_ = msgMap->data;

	mapMinX_ = origin_.position.x;
	mapMaxX_ = origin_.position.x + width_ * resolution_;
	mapMinY_ = origin_.position.y;
	mapMaxY_ = origin_.position.y + height_ * resolution_;
}

/* Get rois from YAML file */
std::vector<Polygon> SemanticGoalsGenerator::getROIParams(){
	XmlRpc::XmlRpcValue xmlRoiList;
	std::vector<Polygon> rois;
	std::vector<std::string> roisNames;

	if(nodePrivate_.hasParam("inflation_radius")){
		nodePrivate_.getParam("inflation_radius", inflationRadius_);
	}else{
		inflationRadius_ = 0.5;
	}

	if(nodePrivate_.hasParam("rois")){
		nodePrivate_.getParam("rois", xmlRoiList);
		for(int32_t i = 0; i < xmlRoiList.size(); ++i){
			// Create a polygon
			Polygon poly;
			poly.setName(static_cast<std::string>(xmlRoiList[i]["name"]));

			// Extract points
			/*int pointsSize = xmlRoiList[i]["points"].size();
			for(int p = 0; p < pointsSize-1; p++){
				Point a(static_cast<double>(xmlRoiList[i]["points"][p][0]), static_cast<double>(xmlRoiList[i]["points"][p][1]));
				Point b(static_cast<double>(xmlRoiList[i]["points"][p+1][0]), static_cast<double>(xmlRoiList[i]["points"][p+1][1]));
				poly.addEdge({a,b});
			}
			// Add the last edge
			Point a(static_cast<double>(xmlRoiList[i]["points"][pointsSize-1][0]), static_cast<double>(xmlRoiList[i]["points"][pointsSize-1][1]));
			Point b(static_cast<double>(xmlRoiList[i]["points"][0][0]), static_cast<double>(xmlRoiList[i]["points"][0][1]));
			poly.addEdge({a,b});
			rois.push_back(poly);*/

			// Extract edges
			int edgesSize = xmlRoiList[i]["edges"].size();
			for(int e = 0; e < edgesSize; e++){
				Point a(static_cast<double>(xmlRoiList[i]["edges"][e][0][0]), static_cast<double>(xmlRoiList[i]["edges"][e][0][1]));
				Point b(static_cast<double>(xmlRoiList[i]["edges"][e][1][0]), static_cast<double>(xmlRoiList[i]["edges"][e][1][1]));
				poly.addEdge({a,b});
			}
			rois.push_back(poly);
		}
	}else{
		ROS_ERROR("[Semantic goals generator]: Param 'rois' not exist");
	}

	ROS_INFO("[Semantic goals generator]: ROIs read");
	return rois;
}

/* Calculate bounding box for ROI */
void SemanticGoalsGenerator::processBoundingBox(){
	// Inflation radius must be positive
	if(inflationRadius_ < 0) inflationRadius_ = 0.5;

	inflatedFootprintSize_ = int(inflationRadius_ / resolution_) + 1;

	// Region of interest (ROI) must lie inside the map boundaries
	// If ROI is empty, the whole map is treated as ROI by default
	if(roi_.empty()){
		bBoxMinX_ = mapMinX_;
		bBoxMaxX_ = mapMaxX_;
		bBoxMinY_ = mapMinY_;
		bBoxMaxY_ = mapMaxY_;
		ROS_INFO("[Semantic goals generator]: No ROI specified, full map is used.");
	}else{
		// If the ROI is outside the map, adjust to map boundaries
		// Determine bounding box of ROI
		bBoxMinX_ = std::numeric_limits<int>::infinity();
		bBoxMaxX_ = -std::numeric_limits<int>::infinity();
		bBoxMinY_ = std::numeric_limits<int>::infinity();
		bBoxMaxY_ = -std::numeric_limits<int>::infinity();

		for(int e = 0; e < roi_.size(); e++){
			Point p = roi_.getEdge(e).a;

			if(p.x < mapMinX_) p.x = mapMinX_;
			if(p.x > mapMaxX_) p.x = mapMaxX_;

			if(p.x < bBoxMinX_) bBoxMinX_ = p.x;
			if(p.x > bBoxMaxX_) bBoxMaxX_ = p.x;

			if(p.y < mapMinY_) p.y = mapMinY_;
			if(p.y > mapMaxY_) p.y = mapMaxY_;

			if(p.y < bBoxMinY_) bBoxMinY_ = p.y;
			if(p.y > bBoxMaxY_) bBoxMaxY_ = p.y;
		}
	}

	// Calculate bounding box for cell array
	cellMinX_ = int((bBoxMinX_ - origin_.position.x) / resolution_);
	cellMaxX_ = int((bBoxMaxX_ - origin_.position.x) / resolution_);
	cellMinY_ = int((bBoxMinY_ - origin_.position.y) / resolution_);
	cellMaxY_ = int((bBoxMaxY_ - origin_.position.y) / resolution_);

	ROS_INFO("[Semantic goals generator]: ROI bounding box (meters): (%f,%f) (%f,%f)", bBoxMinX_, bBoxMinY_, bBoxMaxX_, bBoxMaxY_);
	ROS_INFO("[Semantic goals generator]: ROI bounding box (cells): (%i,%i) (%i,%i)", cellMinX_, cellMinY_, cellMaxX_, cellMaxY_);
}

/* Service for sending random goals based on labeled rois */
bool SemanticGoalsGenerator::SemanticGoalsService(semantic_goals_generator::SemanticGoals::Request& req, semantic_goals_generator::SemanticGoals::Response& res){
	ROS_INFO("[Semantic goals generator]: Incoming service request: %i, %s", req.n, req.roi_name.c_str());

	// Clear previous ROI
	roi_.clear();

	// Get arguments
	int n = req.n;
	for(Polygon roi: roisList_){
		if(roi.getName() == req.roi_name){
			roi_ = roi;
			break;
		}
	}
	orientation_ = req.orientation;
	border_ = req.border;

	// Decrease the ROI
	decreaseROI();

	// Wait for map
	nav_msgs::OccupancyGrid::ConstPtr msgMap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(mapTopic_, node_, ros::Duration(10));
	if(msgMap){
		mapCallback(msgMap);
	}else{
		ROS_FATAL("[Semantic goals generator]: Failed to get %s", mapTopic_.c_str());
		return false;
	}

	// Process bounding box
	processBoundingBox();

	// Generate response
	res.goals.header.frame_id = "map";

	// Generate random goal pose
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 gen(rd()); // seed the generator
	std::uniform_int_distribution<int> distX(cellMinX_, cellMaxX_); // define the range
	std::uniform_int_distribution<int> distY(cellMinY_, cellMaxY_); // define the range
	std::uniform_real_distribution<double> distPI(0.0, 2 * M_PI);

	int count = 0;
	//int upperBound = n  * ( 2 +  inflationRadius_ / 0.01);
	//while( (res.goals.poses.size() < n) && (count < upperBound) ){
	while( (res.goals.poses.size() < n)){
		count += 1;
		int cellX = distX(gen);
		int cellY = distY(gen);

		geometry_msgs::Pose pose;
		pose.position.x = cellX * resolution_ + origin_.position.x;
		pose.position.y = cellY * resolution_ + origin_.position.y;

		// If the point lies within ROI and is not in collision
		if(inROI(pose.position.x, pose.position.y) && !inCollision(cellX, cellY)){
			// Generate orientation
			double yaw;
			if(orientation_ == "outside"){
				yaw = atan2((pose.position.y - roi_.centroid().y), (pose.position.x - roi_.centroid().x));
			}else if(orientation_ == "inside"){
				yaw = atan2((pose.position.y - roi_.centroid().y), (pose.position.x - roi_.centroid().x)) + M_PI;
			}else{
				yaw = distPI(gen);
			}
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), pose.orientation);
			ROS_INFO("[Semantic goals generator]: Pose (x: %f, y: %f, z: %f)", pose.position.x, pose.position.y, yaw);

			res.goals.poses.push_back(pose);
		}
	}

	navGoalsPub_.publish(res.goals);

	return true;
}

/* Service for request the semantic pose  */
bool SemanticGoalsGenerator::SemanticPositionService(semantic_goals_generator::SemanticPosition::Request& req, semantic_goals_generator::SemanticPosition::Response& res){
	ROS_INFO("[Semantic goals generator]: Incoming service request: %f, %f", req.position.x, req.position.y);

	// Get arguments
	for(int r = 0; r < roisList_.size(); r++){
		roi_ = roisList_[r];
		// If the point lies within ROI
		if(inROI(req.position.x, req.position.y)){
			res.roi_name = roi_.getName();
			return true;
		}
	}

	res.roi_name = "Region unknown";
	ROS_FATAL("[Semantic goals generator]: Failed to get semantic position");
	return false;
}

/* Return the cell of the costmap */
int SemanticGoalsGenerator::cell(int x, int y){
	// Return 'unknown' if out of bounds
	if(x < 0 || y < 0 || x >= width_  || y >= height_) return -1;

	return mapData_[x +  width_ * y];
}

/* Check if a point is inside the region of interest (ROI) */
bool SemanticGoalsGenerator::inROI(float x, float y){
	if(roi_.size() == 0) return true;
	return roi_.contains(Point(x,y));
}

/* Check if a point is in collision */
bool SemanticGoalsGenerator::inCollision(int x, int y){
	int xMin, xMax, yMin, yMax;

	if(isCostmap_){
		if(cell(x, y) != 0) return true;
		return false;
	}

	xMin = x - inflatedFootprintSize_;
	xMax = x + inflatedFootprintSize_;
	yMin = y - inflatedFootprintSize_;
	yMax = y + inflatedFootprintSize_;

	for(int i = xMin; i < xMax; i++){
		for(int j = yMin; j < yMax; j++){
			if(cell(i, j) != 0) return true;
		}
	}
	return false;
}

/* Decrease the size of the ROI by border distance */
void SemanticGoalsGenerator::decreaseROI(){
	Polygon newRoi;
	std::vector<Edge> edges = roi_.getEdges();

	for(Edge edge: edges){
		if(edge.a.x < roi_.centroid().x) edge.a.x += border_;
		if(edge.a.x > roi_.centroid().x) edge.a.x -= border_;
		if(edge.a.y < roi_.centroid().y) edge.a.y += border_;
		if(edge.a.y > roi_.centroid().y) edge.a.y -= border_;

		if(edge.b.x < roi_.centroid().x) edge.b.x += border_;
		if(edge.b.x > roi_.centroid().x) edge.b.x -= border_;
		if(edge.b.y < roi_.centroid().y) edge.b.y += border_;
		if(edge.b.y > roi_.centroid().y) edge.b.y -= border_;
		newRoi.addEdge(edge);
	}
	roi_ = newRoi;
}

/* Show the rois in rviz */
void SemanticGoalsGenerator::showVisualization(){
	jsk_recognition_msgs::PolygonArray polygonArray;
	polygonArray.header.frame_id = "map";
	polygonArray.header.stamp = ros::Time::now();

	visualization_msgs::MarkerArray namesArray;

	for(int r = 0; r < roisList_.size(); r++){
		// Create polygon marker
		geometry_msgs::PolygonStamped polygonMk;
		polygonMk.header.frame_id = "map";
		polygonMk.header.stamp = ros::Time::now();

		for(int e = 0; e < roisList_[r].size(); e++){
			Point p = roisList_[r].getEdge(e).a;
			geometry_msgs::Point32 pg; 
			pg.x = p.x; pg.y = p.y; pg.z = 0.0;
			polygonMk.polygon.points.push_back(pg);
		}
		polygonArray.polygons.push_back(polygonMk);
		polygonArray.labels.push_back(r);

		// Create label
		visualization_msgs::Marker labelMk;
		labelMk.header.frame_id = "map";
		labelMk.header.stamp = ros::Time::now();
		labelMk.ns = "labelroi";
		labelMk.id = r;
		labelMk.text = roisList_[r].getName();
		labelMk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		labelMk.action = visualization_msgs::Marker::ADD;
		labelMk.pose.position.x = roisList_[r].centroid().x;
		labelMk.pose.position.y = roisList_[r].centroid().y;
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

	roisVizPub_.publish(polygonArray);
	roisNamesVizPub_.publish(namesArray);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "semantic_goals_generator");
	ros::NodeHandle node("");
	ros::NodeHandle node_private("~");

	try{
		ROS_INFO("[Semantic goals generator]: Initializing node");
		SemanticGoalsGenerator detector(node, node_private);
		ros::spin();
	}catch(const char* s){
		ROS_FATAL_STREAM("[Semantic goals generator]: " << s);
	}catch(...){
		ROS_FATAL_STREAM("[Semantic goals generator]: Unexpected error");
	}

	return 0;
}
