/*
 * SEMANTIC GOALS GENERATOR ROS NODE
 *
 * Copyright (c) 2020-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of semantic_navigation.
 * 
 * All rights reserved.
 *
 */

// C++
#include <limits>

// ROS
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <simple_laser_geometry/point2D.h>

// Semantic Goals
#include "semantic_goals_generator/semantic_goals_generator.h"

/* Initialize the subscribers and publishers */
SemanticGoalsGenerator::SemanticGoalsGenerator(ros::NodeHandle& node, ros::NodeHandle& node_private) : node_(node), nodePrivate_(node_private){
	// Initialize ROS parameters
	getParams();

	navGoalsPub_ = nodePrivate_.advertise<geometry_msgs::PoseArray>("semantic_goals", 1);
	roisVizPub_ = nodePrivate_.advertise<jsk_recognition_msgs::PolygonArray>("rois_viz", 1, true);
	roisNamesVizPub_ = nodePrivate_.advertise<visualization_msgs::MarkerArray>("rois_names_viz", 1, true);

	navGenSrv_ = nodePrivate_.advertiseService("/semantic_goals", &SemanticGoalsGenerator::SemanticGoalsService, this);
	semanticPosSrv_ = nodePrivate_.advertiseService("/semantic_position", &SemanticGoalsGenerator::SemanticPositionService, this);
	showVisualization();

	border_ = 0.0;
	direction_ = semantic_goals_generator::SemanticGoalsRequest::RANDOM;
}

/* Delete all parameteres */
SemanticGoalsGenerator::~SemanticGoalsGenerator(){
	nodePrivate_.deleteParam("map_topic");
	nodePrivate_.deleteParam("is_costmap");
	nodePrivate_.deleteParam("full_map");
}

/* Update parameters of the node */
void SemanticGoalsGenerator::getParams(){
	ROS_INFO("[Semantic goals generator]: Reading ROS parameters");

	nodePrivate_.param<std::string>("map_topic", mapTopic_, "map");
	nodePrivate_.param<bool>("is_costmap", isCostmap_, false);
	nodePrivate_.param<bool>("full_map", fullMap_, false);
	roiList_ = getROIParams();
	if (roiList_.empty()){
		ROS_ERROR("[Semantic goals generator]: The list of ROIs could not be found");
		exit(1);
	}
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
std::vector<ROI> SemanticGoalsGenerator::getROIParams(){
	XmlRpc::XmlRpcValue xmlRoiList;
	std::vector<ROI> rois;

	if(nodePrivate_.hasParam("inflation_radius")){
		nodePrivate_.getParam("inflation_radius", inflationRadius_);
	}else{
		inflationRadius_ = 0.5;
	}

	if(nodePrivate_.hasParam("rois")){
		nodePrivate_.getParam("rois", xmlRoiList);
		for(int32_t i = 0; i < xmlRoiList.size(); ++i){
			// Get yaw and name
			ROI roi;
			roi.yaw = static_cast<double>(xmlRoiList[i]["yaw"]);
			roi.polygon.setName(static_cast<std::string>(xmlRoiList[i]["name"]));

			// Extract edges
			int edgesSize = xmlRoiList[i]["edges"].size();
			for(int e = 0; e < edgesSize; e++){
				slg::Point2D a(static_cast<double>(xmlRoiList[i]["edges"][e][0][0]), static_cast<double>(xmlRoiList[i]["edges"][e][0][1]));
				slg::Point2D b(static_cast<double>(xmlRoiList[i]["edges"][e][1][0]), static_cast<double>(xmlRoiList[i]["edges"][e][1][1]));
				roi.polygon.addEdge({a,b});
			}
			rois.push_back(roi);
		}
	}else{
		ROS_ERROR("[Semantic goals generator]: Param 'rois' not exists");
		return std::vector<ROI>();
	}

	ROS_INFO("[Semantic goals generator]: ROIs read");
	return rois;
}

/* Calculate bounding box for ROI */
void SemanticGoalsGenerator::processBoundingBox(ROI roi){
	// Region of interest (ROI) must lie inside the map boundaries
	// If ROI is empty, the whole map is treated as ROI by default
	if(roi.empty()){
		bBoxMinX_ = mapMinX_;
		bBoxMaxX_ = mapMaxX_;
		bBoxMinY_ = mapMinY_;
		bBoxMaxY_ = mapMaxY_;
		ROS_INFO("[Semantic goals generator]: No ROI specified, full map is used");
	}else{
		// If the ROI is outside the map, adjust to map boundaries
		// Determine bounding box of ROI
		bBoxMinX_ = std::numeric_limits<double>::infinity();
		bBoxMaxX_ = -std::numeric_limits<double>::infinity();
		bBoxMinY_ = std::numeric_limits<double>::infinity();
		bBoxMaxY_ = -std::numeric_limits<double>::infinity();

		slg::Polygon polygonRoi = roi.polygon;
		for(int e = 0; e < polygonRoi.size(); e++){
			slg::Point2D p = polygonRoi.getEdge(e).a;

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

	ROI currentRoi;

	// Get arguments
	int n = req.n;
	for(ROI roi: roiList_){
		if(roi.polygon.getName() == req.roi_name){
			currentRoi = roi;
			break;
		}
	}
	direction_ = req.direction;
	border_ = req.border;

	// If the requested ROI is empty and we don't want to use the full map
	if (currentRoi.empty() && !fullMap_){
		ROS_FATAL("[Semantic goals generator]: The requested ROI: %s, could not be found in the list", req.roi_name.c_str());
		return false;
	}

	// Wait for map
	nav_msgs::OccupancyGrid::ConstPtr msgMap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(mapTopic_, node_, ros::Duration(10));
	if(msgMap){
		mapCallback(msgMap);
	}else{
		ROS_FATAL("[Semantic goals generator]: Failed to get %s", mapTopic_.c_str());
		return false;
	}

	// Inflation radius must be positive
	if(inflationRadius_ < 0) inflationRadius_ = 0.5;
	inflatedFootprintSize_ = int(inflationRadius_ / resolution_) + 1;

	// Process bounding box
	processBoundingBox(currentRoi);

	// Generate response
	res.goals.header.frame_id = mapTopic_;

	// Generate random goal pose
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 gen(rd()); // seed the generator
	std::uniform_int_distribution<int> distX(cellMinX_, cellMaxX_); // define the range
	std::uniform_int_distribution<int> distY(cellMinY_, cellMaxY_); // define the range
	std::uniform_real_distribution<double> distPI(0.0, 2 * M_PI);

	int count = 0;
	while( (res.goals.poses.size() < n)){
		count += 1;
		int cellX = distX(gen);
		int cellY = distY(gen);

		geometry_msgs::Pose pose;
		pose.position.x = cellX * resolution_ + origin_.position.x;
		pose.position.y = cellY * resolution_ + origin_.position.y;

		// If the point lies within ROI and is not in collision
		if(currentRoi.inROI(pose.position.x, pose.position.y) && !inCollision(cellX, cellY) && currentRoi.disFromBorders(pose.position.x, pose.position.y, border_)){
			// Generate orientation
			double yaw;
			if (direction_ == semantic_goals_generator::SemanticGoalsRequest::OUTSIDE){
				yaw = atan2((pose.position.y - currentRoi.polygon.centroid().y), (pose.position.x - currentRoi.polygon.centroid().x));
			}else if (direction_ == semantic_goals_generator::SemanticGoalsRequest::INSIDE){
				yaw = atan2((pose.position.y - currentRoi.polygon.centroid().y), (pose.position.x - currentRoi.polygon.centroid().x)) + M_PI;
			}else if (direction_ == semantic_goals_generator::SemanticGoalsRequest::STORED){
				if (currentRoi.yaw > -M_PI && currentRoi.yaw < M_PI){
					yaw = currentRoi.yaw;
				}else{
					yaw = distPI(gen);
				}
			}else if (direction_ == semantic_goals_generator::SemanticGoalsRequest::REQUESTED){
				if (req.yaw > -M_PI && req.yaw < M_PI){
					yaw = req.yaw;
				}else{
					yaw = distPI(gen);
				}
			}else{
				yaw = distPI(gen);
			}
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), pose.orientation);
			ROS_INFO("[Semantic goals generator]: Pose %i (x: %f, y: %f, yaw: %f)", res.goals.poses.size()+1, pose.position.x, pose.position.y, yaw);

			res.goals.poses.push_back(pose);
		}
	}

	navGoalsPub_.publish(res.goals);

	return true;
}

/* Service for request the semantic pose  */
bool SemanticGoalsGenerator::SemanticPositionService(semantic_goals_generator::SemanticPosition::Request& req, semantic_goals_generator::SemanticPosition::Response& res){
	ROS_INFO("[Semantic goals generator]: Incoming service request: %f, %f", req.position.x, req.position.y);

	// Get arguments and check if the point lies within ROI
	for(ROI roi: roiList_){
		if(roi.inROI(req.position.x, req.position.y)){
			res.roi_name = roi.getName();
			return true;
		}
	}

	res.roi_name = semantic_goals_generator::SemanticPosition::Response::UNKNOWN;
	ROS_FATAL("[Semantic goals generator]: Failed to get semantic position");
	return false;
}

/* Return the cell of the costmap */
int SemanticGoalsGenerator::cell(int x, int y){
	// Return 'unknown' if out of bounds
	if(x < 0 || y < 0 || x >= width_  || y >= height_) return -1;

	return mapData_[x +  width_ * y];
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

/* Show the rois in rviz */
void SemanticGoalsGenerator::showVisualization(){
	jsk_recognition_msgs::PolygonArray polygonArray;
	polygonArray.header.frame_id = mapTopic_;
	polygonArray.header.stamp = ros::Time::now();

	visualization_msgs::MarkerArray namesArray;

	for(int r = 0; r < roiList_.size(); r++){
		// Create polygon marker
		geometry_msgs::PolygonStamped polygonMk;
		polygonMk.header.frame_id = mapTopic_;
		polygonMk.header.stamp = ros::Time::now();

		slg::Polygon polygon = roiList_[r].polygon;
		for(int e = 0; e < polygon.size(); e++){
			slg::Point2D p = polygon.getEdge(e).a;
			geometry_msgs::Point32 pg; 
			pg.x = p.x; pg.y = p.y; pg.z = 0.0;
			polygonMk.polygon.points.push_back(pg);
		}
		polygonArray.polygons.push_back(polygonMk);
		polygonArray.labels.push_back(r);

		// Create label
		visualization_msgs::Marker labelMk;
		labelMk.header.frame_id = mapTopic_;
		labelMk.header.stamp = ros::Time::now();
		labelMk.ns = "labelroi";
		labelMk.id = r;
		labelMk.text = roiList_[r].getName();
		labelMk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		labelMk.action = visualization_msgs::Marker::ADD;
		labelMk.pose.position.x = polygon.centroid().x;
		labelMk.pose.position.y = polygon.centroid().y;
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
