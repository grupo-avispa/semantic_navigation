/*
 * SEMANTIC GOALS GENERATOR ROS NODE
 *
 * Author: Alberto José Tudela Roldán
 * Copyright (c) 2020, Universidad de Málaga
 * All rights reserved.
 *
 */

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Universidad de Málaga nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <map>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include "semantic_goals_generator/semantic_goals_generator.h"

/* Initialize the subscribers and publishers */
SemanticGoalsGenerator::SemanticGoalsGenerator(ros::NodeHandle& node, ros::NodeHandle& node_private) : node_(node), nodePrivate_(node_private){
	paramsSrv_ = nodePrivate_.advertiseService("params", &SemanticGoalsGenerator::updateParams, this);

	initialize();

	navGoalsPub_ = node_.advertise<geometry_msgs::PoseArray>("semantic_goals", 1);
	roiPub_ = node_.advertise<geometry_msgs::PolygonStamped>("roi_visualization", 1);

	navsGenSrv_ = nodePrivate_.advertiseService("/semantic_goals", &SemanticGoalsGenerator::SemanticGoalsService, this);
}

/* Delete all parameteres */
SemanticGoalsGenerator::~SemanticGoalsGenerator() {
	nodePrivate_.deleteParam("map_frame");
	nodePrivate_.deleteParam("is_costmap");
	nodePrivate_.deleteParam("inflation_radius");
}

/* Update parameters of the node */
bool SemanticGoalsGenerator::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	nodePrivate_.param<std::string>("map_frame", mapFrame_, "/map");
	nodePrivate_.param<bool>("is_costmap", isCostmap_, false);
	nodePrivate_.param<float>("inflation_radius", inflationRadius_, 0.5);

	roiVector_ = getROIParams();

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
	std::vector<std::string> roomNames;

	if(nodePrivate_.hasParam("rois/room_names")){
		nodePrivate_.getParam("rois/room_names", roomNames);

		for(int roi = 0; roi < roomNames.size(); roi++){
			if(nodePrivate_.hasParam("rois/" + roomNames[roi])){
				nodePrivate_.getParam("rois/" + roomNames[roi], xmlRoiList);

				Polygon poly;
				poly.name = roomNames[roi];

				for(int p = 0; p < xmlRoiList.size()-1; p++){
					Point a(static_cast<double>(xmlRoiList[p][0]), static_cast<double>(xmlRoiList[p][1]));
					Point b(static_cast<double>(xmlRoiList[p+1][0]), static_cast<double>(xmlRoiList[p+1][1]));
					Edge edge; edge.a = a; edge.b = b;
					poly.edges.push_back(edge);
				}
				// Add the last edge
				Point a(static_cast<double>(xmlRoiList[xmlRoiList.size()-1][0]), static_cast<double>(xmlRoiList[xmlRoiList.size()-1][1]));
				Point b(static_cast<double>(xmlRoiList[0][0]), static_cast<double>(xmlRoiList[0][1]));
				Edge edge; edge.a = a; edge.b = b;
				poly.edges.push_back(edge);
				rois.push_back(poly);
			}else{
				ROS_ERROR("Room 'rois/%s' not defined", roomNames[roi].c_str());
			}
		}
	}else{
		ROS_ERROR("Param 'rois/room_names' not exist");
	}
	
	ROS_INFO("[SemanticGoalsGenerator]: ROIs readed");
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
		ROS_INFO("No ROI specified, full map is used.");
	}else{
		// If the ROI is outside the map, adjust to map boundaries
		// Determine bounding box of ROI
		// We have to change point a on edge and b from the prior edge
		bBoxMinX_ = std::numeric_limits<int>::infinity();
		bBoxMaxX_ = -std::numeric_limits<int>::infinity();
		bBoxMinY_ = std::numeric_limits<int>::infinity();
		bBoxMaxY_ = -std::numeric_limits<int>::infinity();

		for(int e = 0; e < roi_.size(); e++){
			Point p = roi_.edges[e].a;

			if(p.x < mapMinX_) p.x = mapMinX_;
			if(p.x > mapMaxX_) p.x = mapMaxX_;

			if(p.x < bBoxMinX_) bBoxMinX_ = p.x;
			if(p.x > bBoxMaxX_) bBoxMaxX_ = p.x;

			if(p.y < mapMinY_) p.y = mapMinY_;
			if(p.y > mapMaxY_) p.y = mapMaxY_;

			if(p.y < bBoxMinY_) bBoxMinY_ = p.y;
			if(p.y > bBoxMaxY_) bBoxMaxY_ = p.y;
		}

		// Calculate bounding box for cell array
		cellMinX_ = int((bBoxMinX_ - origin_.position.x) / resolution_);
		cellMaxX_ = int((bBoxMaxX_ - origin_.position.x) / resolution_);
		cellMinY_ = int((bBoxMinY_ - origin_.position.y) / resolution_);
		cellMaxY_ = int((bBoxMaxY_ - origin_.position.y) / resolution_);

		ROS_INFO("[SemanticGoalsGenerator]: ROI bounding box (meters): (%f,%f) (%f,%f)", bBoxMinX_, bBoxMinY_, bBoxMaxX_, bBoxMaxY_);
		ROS_INFO("[SemanticGoalsGenerator]: ROI bounding box (cells): (%i,%i) (%i,%i)", cellMinX_, cellMinY_, cellMaxX_, cellMaxY_);
	}
}

/* Service for sending random goals based on labeled rois */
bool SemanticGoalsGenerator::SemanticGoalsService(semantic_goals_generator::SemanticGoals::Request& req, semantic_goals_generator::SemanticGoals::Response& res){
	ROS_INFO("[SemanticGoalsGenerator]: Incoming service request: %i, %s", req.n, req.roi_name.c_str());

	// Get arguments
	int n = req.n;
	for(int r = 0; r < roiVector_.size(); r++){
		if(roiVector_[r].name == req.roi_name){
			roi_ = roiVector_[r];
			break;
		}else if (req.roi_name.empty()){
			roi_.clear();
		}
	}

	try{
		nav_msgs::OccupancyGrid::ConstPtr msgMap;
		msgMap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(mapFrame_, ros::Duration(10));
		mapCallback(msgMap);
	}catch(...){
		ROS_FATAL("[SemanticGoalsGenerator]: Failed to get %s", mapFrame_.c_str());
		return false;
	}

	// Process bounding box
	processBoundingBox();

	// Generate response
	res.goals.header.frame_id = mapFrame_;

	// Generate random goal pose
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 eng(rd()); // seed the generator
	std::uniform_int_distribution<> distX(cellMinX_, cellMaxX_); // define the range
	std::uniform_int_distribution<> distY(cellMinY_, cellMaxY_); // define the range
	std::uniform_real_distribution<double> distPI(0.0, 2 * M_PI);

	int upperBound = n  * ( 2 +  inflationRadius_ / 0.01);
	int count = 0;
	//while( (res.goals.poses.size() < n) && (count < upperBound) ){
	while( res.goals.poses.size() < n){
		count += 1;
		int cellX = distX(eng);
		int cellY = distY(eng);     

		geometry_msgs::Pose pose;
		pose.position.x = cellX * resolution_ + origin_.position.x;
		pose.position.y = cellY * resolution_ + origin_.position.y;

		// If the point lies within ROI and is not in collision
		if(inROI(pose.position.x, pose.position.y) && !inCollision(cellX, cellY)){
			double yaw = distPI(eng);            
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), pose.orientation);

			ROS_INFO("[SemanticGoalsGenerator]: Pose (x: %f, y: %f, z: %f)", pose.position.x, pose.position.y, yaw);

			res.goals.poses.push_back(pose);
		}
	}

	navGoalsPub_.publish(res.goals);
	publishPolygonRoi();

	return true;
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
	Point p(x,y);
	return roi_.contains(p);
}

/* Check if a point is in collision */
bool SemanticGoalsGenerator::inCollision(int x, int y){
	int xMin, xMax, yMin, yMax;

	if(isCostmap_){
		if(cell(x, y) !=0 ) return true;
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

/* Publish the polygon roi */
void SemanticGoalsGenerator::publishPolygonRoi(){
	geometry_msgs::PolygonStamped polygonMk;
	polygonMk.header.frame_id = mapFrame_;
	polygonMk.header.stamp = ros::Time::now();

	for(int e = 0; e < roi_.size(); e++){
		Point p = roi_.edges[e].a;    
		geometry_msgs::Point32 pg; pg.x = p.x; pg.y = p.y; pg.z = 0.0;
		polygonMk.polygon.points.push_back(pg);
	}

	roiPub_.publish(polygonMk);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "SemanticGoalsGenerator");
	ros::NodeHandle node("");
	ros::NodeHandle node_private("~");

	try{
		ROS_INFO("[SemanticGoalsGenerator]: Initializing node");
		SemanticGoalsGenerator detector(node, node_private);
		ros::spin();
	}catch(const char* s){
		ROS_FATAL_STREAM("[NavGoals Generatorr]: " << s);
	}catch(...){
		ROS_FATAL_STREAM("[SemanticGoalsGenerator]: Unexpected error");
	}

	return 0;
}
