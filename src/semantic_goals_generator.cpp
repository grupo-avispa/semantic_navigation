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

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include "semantic_goals_generator/semantic_goals_generator.h"


/* Initialize the subscribers and publishers */
SemanticGoalsGenerator::SemanticGoalsGenerator(ros::NodeHandle& node, ros::NodeHandle& node_private) : node_(node), nodePrivate_(node_private){
    paramsSrv_ = nodePrivate_.advertiseService("params", &SemanticGoalsGenerator::updateParams, this);

    initialize();

    navGoalsPub_ = node_.advertise<geometry_msgs::PoseArray>("semantic_goals", 1);
    visNavGoalsPub_ = node_.advertise<visualization_msgs::MarkerArray>("vis_semantic_goals", 1);

    navsGenSrv_ = nodePrivate_.advertiseService("/semantic_goals", &SemanticGoalsGenerator::SemanticGoalsService, this);

    markersLen_ = 0;
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
   // mapData_ = msgMap->data;

    mapMinX_ = origin_.position.x;
    mapMaxX_ = origin_.position.x + width_ * resolution_;
    mapMinY_ = origin_.position.y;
    mapMaxY_ = origin_.position.y + height_ * resolution_;
}

/* Get rois from YAML file */
std::vector<Polygon> SemanticGoalsGenerator::getROIParams(){
    XmlRpc::XmlRpcValue xmlRoiList;
    std::vector<Polygon> rois;

    nodePrivate_.getParam("rois", xmlRoiList);
    if(xmlRoiList.getType() != XmlRpc::XmlRpcValue::TypeArray ){
        ROS_ERROR("Param '[%s]' not a list", xmlRoiList);
    }else{
        for(int roi = 0; roi < xmlRoiList.size(); ++roi){
            if(xmlRoiList[roi].getType() != XmlRpc::XmlRpcValue::TypeArray){
                ROS_ERROR("Param [%s] is not a list", roi);
            }else{
                if(xmlRoiList[roi][0].getType() != XmlRpc::XmlRpcValue::TypeString){
                    ROS_ERROR("[%s] is not a label", xmlRoiList[roi][0]);
                }else{
                    Polygon poly;
                    poly.name = static_cast<std::string>(xmlRoiList[roi][0]);
                    for(int point = 0; point < xmlRoiList[roi].size()-1; ++point){
                        if(xmlRoiList[roi][point].size() != 2){
                            ROS_ERROR("[%d] is not a pair", point);
                        }else if(xmlRoiList[roi][point][0].getType() != XmlRpc::XmlRpcValue::TypeDouble || xmlRoiList[roi][point][1].getType() != XmlRpc::XmlRpcValue::TypeDouble){
                            ROS_ERROR("[%d] is not a pair of doubles", point);
                        }else{
                            Edge edge;
                            edge.a = (static_cast<double>(xmlRoiList[roi][point][0]), static_cast<double>(xmlRoiList[roi][point][1]));
                            edge.b = (static_cast<double>(xmlRoiList[roi][point+1][0]), static_cast<double>(xmlRoiList[roi][point+1][1]));
                            poly.edges.push_back(edge);
                        }
                    }
                    // Add the last edge
                    Edge edge;
                    edge.a = (static_cast<double>(xmlRoiList[roi][xmlRoiList[roi].size()-1][0]), static_cast<double>(xmlRoiList[roi][xmlRoiList[roi].size()-1][1]));
                    edge.b = (static_cast<double>(xmlRoiList[roi][0][0]), static_cast<double>(xmlRoiList[roi][0][1]));
                    poly.edges.push_back(edge);
                    rois.push_back(poly);
                }
            }
        }
    }
    return rois;
}

/* Calculate bounding box for ROI */
void SemanticGoalsGenerator::processBoundingBox(){
    // Inflation radius must be positive
    if(inflationRadius_ < 0) inflationRadius_ = 0.5;

    inflatedFootprintSize_ = int(inflationRadius_ / resolution_) + 1;

    // Region of interest (ROI) must lie inside the map boundaries
    // If ROI is empty, the whole map is treated as ROI by default
    if(roi_.edges.size() == 0){
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
        for(int e = 0; e < roi_.edges.size(); e++){
            if(roi_.edges[e].a.x < mapMinX_){
                roi_.edges[e].a.x = mapMinX_;
                if(e == 0) roi_.edges[roi_.edges.size()-1].b.x = mapMinX_;
                else roi_.edges[e-1].b.x = mapMinX_;
            }
            if(roi_.edges[e].a.x > mapMaxX_){
                roi_.edges[e].a.x = mapMaxX_;
                if(e == 0) roi_.edges[roi_.edges.size()-1].b.x = mapMaxX_;
                else roi_.edges[e-1].b.x = mapMaxX_;

            }
            if(roi_.edges[e].a.x < bBoxMinX_) bBoxMinX_ = roi_.edges[e].a.x;
            if(roi_.edges[e].a.x > bBoxMaxX_) bBoxMaxX_ = roi_.edges[e].a.x;

            if(roi_.edges[e].a.y < mapMinY_){
                roi_.edges[e].a.y = mapMinY_;
                if(e == 0) roi_.edges[roi_.edges.size()-1].b.y = mapMinY_;
                else roi_.edges[e-1].b.y = mapMinY_;
            }
            if(roi_.edges[e].a.y > mapMaxY_){
                roi_.edges[e].a.y = mapMaxY_;
                if(e == 0) roi_.edges[roi_.edges.size()-1].b.y = mapMaxY_;
                else roi_.edges[e-1].b.y = mapMaxY_;
            }
            if(roi_.edges[e].a.y < bBoxMinY_) bBoxMinY_ = roi_.edges[e].a.y;
            if(roi_.edges[e].a.y > bBoxMaxY_) bBoxMaxY_ = roi_.edges[e].a.y;
        }

        // Calculate bounding box for cell array
        cellMinX_ = int((bBoxMinX_ - origin_.position.x) / resolution_);
        cellMaxX_ = int((bBoxMaxX_ - origin_.position.x) / resolution_);
        cellMinY_ = int((bBoxMinY_ - origin_.position.y) / resolution_);
        cellMaxY_ = int((bBoxMaxY_ - origin_.position.y) / resolution_);

        ROS_INFO("ROI bounding box (meters): (%s,%s) (%s,%s)", bBoxMinX_, bBoxMinY_, bBoxMaxX_, bBoxMaxY_);
        ROS_INFO("ROI bounding box (cells): (%s,%s) (%s,%s)", cellMinX_, cellMinY_, cellMaxX_, cellMaxY_);
    }
}

/* Service for sending random goals based on labeled rois */
bool SemanticGoalsGenerator::SemanticGoalsService(semantic_goals_generator::SemanticGoals::Request& req, semantic_goals_generator::SemanticGoals::Response& res){
    visualization_msgs::MarkerArray markerArray;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distributionPI(0.0, 2 * M_PI);
	std::uniform_int_distribution<int> distributionX(cellMinX_, cellMaxX_);  
	std::uniform_int_distribution<int> distributionY(cellMinY_, cellMaxY_);
	
    ROS_INFO("[SemanticGoalsGenerator]: Incoming service request: %s", req);

    // Get arguments
    int n = req.n;
    for(int r = 0; r < roiVector_.size(); r++){
        if(roiVector_[r].name == req.roi_name){
            roi_ = roiVector_[r];
            break;
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

	deleteMarkers();

	// Generate random goal pose
    int upperBound = n * (2 + inflationRadius_ / 0.01);
    int goalCount = 0;
    while(res.goals.poses.size() < n  && goalCount < upperBound){
        goalCount += 1;
        int cellX = distributionX(generator);
        int cellY = distributionY(generator);

        geometry_msgs::Pose pose;
        pose.position.x = cellX * resolution_ + origin_.position.x;
        pose.position.y = cellY * resolution_ + origin_.position.y;

        // If the point lies within ROI and is not in collision
        if(inROI(pose.position.x, pose.position.y) && !inCollision(cellX, cellY)){
            double yaw = distributionPI(generator);            
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), pose.orientation);         

            ROS_INFO("[SemanticGoalsGenerator]: Pose (x: %f, y: %f)", pose.position.x, pose.position.y);

            res.goals.poses.push_back(pose);
            //createMarker(markerArray, res.goals.poses.size()-1, pose);
        }
    }

    navGoalsPub_.publish(res.goals);
    markersLen_ = markerArray.markers.size();

    return true;
}

/* Return the cell of the costmap */
int SemanticGoalsGenerator::cell(int x, int y){
    // Return 'unknown' if out of bounds
    if(x < 0 || y < 0 || x >= width_  || y >= height_) return -1;

    //return mapData_[x +  width_ * y];
}

/* Check if a point is inside the region of interest (ROI) */
bool SemanticGoalsGenerator::inROI(int x, int y){
    if(roi_.edges.size() == 0) return true;
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

/* Create navigation goals visualization markers */
void SemanticGoalsGenerator::createMarker(visualization_msgs::MarkerArray& markerArray, int markerId, geometry_msgs::Pose pose){
    /* Create a marker triangle */
	visualization_msgs::Marker vizMarker;
	vizMarker.header.frame_id = mapFrame_;
	vizMarker.header.stamp = ros::Time::now();
	//vizMarker.lifetime = ros::Duration(0.1);
	vizMarker.id = markerId;
	vizMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	vizMarker.action = visualization_msgs::Marker::ADD;
	vizMarker.scale.x = 1;
    vizMarker.scale.y = 1;
    vizMarker.scale.z = 1;
    vizMarker.color.a = 0.1;
    vizMarker.color.r = 1.0;
    vizMarker.color.g = 0.0;
    vizMarker.color.b = 0.0;
    vizMarker.pose.orientation = pose.orientation;
    vizMarker.pose.position = pose.position;
    
    geometry_msgs::Point p1; p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    geometry_msgs::Point p2; p2.x = 3.0; p2.y = -1.5; p2.z = 0.0;
    geometry_msgs::Point p3; p3.x = 3.0; p3.y = 1.5; p3.z = 0.0;
    
    vizMarker.points = {p1, p2, p3};

    markerArray.markers.push_back(vizMarker);
}

/* Delete navigation goals visualization markers */
void SemanticGoalsGenerator::deleteMarkers(){
    visualization_msgs::MarkerArray markerArray;

    for(int i = 0; i < markersLen_; i++){
        visualization_msgs::Marker vizMarker;

        vizMarker.header.frame_id = mapFrame_;
        vizMarker.id = i;
        vizMarker.action = visualization_msgs::Marker::DELETE;
        markerArray.markers.push_back(vizMarker);
    }
    visNavGoalsPub_.publish(markerArray);
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
