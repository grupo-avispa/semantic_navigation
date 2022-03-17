/*
 * SEMANTIC GOALS GENERATOR ROS NODE
 *
 * Copyright (c) 2020-2021 Alberto Jos� Tudela Rold�n <ajtudela@gmail.com>
 * 
 * This file is part of semantic_navigation.
 * 
 * All rights reserved.
 *
 */
 
#ifndef SEMANTICGOALSGENERATOR_H
#define SEMANTICGOALSGENERATOR_H

// C++
#include <cmath>
#include <random>
#include <string>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

#include <simple_laser_geometry/polygon.h>
#include "semantic_goals_generator/SemanticGoals.h"
#include "semantic_goals_generator/SemanticPosition.h"

struct ROI{
	slg::Polygon polygon;
	float yaw;

	bool empty(){return polygon.empty();};
	void clear(){return polygon.clear();};
	std::string getName(){return polygon.getName();};

	/* Check if a point is inside the region of interest (ROI) */
	bool inROI(float x, float y){
		if(polygon.size() == 0) return true;
		return polygon.contains(slg::Point2D(x,y));
	}

	/* Check if the point is at distance from all borders */
	bool disFromBorders(float x, float y, float border){
		for(slg::Edge edge: polygon.getEdges()){
			if(edge.distance(slg::Point2D(x,y)) < border) return false;
		}
		return true;
	}
};

class SemanticGoalsGenerator{
	public:
		SemanticGoalsGenerator(ros::NodeHandle& node, ros::NodeHandle& node_private);
		~SemanticGoalsGenerator();
	private:
		ros::NodeHandle node_, nodePrivate_;
		ros::Publisher navGoalsPub_, roisVizPub_, roisNamesVizPub_;
		ros::ServiceServer navGenSrv_, semanticPosSrv_;

		geometry_msgs::Pose origin_;
		bool isCostmap_, fullMap_;
		int width_, height_, inflatedFootprintSize_;
		int cellMinX_, cellMaxX_, cellMinY_, cellMaxY_;
		float bBoxMinX_, bBoxMaxX_, bBoxMinY_, bBoxMaxY_;
		float mapMinX_, mapMaxX_, mapMinY_, mapMaxY_;
		float resolution_, inflationRadius_, border_;
		std::string mapTopic_, direction_;
		std::vector<int8_t> mapData_;
		std::vector<ROI> roiList_;

		void getParams();
		bool SemanticGoalsService(semantic_goals_generator::SemanticGoals::Request& req, semantic_goals_generator::SemanticGoals::Response& res);
		bool SemanticPositionService(semantic_goals_generator::SemanticPosition::Request& req, semantic_goals_generator::SemanticPosition::Response& res);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgMap);
		std::vector<ROI> getROIParams();
		void showVisualization();
		void processBoundingBox(ROI roi);
		int cell(int x, int y);
		bool inCollision(int x, int y);
};
#endif
