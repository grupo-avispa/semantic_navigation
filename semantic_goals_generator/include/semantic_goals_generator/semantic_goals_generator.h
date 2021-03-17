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
 
#ifndef SEMANTICGOALSGENERATOR_H
#define SEMANTICGOALSGENERATOR_H
 
#include <cmath>
#include <random>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <laser_utils/polygon.h>
#include "semantic_goals_generator/SemanticGoals.h"
#include "semantic_goals_generator/SemanticPosition.h"

class SemanticGoalsGenerator{
	public:
		SemanticGoalsGenerator(ros::NodeHandle& node, ros::NodeHandle& node_private);
		~SemanticGoalsGenerator();
	private:
		ros::NodeHandle node_, nodePrivate_;
		ros::Publisher navGoalsPub_, roisVizPub_, roisNamesVizPub_;
		ros::ServiceServer paramsSrv_, navsGenSrv_, semanticPosSrv_;

		geometry_msgs::Pose origin_;
		bool isCostmap_;
		int width_, height_, inflatedFootprintSize_;
		int cellMinX_, cellMaxX_, cellMinY_, cellMaxY_;
		float bBoxMinX_, bBoxMaxX_, bBoxMinY_, bBoxMaxY_;
		float mapMinX_, mapMaxX_, mapMinY_, mapMaxY_;
		float resolution_, inflationRadius_, border_;
		std::string mapTopic_, orientation_;
		std::vector<int8_t> mapData_;
		Polygon roi_;
		std::vector<Polygon> roisList_;

		void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }
		bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool SemanticGoalsService(semantic_goals_generator::SemanticGoals::Request& req, semantic_goals_generator::SemanticGoals::Response& res);
		bool SemanticPositionService(semantic_goals_generator::SemanticPosition::Request& req, semantic_goals_generator::SemanticPosition::Response& res);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgMap);
		std::vector<Polygon> getROIParams();
		void showVisualization();
		void processBoundingBox();
		int cell(int x, int y);
		bool inROI(float x, float y);
		bool inCollision(int x, int y);
};
#endif
